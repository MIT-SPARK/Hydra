/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra/places/gvd_places/compressed_node.h"

#include "hydra/places/gvd_places/gvd_graph.h"

namespace hydra::places {

CompressedNode::CompressedNode(uint64_t id) : node_id(id) {}

void CompressedNode::addEdgeObservation(uint64_t gvd_id,
                                        uint64_t neighbor_gvd_id,
                                        uint64_t sibling_id) {
  auto iter = sibling_support.find(gvd_id);
  if (iter == sibling_support.end()) {
    iter = sibling_support.emplace(gvd_id, std::map<uint64_t, uint64_t>()).first;
  }

  auto riter = sibling_ref_counts.find(sibling_id);
  if (riter == sibling_ref_counts.end()) {
    siblings.insert(sibling_id);
    riter = sibling_ref_counts.emplace(sibling_id, 0).first;
  }

  if (iter->second.emplace(neighbor_gvd_id, sibling_id).second) {
    riter->second++;  // increment ref count for a new observation
  }
}

bool CompressedNode::removeEdgeObservation(uint64_t gvd_id, uint64_t neighbor_gvd_id) {
  // these both should work, but we use find to make erase more efficient
  auto& connections = sibling_support.at(gvd_id);
  auto iter = connections.find(neighbor_gvd_id);

  // decrement the ref count for the sibling and drop the sibling if no longer connected
  auto& ref_count = sibling_ref_counts.at(iter->second);
  ref_count--;
  bool deleted_sibling = ref_count == 0;
  if (deleted_sibling) {
    siblings.erase(iter->second);
    sibling_ref_counts.erase(iter->second);
  }

  // clean up connection tracking
  connections.erase(iter);
  if (connections.empty()) {
    sibling_support.erase(gvd_id);
  }

  return deleted_sibling;
}

std::list<uint64_t> CompressedNode::removeEdgeObservations(uint64_t gvd_id,
                                                           CompressedNodeMap& nodes) {
  auto iter = sibling_support.find(gvd_id);
  if (iter == sibling_support.end()) {
    return {};
  }

  std::list<uint64_t> to_remove;
  for (const auto& edge_pairs : iter->second) {
    auto& neighbor = nodes.at(edge_pairs.second);
    // when removing an observation from another node, we flip the order of the
    // underlying gvd voxels
    neighbor.removeEdgeObservation(edge_pairs.first, gvd_id);
    to_remove.push_back(edge_pairs.first);
  }

  // removeEdgeObservation changes the underlying sibling_support data structure, so we
  // need to erase internal observations separately. Note that this also clear gvd_id
  // from sibling_support
  std::list<uint64_t> removed_siblings;
  for (const auto neighbor_id : to_remove) {
    const auto sibling_id = iter->second.at(neighbor_id);
    if (removeEdgeObservation(gvd_id, neighbor_id)) {
      removed_siblings.push_back(sibling_id);
    }
  }

  return removed_siblings;
}

void CompressedNode::mergeObservations(uint64_t original_id, uint64_t new_id) {
  for (auto& id_observation_pair : sibling_support) {
    for (auto& id_pair : id_observation_pair.second) {
      if (id_pair.second == original_id) {
        id_pair.second = new_id;
      }
    }
  }

  auto iter = sibling_ref_counts.find(new_id);
  if (iter == sibling_ref_counts.end()) {
    iter = sibling_ref_counts.emplace(new_id, 0).first;
  }

  iter->second += sibling_ref_counts.at(original_id);
  sibling_ref_counts.erase(original_id);
  siblings.erase(original_id);
  siblings.insert(new_id);
}

void CompressedNode::merge(CompressedNode& other, CompressedNodeMap& nodes) {
  siblings.insert(other.siblings.begin(), other.siblings.end());
  active_refs.insert(other.active_refs.begin(), other.active_refs.end());
  archived_refs.insert(other.archived_refs.begin(), other.archived_refs.end());
  sibling_support.insert(other.sibling_support.begin(), other.sibling_support.end());
  for (const auto& id_count_pair : other.sibling_ref_counts) {
    auto iter = sibling_ref_counts.find(id_count_pair.first);
    if (iter == sibling_ref_counts.end()) {
      sibling_ref_counts.emplace(id_count_pair.first, id_count_pair.second);
    } else {
      iter->second += id_count_pair.second;
    }
  }

  for (auto sibling : other.siblings) {
    nodes.at(sibling).mergeObservations(other.node_id, node_id);
  }
}

CompressedGraph::CompressedGraph(float resolution_m) : next_id(0), grid(resolution_m) {}

std::list<uint64_t> CompressedGraph::add(uint64_t gvd_id, const GvdMemberInfo& node) {
  const auto index = grid.toIndex(node.position);
  auto iter = index_map.find(index);
  if (iter == index_map.end()) {
    // add new hash to lookup if we don't have any nodes under the index
    iter = index_map.emplace(index, std::set<uint64_t>()).first;
  }

  std::optional<uint64_t> cluster;
  for (const auto curr_id : iter->second) {
    const auto& info = nodes.at(curr_id);
    if (info.active_refs.count(gvd_id)) {
      cluster = curr_id;
      break;
    }

    for (const auto sibling : node.siblings) {
      if (info.active_refs.count(sibling) || info.archived_refs.count(sibling)) {
        cluster = curr_id;
        break;
      }
    }
  }

  if (!cluster) {
    iter->second.insert(next_id);
    id_map.emplace(next_id, index);
    nodes.emplace(next_id, CompressedNode(next_id));
    cluster = next_id;
    ++next_id;
  }

  // always insert an active ref and add to updated (attributes might update regardless
  // if ref is new)
  auto& info = nodes.at(*cluster);
  info.active_refs.insert(gvd_id);
  remapping[gvd_id] = *cluster;
  updated.insert(*cluster);

  // construct edges by checking to see if any uncompressed neighbors map to a different
  // compressed node
  std::list<uint64_t> removed_graph_ids;
  for (const auto neighbor : node.siblings) {
    auto niter = remapping.find(neighbor);
    if (niter == remapping.end()) {
      continue;
    }

    if (niter->second == *cluster) {
      continue;
    }

    if (iter->second.count(niter->second)) {
      // voxel connected two neighboring clusters for the same index
      iter->second.erase(niter->second);
      merge(*cluster, info, niter->second);
      removed_graph_ids.push_back(niter->second);
      continue;
    }

    info.addEdgeObservation(gvd_id, neighbor, niter->second);
    nodes.at(niter->second).addEdgeObservation(neighbor, gvd_id, *cluster);
  }

  return removed_graph_ids;
}

CompressedGraph::DeleteResult CompressedGraph::remove(uint64_t gvd_id,
                                                      bool is_archive) {
  uint64_t node_id;
  {  // scope limiting iter lifetime
    auto iter = remapping.find(gvd_id);
    if (iter == remapping.end()) {
      return {};  // nothing to do if voxel wasn't compressed
    }

    node_id = iter->second;
    remapping.erase(iter);
  }

  auto iter = nodes.find(node_id);
  auto& node = iter->second;

  DeleteResult result;
  result.id = node_id;
  result.was_best_id = gvd_id == node.best_gvd_id;
  result.cleared_neighbors = node.removeEdgeObservations(gvd_id, nodes);

  node.active_refs.erase(gvd_id);
  if (is_archive) {
    node.archived_refs.insert(gvd_id);
  }

  result.has_active = node.active_refs.empty();
  result.has_archived = !node.archived_refs.empty();
  if (result.has_active) {
    return result;
  }

  // clear mapping between compression index and node id
  {  // erase node entry from compression cell and erase compression cell
    const auto& index = id_map.at(node_id);
    auto iter = index_map.find(index);
    iter->second.erase(node_id);
    if (iter->second.empty()) {
      index_map.erase(iter);
    }
  }

  id_map.erase(node_id);
  for (const auto sibling : node.siblings) {
    nodes.at(sibling).siblings.erase(node_id);
  }

  nodes.erase(iter);
  return result;
}

void CompressedGraph::merge(uint64_t curr_node_id,
                            CompressedNode& curr_node,
                            uint64_t neighbor_node_id) {
  auto iter = nodes.find(neighbor_node_id);
  for (const auto child : iter->second.active_refs) {
    remapping[child] = curr_node_id;
  }

  for (const auto child : iter->second.archived_refs) {
    remapping[child] = curr_node_id;
  }

  curr_node.merge(iter->second, nodes);
  id_map.erase(neighbor_node_id);
  updated.erase(neighbor_node_id);
  nodes.erase(iter);
}

}  // namespace hydra::places
