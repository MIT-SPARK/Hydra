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
#include "hydra/places/gvd_places/gvd_graph.h"

#include <glog/logging.h>

namespace hydra::places {

using Node = GvdGraph::Node;
using Nodes = GvdGraph::Nodes;
using CompressedNode = GvdGraph::CompressedNode;
using CompressedNodes = GvdGraph::CompressedNodes;

namespace {

std::vector<std::vector<uint64_t>> getComponentsForNode(const CompressedNode& node,
                                                        const Nodes& uncompressed) {
  std::set<uint64_t> children(node.active_refs.begin(), node.active_refs.end());
  children.insert(node.archived_refs.begin(), node.archived_refs.end());

  std::set<uint64_t> visited;
  std::vector<std::vector<uint64_t>> components;
  for (const auto& seed : children) {
    if (visited.count(seed)) {
      continue;
    }

    visited.insert(seed);
    std::deque<uint64_t> frontier{seed};
    auto& component = components.emplace_back();
    while (!frontier.empty()) {
      const auto gvd_id = frontier.front();
      component.push_back(gvd_id);
      frontier.pop_front();

      const auto& gvd_node = uncompressed.at(gvd_id);
      for (const auto sibling : gvd_node.siblings) {
        if (!children.count(sibling)) {
          continue;  // don't move to other blocks
        }

        if (!visited.count(sibling)) {
          frontier.push_back(sibling);
          visited.insert(sibling);
        }
      }
    }
  }

  std::sort(components.begin(), components.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.size() > rhs.size();
  });

  return components;
}

}  // namespace

CompressedNode::CompressedNode(uint64_t id) : node_id(id) {}

std::vector<uint64_t> CompressedNode::refs() const {
  std::vector<uint64_t> all_refs(active_refs.begin(), active_refs.end());
  all_refs.insert(all_refs.end(), archived_refs.begin(), archived_refs.end());
  return all_refs;
}

bool CompressedNode::archived() const {
  return active_refs.empty() && !archived_refs.empty();
}

GvdGraph::GvdGraph(float voxel_resolution_m, float compression_resolution_m)
    : voxel_grid(voxel_resolution_m),
      compression_grid(compression_resolution_m),
      neighbor_search(26),
      next_uncompressed_id_(0),
      next_compressed_id_(0) {}

void GvdGraph::add(const GlobalIndex& vindex, double distance, uint8_t basis) {
  auto node = add_uncompressed(vindex, distance, basis);
  if (!node) {
    return;
  }

  for (const auto& n_idx : neighbor_search.neighborIndices(vindex)) {
    auto neighbor = uncompressed_by_index(n_idx);
    if (!neighbor) {
      continue;
    }

    neighbor->siblings.insert(node->id);
    node->siblings.insert(neighbor->id);
  }

  // always insert an active ref and add to updated
  const auto cluster = assign_to_cluster(*node);

  // construct edges by checking to see if any neighbors map to a different cluster
  auto& cell_clusters = compressed_index_map_.at(compressed_id_map_.at(cluster));
  for (const auto neighbor : node->siblings) {
    const auto neighbor_cluster = cluster_for_gvd(neighbor);
    if (!neighbor_cluster || neighbor_cluster == cluster) {
      continue;
    }

    if (cell_clusters.count(*neighbor_cluster)) {
      // voxel connected two neighboring clusters for the same index
      merge_clusters(cluster, *neighbor_cluster);
      cell_clusters.erase(*neighbor_cluster);
      continue;
    }

    compressed_.at(cluster).siblings.insert(*neighbor_cluster);
    compressed_.at(*neighbor_cluster).siblings.insert(cluster);
  }
}

void GvdGraph::remove(const GlobalIndexSet& indices) {
  std::set<uint64_t> updated;
  for (const auto& index : indices) {
    remove(index, updated);
  }

  std::vector<uint64_t> nodes_to_check;
  for (const auto compressed_id : updated) {
    auto& node = compressed_.at(compressed_id);
    nodes_to_check.push_back(compressed_id);

    // clear old connections to invalidated nodes
    for (const auto sibling : node.siblings) {
      compressed_.at(sibling).siblings.erase(compressed_id);
    }
    node.siblings.clear();

    const auto components = getComponentsForNode(node, uncompressed_);

    // iterate through and construct any new components that have split off from the
    // main component
    const auto index = compressed_id_map_.at(compressed_id);
    auto& curr_cell_clusters = compressed_index_map_.at(index);
    for (size_t i = 1; i < components.size(); ++i) {
      const auto new_split_id = next_compressed_id();
      nodes_to_check.push_back(new_split_id);
      curr_cell_clusters.insert(new_split_id);
      compressed_id_map_.emplace(new_split_id, index);

      auto iter = compressed_.emplace(new_split_id, CompressedNode(new_split_id)).first;
      for (const auto new_child : components[i]) {
        const auto active = node.active_refs.count(new_child);
        compression_map_[new_child] = new_split_id;
        if (active) {
          node.active_refs.erase(new_child);
          iter->second.active_refs.insert(new_child);
        } else {
          node.archived_refs.erase(new_child);
          iter->second.archived_refs.insert(new_child);
        }
      }
    }
  }

  // for every updated node or newly generated split, recompute siblings from the gvd
  for (const auto& compressed_id : nodes_to_check) {
    auto& node = compressed_.at(compressed_id);
    const auto children = node.refs();
    for (const auto& child : children) {
      for (const auto& sibling : uncompressed_.at(child).siblings) {
        const auto sibling_cluster = compression_map_.at(sibling);
        if (sibling_cluster != compressed_id) {
          node.siblings.insert(sibling_cluster);
          compressed_.at(sibling_cluster).siblings.insert(compressed_id);
        }
      }
    }
  }
}

void GvdGraph::archive(const GlobalIndex& index) {
  const auto gvd_id = drop_uncompressed(index);
  if (!gvd_id) {
    return;
  }

  uncompressed_.at(*gvd_id).archived = true;

  // we move entries from an active set to an archived set
  auto compressed_id = compression_map_.at(*gvd_id);
  auto& node = compressed_.at(compressed_id);
  node.active_refs.erase(*gvd_id);
  node.archived_refs.insert(*gvd_id);
}

void GvdGraph::remove(const GlobalIndex& index, std::set<uint64_t>& updated) {
  const auto maybe_gvd_id = drop_uncompressed(index);
  if (!maybe_gvd_id) {
    return;
  }

  // clean up uncompressed node information
  const auto gvd_id = *maybe_gvd_id;
  drop_uncompressed_node(gvd_id);

  const auto compressed_id = compression_map_.at(gvd_id);
  compression_map_.erase(gvd_id);

  auto iter = compressed_.find(compressed_id);
  CHECK(iter != compressed_.end()) << "Invalid compressed ID " << compressed_id;

  auto& node = iter->second;
  node.active_refs.erase(gvd_id);
  if (!node.active_refs.empty() || !node.archived_refs.empty()) {
    // mark node for cleanup as long as it still points to valid gvd nodes
    updated.insert(compressed_id);
    return;
  }

  // delete compressed node and clear from nodes to process
  drop_compressed_id(compressed_id);
  for (const auto sibling : node.siblings) {
    compressed_.at(sibling).siblings.erase(compressed_id);
  }

  compressed_.erase(iter);
  updated.erase(compressed_id);
}

void GvdGraph::dropCompressed(uint64_t compressed_id) {
  auto iter = compressed_.find(compressed_id);
  if (iter == compressed_.end()) {
    return;
  }

  const auto refs = iter->second.refs();
  for (const auto gvd_id : refs) {
    auto niter = uncompressed_.find(gvd_id);
    CHECK(niter != uncompressed_.end()) << "Invalid uncompressed ID " << gvd_id;
    drop_uncompressed(niter->second.info.index);
    drop_uncompressed_node(gvd_id);
  }

  for (const auto sibling : iter->second.siblings) {
    compressed_.at(sibling).siblings.erase(compressed_id);
  }

  drop_compressed_id(compressed_id);
  compressed_.erase(iter);
}

std::vector<uint64_t> GvdGraph::clearArchived() {
  std::vector<uint64_t> to_archive;
  for (const auto& [id, node] : compressed_) {
    if (!node.archived()) {
      continue;
    }

    bool can_be_archived = true;
    for (const auto sibling_id : node.siblings) {
      const auto& sibling = compressed_.at(sibling_id);
      if (!sibling.archived()) {
        can_be_archived = false;
        break;
      }
    }

    if (can_be_archived) {
      to_archive.push_back(id);
    }
  }

  for (const auto id : to_archive) {
    dropCompressed(id);
  }

  return to_archive;
}

const GvdMemberInfo* GvdGraph::get(uint64_t node) const {
  auto iter = uncompressed_.find(node);
  return iter == uncompressed_.end() ? nullptr : &iter->second.info;
}

GvdGraph::CompressedAttr GvdGraph::getCompressed(uint64_t compressed_id,
                                                 const MergePolicy& policy) const {
  auto iter = compressed_.find(compressed_id);
  if (iter == compressed_.end()) {
    return {};
  }

  GvdGraph::CompressedAttr result;
  for (const auto node_id : iter->second.active_refs) {
    const auto& gvd_node = uncompressed_.at(node_id);
    if (!result.info || policy.compare(gvd_node.info, *result.info) > 0) {
      result.info = &gvd_node.info;
    }
  }

  for (const auto node_id : iter->second.archived_refs) {
    const auto& gvd_node = uncompressed_.at(node_id);
    if (!result.info || policy.compare(gvd_node.info, *result.info) > 0) {
      result.info = &gvd_node.info;
      result.is_archived = true;
    }
  }

  return result;
}

const Nodes& GvdGraph::uncompressed() const { return uncompressed_; }

const CompressedNodes& GvdGraph::compressed() const { return compressed_; }

const GvdGraph::NodeRemapping& GvdGraph::remapping() const { return compression_map_; }

uint64_t GvdGraph::next_uncompressed_id() {
  uint64_t new_id;
  if (uncompressed_id_queue_.empty()) {
    new_id = next_uncompressed_id_;
    next_uncompressed_id_++;
  } else {
    new_id = uncompressed_id_queue_.front();
    uncompressed_id_queue_.pop_front();
  }

  return new_id;
}

uint64_t GvdGraph::next_compressed_id() {
  // TODO(nathan) make circular queue once backend works better
  uint64_t new_id = next_compressed_id_;
  ++next_compressed_id_;
  return new_id;
}

Node* GvdGraph::add_uncompressed(const GlobalIndex& index, double dist, uint8_t basis) {
  auto iter = uncompressed_index_map_.find(index);
  if (iter != uncompressed_index_map_.end()) {
    auto& node = uncompressed_.at(iter->second);
    node.info.distance = dist;
    node.info.num_basis_points = basis;
    return nullptr;
  }

  const auto pos = voxel_grid.toPoint(index);
  const auto id = next_uncompressed_id();
  CHECK(!uncompressed_.count(id)) << "Re-using uncompressed ID";
  uncompressed_index_map_.emplace(index, id);

  auto niter = uncompressed_.emplace(id, Node{id, {dist, basis, pos, index}, {}}).first;
  return &niter->second;
}

Node* GvdGraph::uncompressed_by_index(const GlobalIndex& index) {
  auto iter = uncompressed_index_map_.find(index);
  return iter == uncompressed_index_map_.end() ? nullptr
                                               : &uncompressed_.at(iter->second);
}

uint64_t GvdGraph::assign_to_cluster(const Node& node) {
  const auto index = compression_grid.toIndex(node.info.position);
  auto iter = compressed_index_map_.find(index);
  if (iter == compressed_index_map_.end()) {
    // add new hash to lookup if we don't have any nodes under the index
    iter = compressed_index_map_.emplace(index, std::set<uint64_t>{}).first;
  }

  std::optional<uint64_t> compressed_id;
  for (const auto& sibling : node.siblings) {
    const auto candidate = compression_map_.at(sibling);
    if (iter->second.count(candidate)) {
      compressed_id = candidate;
      break;
    }
  }

  if (!compressed_id) {
    compressed_id = next_compressed_id();
    iter->second.insert(*compressed_id);
    compressed_id_map_.emplace(*compressed_id, index);
    compressed_.emplace(*compressed_id, CompressedNode(*compressed_id));
  }

  auto& info = compressed_.at(*compressed_id);
  info.active_refs.insert(node.id);
  compression_map_[node.id] = *compressed_id;
  return *compressed_id;
}

std::optional<uint64_t> GvdGraph::cluster_for_gvd(uint64_t gvd_id) const {
  auto iter = compression_map_.find(gvd_id);
  return iter != compression_map_.end() ? std::optional<uint64_t>(iter->second)
                                        : std::nullopt;
}

void GvdGraph::merge_clusters(uint64_t target, uint64_t candidate) {
  auto iter = compressed_.find(candidate);
  auto& candidate_node = iter->second;
  auto& target_node = compressed_.at(target);
  for (const auto child : candidate_node.active_refs) {
    target_node.active_refs.insert(child);
    compression_map_[child] = target;
  }

  for (const auto child : candidate_node.archived_refs) {
    target_node.archived_refs.insert(child);
    compression_map_[child] = target;
  }

  for (const auto sibling_id : candidate_node.siblings) {
    target_node.siblings.insert(sibling_id);
    auto& sibling = compressed_.at(sibling_id);
    sibling.siblings.erase(candidate);
    sibling.siblings.insert(target);
  }

  compressed_id_map_.erase(candidate);
  compressed_.erase(iter);
}

std::optional<uint64_t> GvdGraph::drop_uncompressed(const GlobalIndex& idx) {
  auto iter = uncompressed_index_map_.find(idx);
  if (iter == uncompressed_index_map_.end()) {
    return std::nullopt;
  }

  const auto gvd_id = iter->second;
  uncompressed_index_map_.erase(iter);
  return gvd_id;
}

void GvdGraph::drop_uncompressed_node(uint64_t gvd_id) {
  auto iter = uncompressed_.find(gvd_id);
  if (iter == uncompressed_.end()) {
    LOG(WARNING) << "Dropping node " << gvd_id << " but doesn't exist";
    return;
  }

  for (const auto sibling : iter->second.siblings) {
    uncompressed_.at(sibling).siblings.erase(gvd_id);
  }

  uncompressed_id_queue_.push_back(gvd_id);
  uncompressed_.erase(iter);
}

void GvdGraph::drop_compressed_id(uint64_t compressed_id) {
  const auto id_iter = compressed_id_map_.find(compressed_id);
  auto iter = compressed_index_map_.find(id_iter->second);
  iter->second.erase(compressed_id);
  if (iter->second.empty()) {
    compressed_index_map_.erase(iter);
  }

  compressed_id_map_.erase(id_iter);
}

}  // namespace hydra::places
