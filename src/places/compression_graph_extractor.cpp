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
#include "hydra/places/compression_graph_extractor.h"

#include <glog/stl_logging.h>
#include <spatial_hash/neighbor_utils.h>

#include "hydra/places/graph_extractor_utilities.h"
#include "hydra/places/nearest_voxel_utilities.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra::places {

using PlaceAttrs = PlaceNodeAttributes;
using timing::ScopedTimer;

struct DeleteInfo {
  GlobalIndex index;
  uint64_t id;
};

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
  // these both should be gauranteed to work, but we use find to make erase more
  // efficient
  auto& connections = sibling_support.at(gvd_id);
  auto iter = connections.find(neighbor_gvd_id);

  // first decrement the ref count for the sibling and drop the sibling if no longer
  // connected
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
  CHECK(!other.siblings.count(node_id))
      << "bad siblings for " << other.node_id << ": " << other.siblings << " @ "
      << node_id << " (siblings: " << siblings << ")";
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

CompressionGraphExtractor::CompressionGraphExtractor(
    const CompressionExtractorConfig& config)
    : GraphExtractorInterface(config),
      config_(config),
      merge_policy_(config::create<MergePolicy>(config.merge_policy)),
      next_id_(0) {
  compression_factor_ = 1.0 / config_.compression_distance_m;
}

CompressionGraphExtractor::~CompressionGraphExtractor() = default;

void CompressionGraphExtractor::clearGvdIndex(const GlobalIndex& index) {
  // gvd integrator update removes any invalidated voxels, deleting nodes that no
  // longer have any support. We remove any deleted entries from the active set, and
  // once the active set is empty, we delete the compressed node if the archived set is
  // empty, otherwise we archive.
  uint64_t gvd_id;
  std::set<uint64_t> gvd_siblings;
  {  // scope limiting iter lifetime to be valid
    auto iter = index_id_map_.find(index);
    if (iter == index_id_map_.end()) {
      return;
    }

    gvd_id = iter->second;
    gvd_->removeNode(iter->second);
    index_id_map_.erase(iter);
  }  // end iter scope

  // find compressed node
  uint64_t compressed_id;
  {  // scope limiting iter lifetime
    auto iter = compressed_remapping_.find(gvd_id);
    if (iter == compressed_remapping_.end()) {
      return;  // nothing to do if voxel wasn't compressed
    }

    compressed_id = iter->second;
    compressed_remapping_.erase(iter);
  }

  // remove current voxel from active refs
  auto& info = compressed_info_map_.at(compressed_id);
  const auto deleted_siblings =
      info.removeEdgeObservations(gvd_id, compressed_info_map_);
  for (const auto sibling : deleted_siblings) {
    removeGraphEdge(getPlaceId(compressed_id), getPlaceId(sibling));
  }

  if (info.best_gvd_id == gvd_id) {
    // clear node id from active window until we assign a new member as the
    // representative point
    node_index_map_.erase(getPlaceId(compressed_id));
  }

  info.active_refs.erase(gvd_id);
  updated_nodes_.insert(compressed_id);

  if (!info.active_refs.empty()) {
    // compressed node is still active; nothing else to do
    return;
  }

  if (info.archived_refs.empty()) {
    removeGraphNode(getPlaceId(compressed_id));
    clearCompressionId(compressed_id, true);
    id_queue_.push_back(compressed_id);  // free up id for other nodes to use
  } else {
    to_archive_.insert(compressed_id);
  }
}

void CompressionGraphExtractor::clearCompressionId(uint64_t node_id, bool is_delete) {
  // clear mapping between compression index and node id
  {
    // erase node entry from compression cell and erase compression cell if empty
    const auto& index = compressed_id_map_.at(node_id);
    auto iter = compressed_index_map_.find(index);
    CHECK(iter != compressed_index_map_.end());
    iter->second.erase(node_id);
    if (iter->second.empty()) {
      compressed_index_map_.erase(iter);
    }
  }
  compressed_id_map_.erase(node_id);

  // remove book-keeping around archive / update
  updated_nodes_.erase(node_id);
  to_archive_.erase(node_id);

  // erase sibilings before info gets erased
  const auto& info = compressed_info_map_.at(node_id);
  for (const auto sibling : info.siblings) {
    compressed_info_map_.at(sibling).siblings.erase(node_id);
  }

  compressed_info_map_.erase(node_id);
  VLOG(10) << (is_delete ? "deleting " : "archiving ") << node_id
           << " from compressed graph";
}

void CompressionGraphExtractor::deleteChildren(const CompressedNode& info) {
  for (const auto gvd_id : info.archived_refs) {
    compressed_remapping_.erase(gvd_id);
    gvd_->removeNode(gvd_id);
  }
}

void CompressionGraphExtractor::removeDistantIndex(const GlobalIndex& index) {
  // 4. we archive all voxels, updating each compressed node
  //   a. we move entries from an active set to an archived set
  //   b. once there are no more entires in the active set, we flag as archived
  //   c. once all siblings are also archived, we delete?
  auto iter = index_id_map_.find(index);
  if (iter == index_id_map_.end()) {
    return;
  }

  // we don't need the mapping between global index and id any more: archived gvd nodes
  // still can be accessed by their ID
  const auto gvd_id = iter->second;
  index_id_map_.erase(iter);

  auto citer = compressed_remapping_.find(gvd_id);
  if (citer == compressed_remapping_.end()) {
    // nothing in the compressed graph depends on this node, so delete it
    gvd_->removeNode(gvd_id);
    return;
  }

  auto& info = compressed_info_map_.at(citer->second);
  info.active_refs.erase(gvd_id);
  info.archived_refs.insert(gvd_id);

  if (info.best_gvd_id == gvd_id) {
    // clear node id from active window: current best point is archived
    node_index_map_.erase(getPlaceId(citer->second));
  }

  if (info.active_refs.empty()) {
    to_archive_.insert(citer->second);
  }
}

void CompressionGraphExtractor::fillSeenVoxels(const GvdLayer& layer,
                                               uint64_t timestamp_ns,
                                               IndexVoxelQueue& seen_voxels) {
  ScopedTimer timer("places/prune_gvd_queue", timestamp_ns);

  GlobalIndexSet seen_indices;
  while (!modified_voxel_queue_.empty()) {
    const GlobalIndex index = popFromModifiedQueue();
    if (seen_indices.count(index)) {
      continue;
    }

    const GvdVoxel* voxel = layer.getVoxelPtr(index);
    if (voxel == nullptr) {
      // this should only happen when we encounter voxels from archived blocks
      VLOG(10) << "[Graph Extraction] Invalid index: " << index.transpose()
               << " found in extraction input queue";
      continue;
    }

    // TODO(nathan) use num basis for extraction instead
    if (!voxel->num_extra_basis) {
      // it's possible for a voxel to labeled voronoi, then get visited (clearing the
      // voronoi flag)
      continue;
    }

    seen_indices.insert(index);
    seen_voxels.push_back({index, voxel});
  }
}

void CompressionGraphExtractor::clearArchived() {
  std::list<uint64_t> to_remove;
  for (const auto id : to_archive_) {
    auto& info = compressed_info_map_.at(id);
    bool can_be_archived = true;
    for (const auto sibling_id : info.siblings) {
      if (!to_archive_.count(sibling_id)) {
        can_be_archived = false;
        break;
      }
    }

    if (can_be_archived) {
      to_remove.push_back(id);
    }
  }

  for (const auto id : to_remove) {
    deleteChildren(compressed_info_map_.at(id));
    // TODO(nathan) not sure if we actually want to call this: we need a way to
    // distinguish between deleted edges and archived edges
    clearCompressionId(id, false);
    archived_node_ids_.insert(id);
  }
}

void CompressionGraphExtractor::extract(const GvdLayer& layer, uint64_t timestamp_ns) {
  // 0(a). remove all archived nodes that no longer have active siblings
  clearArchived();

  // 0(b). get unique and valid voxels
  IndexVoxelQueue seen_voxels;
  fillSeenVoxels(layer, timestamp_ns, seen_voxels);

  // 1. we update the full gvd graph with any new voxels from the queue from the
  // integrator (maybe eventually optionally applying thinning)
  updateNodeInfoMap(layer, seen_voxels, timestamp_ns);
  // 2. we update the connectivity of the gvd graph and update the compression for any
  // non-isolated voxels
  updateGvdGraph(seen_voxels, timestamp_ns);

  // 3. we go through a list of updated compressed nodes and decide on a representative
  // set of attributes
  //   a. we can't average attributes, so instead we pick a single candidate
  //   b. we could do centroid, but max(num_basis_points) might make more sense
  assignCompressedNodeAttributes();

  // 4. we assign edge attributes
  updateCompressedEdges(layer);

  // TODO(nathan) think about the order here a little more
  // 5. we optionally merge nearby nodes
  if (config_.merge_nearby_nodes) {
    mergeNearbyNodes();
  }
  updated_nodes_.clear();

  if (config_.add_heuristic_edges) {
    updateHeuristicEdges(layer);
  }

  if (config_.validate_graph) {
    validate(layer);
  }
}

void CompressionGraphExtractor::updateNode(const GlobalIndex& index,
                                           const Eigen::Vector3d& position,
                                           double distance,
                                           uint8_t num_basis_points) {
  auto iter = index_id_map_.find(index);
  if (iter == index_id_map_.end()) {
    const auto next_id = gvd_->addNode(position, index);
    iter = index_id_map_.emplace(index, next_id).first;
  }

  auto& info = *gvd_->getNode(iter->second);
  info.distance = distance;
  info.num_basis_points = num_basis_points;
}

void CompressionGraphExtractor::updateNodeInfoMap(const GvdLayer& layer,
                                                  const IndexVoxelQueue& update_info,
                                                  uint64_t timestamp_ns) {
  ScopedTimer timer("places/handle_gvd_queue", timestamp_ns);

  for (const auto& index_voxel_pair : update_info) {
    updateNode(index_voxel_pair.index,
               layer.getVoxelPosition(index_voxel_pair.index).cast<double>(),
               index_voxel_pair.voxel->distance,
               index_voxel_pair.voxel->num_extra_basis + 1);
  }
}

void CompressionGraphExtractor::updateGvdGraph(const IndexVoxelQueue& update_info,
                                               uint64_t timestamp_ns) {
  ScopedTimer timer("places/update_gvd_graph", timestamp_ns);
  std::list<DeleteInfo> to_delete;

  for (const auto& index_voxel_pair : update_info) {
    const auto& index = index_voxel_pair.index;
    auto iter = index_id_map_.find(index);
    if (iter == index_id_map_.end()) {
      // if we prune the GVD, there might not be a one-to-one correspondence
      continue;
    }

    auto& info = *gvd_->getNode(iter->second);
    spatial_hash::NeighborSearch search(26);
    for (const auto& neighbor_index : search.neighborIndices(index)) {
      auto niter = index_id_map_.find(neighbor_index);
      if (niter == index_id_map_.end()) {
        continue;
      }

      info.siblings.insert(niter->second);
      gvd_->getNode(niter->second)->siblings.insert(iter->second);
    }

    if (info.siblings.empty()) {
      to_delete.push_back({index, iter->second});
      continue;  // isolated voxel
    }

    if (info.distance >= config_.min_node_distance_m) {
      compressNode(iter->second, info);
    }
  }

  for (const auto& delete_info : to_delete) {
    clearGvdIndex(delete_info.index);
  }
}

GlobalIndex CompressionGraphExtractor::getCompressedIndex(
    const Eigen::Vector3d& point) const {
  const Eigen::Vector3d scaled_point = point * compression_factor_;
  return GlobalIndex(std::round(scaled_point(0)),
                     std::round(scaled_point(1)),
                     std::round(scaled_point(2)));
}

NodeId CompressionGraphExtractor::getPlaceId(uint64_t gvd_id) const {
  return NodeSymbol(next_node_id_.category(), gvd_id);
}

uint64_t CompressionGraphExtractor::getNextId() {
  uint64_t new_id;
  // if (id_queue_.empty()) {
  new_id = next_id_;
  next_id_++;
  //} else {
  // new_id = id_queue_.front();
  // id_queue_.pop_front();
  //}

  return new_id;
}

std::optional<uint64_t> CompressionGraphExtractor::findCluster(
    uint64_t node_id, const GvdMemberInfo& node, const std::set<uint64_t>& clusters) {
  for (const auto cluster : clusters) {
    CHECK(compressed_info_map_.count(cluster))
        << "missing cluster: " << cluster << " from clusters: " << clusters;
    const auto& info = compressed_info_map_.at(cluster);
    if (info.active_refs.count(node_id)) {
      // the current node can never be archived (has to be active to be visited by
      // compressNode)
      return cluster;
    }

    // we check if any siblings of this gvd voxel have been added to the compressed node
    // already
    // siblings can be archived (as they aren't necessarily visited by compressNode
    for (const auto sibling : node.siblings) {
      if (info.active_refs.count(sibling) || info.archived_refs.count(sibling)) {
        return cluster;
      }
    }
  }

  return std::nullopt;
}

void CompressionGraphExtractor::mergeCompressedNodes(uint64_t curr_node_id,
                                                     CompressedNode& curr_node,
                                                     uint64_t neighbor_node_id) {
  auto iter = compressed_info_map_.find(neighbor_node_id);
  for (const auto child : iter->second.active_refs) {
    compressed_remapping_[child] = curr_node_id;
  }

  for (const auto child : iter->second.archived_refs) {
    compressed_remapping_[child] = curr_node_id;
  }

  if (iter->second.in_graph) {
    const auto graph_id = getPlaceId(neighbor_node_id);
    removeGraphNode(graph_id);
    node_index_map_.erase(graph_id);
  }

  curr_node.merge(iter->second, compressed_info_map_);
  compressed_info_map_.erase(iter);
  compressed_id_map_.erase(neighbor_node_id);
  updated_nodes_.erase(neighbor_node_id);
  // neighbor probably shouldn't be a member of these, but...
  to_archive_.erase(neighbor_node_id);
  archived_node_ids_.erase(neighbor_node_id);
}

void CompressionGraphExtractor::compressNode(uint64_t node_id,
                                             const GvdMemberInfo& node) {
  // TODO(nathan) can maybe speed this up by looking up id->index mapping
  // get index via spatial hashing at (1 / desired_resolution)
  const auto index = getCompressedIndex(node.position);

  // add new hash to lookup if we don't have any nodes under the index
  auto iter = compressed_index_map_.find(index);
  if (iter == compressed_index_map_.end()) {
    iter = compressed_index_map_.emplace(index, std::set<uint64_t>()).first;
  }

  auto cluster = findCluster(node_id, node, iter->second);
  if (!cluster) {
    const auto next_id = getNextId();
    iter->second.insert(next_id);
    compressed_id_map_.emplace(next_id, index);
    cluster = next_id;

    compressed_info_map_.emplace(next_id, CompressedNode(next_id));
  }

  // always insert an active ref and add to updated (attributes might update regardless
  // if ref is new)
  auto& info = compressed_info_map_.at(*cluster);
  info.active_refs.insert(node_id);
  compressed_remapping_[node_id] = *cluster;
  updated_nodes_.insert(*cluster);

  // construct edges by checking to see if any uncompressed neighbors map to a different
  // compressed node
  for (const auto neighbor : node.siblings) {
    auto niter = compressed_remapping_.find(neighbor);
    if (niter == compressed_remapping_.end()) {
      continue;
    }

    if (niter->second == *cluster) {
      continue;
    }

    if (iter->second.count(niter->second)) {
      // voxel connected two neighboring clusters for the same index
      iter->second.erase(niter->second);
      mergeCompressedNodes(*cluster, info, niter->second);
      continue;
    }

    info.addEdgeObservation(node_id, neighbor, niter->second);
    compressed_info_map_.at(niter->second)
        .addEdgeObservation(neighbor, node_id, *cluster);
  }
}

void CompressionGraphExtractor::assignCompressedNodeAttributes() {
  for (const auto& compressed_id : updated_nodes_) {
    auto& info = compressed_info_map_.at(compressed_id);

    GvdMemberInfo* best_member = nullptr;
    uint64_t best_id = 0;
    for (const auto node_id : info.active_refs) {
      CHECK(gvd_->hasNode(node_id)) << "Failed to get active node " << node_id
                                    << " from compressed node " << compressed_id;
      auto curr_member = gvd_->getNode(node_id);
      CHECK(index_id_map_.count(curr_member->index))
          << "active node doesn't point to index";
      if (!best_member ||
          curr_member->num_basis_points > best_member->num_basis_points) {
        best_member = curr_member;
        best_id = node_id;
      }
    }

    bool best_is_archived = false;
    for (const auto node_id : info.archived_refs) {
      CHECK(gvd_->hasNode(node_id)) << "Failed to get archived node " << node_id
                                    << " from compressed node " << compressed_id;
      auto curr_member = gvd_->getNode(node_id);
      if (!best_member ||
          curr_member->num_basis_points > best_member->num_basis_points) {
        best_is_archived = true;
        best_member = curr_member;
        best_id = node_id;
      }
    }

    CHECK(best_member != nullptr);
    info.best_gvd_id = best_id;
    info.in_graph = true;

    const auto graph_id = getPlaceId(compressed_id);
    if (!best_is_archived) {
      CHECK(index_id_map_.count(best_member->index))
          << "best node doesn't point to index";
      // avoid handing a bad index to graph compression
      node_index_map_[graph_id] = best_member->index;
    }

    if (!getGraph().hasNode(graph_id)) {
      addGraphNode(graph_id);
    }

    auto& attrs = getGraph().getNode(graph_id).attributes<PlaceNodeAttributes>();
    attrs.distance = best_member->distance;
    attrs.num_basis_points = best_member->num_basis_points;
    attrs.position = best_member->position;
  }
}

void CompressionGraphExtractor::mergeNearbyNodes() {
  std::unordered_map<uint64_t, uint64_t> merges;
  std::unordered_map<uint64_t, std::unordered_set<uint64_t>> reversed_merges;
  for (const auto& compressed_id : updated_nodes_) {
    if (merges.count(compressed_id)) {
      continue;
    }

    const auto& info = compressed_info_map_.at(compressed_id);
    if (!info.in_graph) {
      continue;
    }

    for (const auto sibling_id : info.siblings) {
      const auto& sibling_info = compressed_info_map_.at(sibling_id);
      if (!sibling_info.in_graph) {
        continue;
      }

      const auto gvd1 = gvd_->getNode(info.best_gvd_id);
      const auto gvd2 = gvd_->getNode(sibling_info.best_gvd_id);
      if ((gvd1->position - gvd2->position).norm() > config_.node_merge_distance_m) {
        continue;
      }

      // assign merge to the node with the most basis points
      const auto lhs_is_better = merge_policy_->compare(*gvd1, *gvd2);
      const auto from_node = lhs_is_better >= 0 ? sibling_id : compressed_id;
      auto to_node = lhs_is_better >= 0 ? compressed_id : sibling_id;

      auto iter = merges.find(to_node);
      to_node = (iter == merges.end()) ? to_node : iter->second;

      merges[from_node] = to_node;
      auto riter = reversed_merges.find(to_node);
      if (riter == reversed_merges.end()) {
        riter = reversed_merges.emplace(to_node, std::unordered_set<uint64_t>()).first;
      }

      riter->second.insert(from_node);

      auto citer = reversed_merges.find(from_node);
      if (citer == reversed_merges.end()) {
        continue;
      }

      for (const auto child : citer->second) {
        merges[child] = to_node;
        riter->second.insert(child);
      }

      reversed_merges.erase(citer);
    }
  }

  for (const auto& from_to_pair : merges) {
    compressed_info_map_.at(from_to_pair.first).in_graph = false;
    const auto to_merge = getPlaceId(from_to_pair.first);
    node_index_map_.erase(to_merge);
    mergeGraphNodes(to_merge, getPlaceId(from_to_pair.second));
  }
}

void CompressionGraphExtractor::updateCompressedEdges(const GvdLayer& layer) {
  for (const auto& compressed_id : updated_nodes_) {
    auto& info = compressed_info_map_.at(compressed_id);
    const auto graph_id = getPlaceId(compressed_id);
    if (!node_index_map_.count(graph_id)) {
      // this can happen when a node starts pointing to an archived gvd voxel after a
      // call to assignCompressedNodeAttributes
      continue;
    }

    for (const auto sibling_id : info.siblings) {
      const auto sibling_graph_id = getPlaceId(sibling_id);
      if (!node_index_map_.count(sibling_graph_id)) {
        continue;  // sibling edge can't be updated
      }

      auto attrs = makeEdgeInfo(layer, graph_id, sibling_graph_id);
      if (attrs->weight < config_.min_edge_distance_m) {
        removeGraphEdge(graph_id, sibling_id);
        continue;
      }

      updateGraphEdge(graph_id, sibling_graph_id, std::move(attrs));
    }
  }
}

void CompressionGraphExtractor::validate(const GvdLayer& layer) const {
  const auto& graph = getGraph();
  for (const auto& id_node_pair : graph.nodes()) {
    const NodeSymbol node_id(id_node_pair.first);
    if (archived_node_ids_.count(node_id.categoryId())) {
      continue;
    }

    if (node_index_map_.count(node_id)) {
      const auto index = node_index_map_.at(node_id);
      CHECK(index_id_map_.count(index))
          << "unarchived node " << node_id.getLabel()
          << " points to archived index: " << index.transpose();
    }

    CHECK(compressed_info_map_.count(node_id.categoryId()))
        << "unarchived node " << node_id.getLabel() << " missing from info map";
    const auto& info = compressed_info_map_.at(node_id.categoryId());

    for (const auto sibling : info.siblings) {
      if (!compressed_info_map_.count(sibling)) {
        CHECK(archived_node_ids_.count(sibling))
            << "missing sibling: " << sibling << " from node " << node_id.getLabel();
      }
    }

    std::list<uint64_t> children;
    for (const auto child : info.active_refs) {
      children.push_back(child);
      const auto vertex = gvd_->getNode(child);
      const auto voxel = layer.getVoxelPtr(vertex->index);
      CHECK_NOTNULL(voxel);
      CHECK_GT(voxel->num_extra_basis, 0)
          << " invalid child " << child << " voxel " << *voxel << " @ "
          << vertex->index.transpose();
    }

    for (const auto child : info.archived_refs) {
      children.push_back(child);
    }

    for (const auto child : children) {
      CHECK_EQ(compressed_remapping_.at(child), node_id.categoryId());
    }

    CHECK(!children.empty()) << "invalid node: " << node_id.getLabel();

    std::set<uint64_t> gvd_neighbors;
    for (const auto child : children) {
      CHECK(gvd_->hasNode(child))
          << node_id.getLabel() << " points to missing child " << child;
      auto child_info = gvd_->getNode(child);
      for (const auto neighbor : child_info->siblings) {
        gvd_neighbors.insert(neighbor);
      }
    }

    std::set<uint64_t> neighbors;
    for (const auto neighbor : gvd_neighbors) {
      CHECK(compressed_remapping_.count(neighbor))
          << neighbor << " missing from compressed remapping";
      const auto compressed_neighbor = compressed_remapping_.at(neighbor);
      if (compressed_neighbor != node_id.categoryId()) {
        neighbors.insert(compressed_neighbor);
      }
    }

    for (const auto neighbor : neighbors) {
      if (archived_node_ids_.count(neighbor)) {
        continue;
      }

      const auto index = compressed_id_map_.at(neighbor);
      CHECK_NE(index, compressed_id_map_.at(node_id.categoryId()))
          << "unmerged nodes found!";
    }

    CHECK_EQ(neighbors, info.siblings)
        << "current siblings don't agree for " << node_id.getLabel();

    std::set<uint64_t> graph_siblings;
    for (const auto sibling : id_node_pair.second->siblings()) {
      graph_siblings.insert(NodeSymbol(sibling).categoryId());
    }

    auto iter = graph_siblings.begin();
    while (iter != graph_siblings.end()) {
      if (archived_node_ids_.count(*iter)) {
        iter = graph_siblings.erase(iter);
      } else {
        CHECK(compressed_info_map_.count(*iter))
            << "graph edge for " << node_id.getLabel()
            << " points to invalid index: " << *iter;
        ++iter;
      }
    }
  }
}

}  // namespace hydra::places
