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
#include "hydra/places/gvd_places/graph_extractor.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <glog/logging.h>
#include <glog/stl_logging.h>
#include <spark_dsg/edge_attributes.h>
#include <spatial_hash/neighbor_utils.h>

#include "hydra/places/gvd_places/graph_extractor_utilities.h"
#include "hydra/places/gvd_places/gvd_parent_tracker.h"
#include "hydra/utils/printing.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra::places {

using spark_dsg::EdgeAttributes;
using spark_dsg::EdgeKey;
using spark_dsg::NearestVertexInfo;
using spark_dsg::NodeId;
using spark_dsg::NodeSymbol;
using spark_dsg::PlaceNodeAttributes;
using timing::ScopedTimer;

namespace {

struct BestMemberResult {
  uint64_t id = 0;
  bool is_archived = false;
  const GvdMemberInfo* info = nullptr;

  operator bool() const { return info != nullptr; }
};

struct DeleteInfo {
  GlobalIndex index;
  uint64_t id;
};

NearestVertexInfo convertInfo(const GvdVertexInfo& parent_info) {
  NearestVertexInfo info;
  info.voxel_pos[0] = parent_info.pos[0];
  info.voxel_pos[1] = parent_info.pos[1];
  info.voxel_pos[2] = parent_info.pos[2];
  return info;
}

BestMemberResult getBestMember(const CompressedNode& node,
                               const GvdGraph& gvd,
                               const MergePolicy& policy) {
  BestMemberResult best_member;
  for (const auto node_id : node.active_refs) {
    auto curr_member = gvd.getNode(node_id);
    if (!best_member.info || policy.compare(*curr_member, *best_member.info) > 0) {
      best_member.info = curr_member;
      best_member.id = node_id;
    }
  }

  for (const auto node_id : node.archived_refs) {
    auto curr_member = gvd.getNode(node_id);
    if (!best_member.info || policy.compare(*curr_member, *best_member.info) > 0) {
      best_member.is_archived = true;
      best_member.info = curr_member;
      best_member.id = node_id;
    }
  }

  return best_member;
}

}  // namespace

void declare_config(GraphExtractor::Config::OverlapEdges& config) {
  using namespace config;
  name("OverlapEdgeConfig");
  field(config.enable, "enable");
  field(config.min_clearance_m, "min_clearance_m");
}

void declare_config(GraphExtractor::Config::FreespaceEdges& config) {
  using namespace config;
  name("FreespaceEdgeConfig");
  field(config.enable, "enable");
  field(config.max_length_m, "max_length_m");
  field(config.num_nodes_to_check, "num_nodes_to_check");
  field(config.num_neighbors_to_find, "num_neighbors_to_find");
  field(config.min_clearance_m, "min_clearance_m");
}

void declare_config(GraphExtractor::Config& config) {
  using namespace config;
  name("GraphExtractor::Config");
  field(config.prefix, "prefix");
  field(config.compression_distance_m, "compression_distance_m");
  field(config.min_node_distance_m, "min_node_distance_m");
  field(config.min_edge_distance_m, "min_edge_distance_m");
  field(config.merge_nearby_nodes, "merge_new_nodes");
  field(config.merge_policy, "merge_policy");
  field(config.node_merge_distance_m, "node_merge_distance_m");
  field(config.overlap_edges, "overlap_edges");
  field(config.freespace_edges, "freespace_edges");
}

GraphExtractor::GraphExtractor(const Config& config)
    : config(config),
      next_id_(0),
      compressed_(config.compression_distance_m),
      merge_policy_(config::create<MergePolicy>(config.merge_policy)) {}

GraphExtractor::~GraphExtractor() = default;

void GraphExtractor::pushIndex(const GlobalIndex& index) {
  modified_voxel_queue_.push(index);
}

void GraphExtractor::clearIndex(const GlobalIndex& index) {
  // gvd integrator update removes any invalidated voxels, so we update the support for
  // all of hte nodes, deleting nodes that no longer have any support.
  uint64_t gvd_id;
  std::set<uint64_t> gvd_siblings;
  {  // scope limiting iter lifetime to be valid
    auto iter = index_id_map_.find(index);
    if (iter == index_id_map_.end()) {
      return;
    }

    gvd_id = iter->second;
    gvd_.removeNode(iter->second);
    index_id_map_.erase(iter);
  }  // end iter scope

  const auto info = compressed_.remove(gvd_id, false);
  if (!info.id) {
    return;
  }

  const auto compressed_id = info.id.value();
  const NodeSymbol node_id(config.prefix, compressed_id);
  if (info.was_best_id) {
    // clear node id from active window until we assign a new member
    node_index_map_.erase(node_id);
  }

  for (const auto sibling : info.cleared_neighbors) {
    graph_.remove(node_id, NodeSymbol(config.prefix, sibling));
    // TODO(nathan) track deleted
  }

  if (info.has_active) {
    return;  // compressed node is still active; nothing else to do
  }

  if (!info.has_archived) {
    graph_.remove(node_id);
    // TODO(nathan) track deleted
  } else {
    to_archive_.insert(compressed_id);
  }
}

void GraphExtractor::archiveIndex(const GlobalIndex& index) {
  // 4. we archive all voxels, updating each compressed node
  //   a. we move entries from an active set to an archived set
  //   b. once there are no more entires in the active set, we flag as archived
  //   c. once all siblings are also archived, we delete?
  auto iter = index_id_map_.find(index);
  if (iter == index_id_map_.end()) {
    return;
  }

  // archived gvd nodes still can be accessed by their ID
  const auto gvd_id = iter->second;
  index_id_map_.erase(iter);
  const auto info = compressed_.remove(gvd_id, true);
  if (!info.id) {
    gvd_.removeNode(gvd_id);  // nothing in the compressed graph depends on the node
    return;
  }

  const auto compressed_id = info.id.value();
  const NodeSymbol node_id(config.prefix, compressed_id);
  if (info.was_best_id == gvd_id) {
    // clear node id from active window: current best point is archived
    node_index_map_.erase(node_id);
  }

  if (!info.has_active) {
    to_archive_.insert(compressed_id);
  }
}

void GraphExtractor::extract(const GvdLayer& layer, uint64_t timestamp_ns) {
  // 0. remove all archived nodes that no longer have active siblings
  clearArchived();

  // 1. get unique and valid voxels, and then update the gvd graph and compression
  IndexVoxelQueue seen_voxels;
  fillSeenVoxels(layer, timestamp_ns, seen_voxels);
  updateGvdGraph(layer, seen_voxels, timestamp_ns);

  // 2. we go through a list of updated compressed nodes and decide on a representative
  // set of attributes and edges
  //   a. we can't average attributes, so instead we pick a single candidate
  //   b. we could do centroid, but max(num_basis_points) might make more sense
  assignCompressedNodeAttributes();
  updateCompressedEdges(layer);

  // 3. we optionally merge nearby nodes
  if (config.merge_nearby_nodes) {
    mergeNearbyNodes();
  }

  compressed_.updated.clear();
  updateOverlapEdges();
  updateFreespaceEdges(layer);
}

void GraphExtractor::fillParentInfo(const GvdLayer& gvd,
                                    const GvdParentTracker& tracker) {
  for (const auto& [node_id, node_index] : node_index_map_) {
    auto attrs = graph_.at(node_id);
    attrs->voxblox_mesh_connections.clear();
    const auto* voxel = gvd.getVoxelPtr(node_index);
    if (!voxel) {
      // the compression-based extractor can have nodes pointing to archived voxels
      continue;
    }

    if (!tracker.parents.count(node_index)) {
      LOG(ERROR) << "bad gvd voxel: " << *voxel << " @ " << node_index.transpose();
      continue;
    }

    // save primary parent first
    const GlobalIndex curr_parent = voxel->parent;
    auto iter = tracker.parent_vertices.find(curr_parent);
    if (iter != tracker.parent_vertices.end()) {
      attrs->voxblox_mesh_connections.push_back(convertInfo(iter->second));
    }

    // save all other basis points
    for (const auto& parent : tracker.parents.at(node_index)) {
      if (!tracker.parent_vertices.count(parent)) {
        continue;
      }

      const auto& parent_info = tracker.parent_vertices.at(parent);
      attrs->voxblox_mesh_connections.push_back(convertInfo(parent_info));
    }
  }
}

void GraphExtractor::clearArchived() {
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
    for (const auto gvd_id : compressed_info_map_.at(id).archived_refs) {
      compressed_remapping_.erase(gvd_id);
      gvd_.removeNode(gvd_id);
    }

    clearCompressionId(id, false);
    archived_node_ids_.insert(id);
  }
}

void GraphExtractor::fillSeenVoxels(const GvdLayer& layer,
                                    uint64_t timestamp_ns,
                                    IndexVoxelQueue& seen_voxels) {
  ScopedTimer timer("places/prune_gvd_queue", timestamp_ns);

  GlobalIndexSet seen_indices;
  while (!modified_voxel_queue_.empty()) {
    const auto index = modified_voxel_queue_.front();
    modified_voxel_queue_.pop();
    if (seen_indices.count(index)) {
      continue;
    }

    const auto voxel = layer.getVoxelPtr(index);
    if (voxel == nullptr) {
      // this should only happen when we encounter voxels from archived blocks
      MLOG(1) << "Invalid index: " << showIndex(index) << " found in extraction queue";
      continue;
    }

    if (!voxel->num_extra_basis) {
      continue;
    }

    seen_indices.insert(index);
    seen_voxels.push_back({index, voxel});
  }
}

void GraphExtractor::updateGvdGraph(const GvdLayer& layer,
                                    const IndexVoxelQueue& update_info,
                                    uint64_t timestamp_ns) {
  ScopedTimer timer("places/update_gvd_graph", timestamp_ns);
  for (const auto& [index, voxel] : update_info) {
    auto iter = index_id_map_.find(index);
    if (iter == index_id_map_.end()) {
      const auto next_id = gvd_.addNode(layer.getVoxelPosition(index), index);
      iter = index_id_map_.emplace(index, next_id).first;
    }

    auto& info = *gvd_.getNode(iter->second);
    info.distance = voxel->distance;
    info.num_basis_points = voxel->num_extra_basis + 1;
  }

  std::list<DeleteInfo> to_delete;
  const spatial_hash::NeighborSearch search(26);
  for (const auto& [index, voxel] : update_info) {
    auto iter = index_id_map_.find(index);
    if (iter == index_id_map_.end()) {
      // if we prune the GVD, there might not be a one-to-one correspondence
      continue;
    }

    auto& info = *gvd_.getNode(iter->second);
    for (const auto& neighbor_index : search.neighborIndices(index)) {
      auto niter = index_id_map_.find(neighbor_index);
      if (niter == index_id_map_.end()) {
        continue;
      }

      info.siblings.insert(niter->second);
      gvd_.getNode(niter->second)->siblings.insert(iter->second);
    }

    if (info.siblings.empty()) {
      to_delete.push_back({index, iter->second});
      continue;  // isolated voxel
    }

    if (info.distance >= config.min_node_distance_m) {
      compressed_.add(iter->second, info);
    }
  }

  for (const auto& delete_info : to_delete) {
    clearIndex(delete_info.index);
  }
}

void GraphExtractor::assignCompressedNodeAttributes() {
  for (const auto& compressed_id : compressed_.updated) {
    auto& info = compressed_.nodes.at(compressed_id);
    auto result = getBestMember(info, gvd_, *merge_policy_);
    if (!result) {
      LOG(WARNING) << "Empty compressed node encountered: " << compressed_id;
      continue;
    }

    info.best_gvd_id = result.id;
    info.in_graph = true;
    const auto graph_id = NodeSymbol(config.prefix, compressed_id);
    if (!result.is_archived) {
      // avoid handing a bad index to graph compression
      node_index_map_[graph_id] = result.info->index;
    }

    auto attrs = std::make_unique<PlaceNodeAttributes>();
    attrs->distance = result.info->distance;
    attrs->num_basis_points = result.info->num_basis_points;
    attrs->position = result.info->position;
    graph_.update(graph_id, std::move(attrs));
  }
}

void GraphExtractor::updateCompressedEdges(const GvdLayer& layer) {
  for (const auto compressed_id : compressed_.updated) {
    auto& info = compressed_.nodes.at(compressed_id);
    const auto graph_id = NodeSymbol(config.prefix, compressed_id);
    if (!node_index_map_.count(graph_id)) {
      // this can happen when a node starts pointing to an archived gvd voxel
      continue;
    }

    for (const auto sibling : info.siblings) {
      const NodeSymbol sibling_id(config.prefix, sibling);
      if (!node_index_map_.count(sibling_id)) {
        continue;  // sibling edge can't be updated
      }

      auto attrs = getFreespaceEdgeInfo(graph_,
                                        layer,
                                        node_index_map_,
                                        graph_id,
                                        sibling_id,
                                        config.min_edge_distance_m,
                                        false);
      if (!attrs) {
        graph_.remove(graph_id, sibling_id);
        // TODO(nathan) track deleted
        continue;
      }

      const EdgeKey key{graph_id, sibling_id};
      graph_.update(graph_id, sibling_id, std::move(attrs));
      // override heuristic edges with actual graph edges
      overlap_edges_.erase(key);
      freespace_edges_.erase(key);
    }
  }
}

void GraphExtractor::mergeNearbyNodes() {
  std::unordered_map<uint64_t, uint64_t> merges;
  std::unordered_map<uint64_t, std::unordered_set<uint64_t>> reversed_merges;
  for (const auto& compressed_id : compressed_.updated) {
    if (merges.count(compressed_id)) {
      continue;
    }

    const auto& info = compressed_.nodes.at(compressed_id);
    if (!info.in_graph) {
      continue;
    }

    for (const auto sibling_id : info.siblings) {
      const auto& sibling_info = compressed_.nodes.at(sibling_id);
      if (!sibling_info.in_graph) {
        continue;
      }

      const auto gvd1 = gvd_.getNode(info.best_gvd_id);
      const auto gvd2 = gvd_.getNode(sibling_info.best_gvd_id);
      if ((gvd1->position - gvd2->position).norm() > config.node_merge_distance_m) {
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

  for (const auto& [from, to] : merges) {
    compressed_.nodes.at(from).in_graph = false;
    const NodeSymbol to_merge(config.prefix, from);
    node_index_map_.erase(to_merge);
    graph_.contract(to_merge, NodeSymbol(config.prefix, to));
  }
}

void GraphExtractor::updateOverlapEdges() {
  if (!config.overlap_edges.enable) {
    return;
  }

  for (auto iter = overlap_edges_.begin(); iter != overlap_edges_.end();) {
    const auto [source, target] = *iter;
    if (!node_index_map_.count(source) && !node_index_map_.count(target)) {
      // edge between archived nodes can be fixed
      iter = overlap_edges_.erase(iter);
    } else {
      ++iter;
    }
  }

  const auto threshold = config.overlap_edges.min_clearance_m;
  for (auto source = graph_.nodes().begin(); source != graph_.nodes().end(); ++source) {
    for (auto target = source; target != graph_.nodes().end(); ++target) {
      if (source == target) {
        continue;
      }

      EdgeKey key{source->first, target->first};
      if (graph_.has(key.k1, key.k2) && !overlap_edges_.count(key)) {
        continue;
      }

      auto info = getOverlapEdgeInfo(graph_, key.k1, key.k2, threshold);
      if (info) {
        graph_.update(key.k1, key.k2, std::move(info));
        overlap_edges_.insert(key);
      } else {
        overlap_edges_.erase(key);
        graph_.remove(key.k1, key.k2);
      }
    }
  }
}

void GraphExtractor::updateFreespaceEdges(const GvdLayer& gvd) {
  if (!config.freespace_edges.enable) {
    return;
  }

  for (auto iter = freespace_edges_.begin(); iter != freespace_edges_.end();) {
    const auto [source, target] = *iter;
    if (!node_index_map_.count(source) && !node_index_map_.count(target)) {
      iter = freespace_edges_.erase(iter);
      continue;
    }

    auto info = getFreespaceEdgeInfo(graph_,
                                     gvd,
                                     node_index_map_,
                                     source,
                                     target,
                                     config.freespace_edges.min_clearance_m,
                                     true);
    if (!info) {
      graph_.remove(source, target);
      iter = freespace_edges_.erase(iter);
    } else {
      graph_.update(source, target, std::move(info));
      ++iter;
    }
  }

  EdgeInfoMap proposed_edges;
  findFreespaceEdges(config.freespace_edges,
                     *graph_,
                     gvd,
                     active_nodes,
                     node_index_map_,
                     proposed_edges);

  for (auto& [key, info] : proposed_edges) {
    graph_.update(key.k1, key.k2, std::move(info));
    freespace_edges_.insert(key);
  }
}

void GraphExtractor::clearCompressionId(uint64_t node_id, bool is_delete) {
  to_archive_.erase(node_id);
  MLOG(2) << (is_delete ? "deleting " : "archiving ") << node_id;
}

}  // namespace hydra::places
