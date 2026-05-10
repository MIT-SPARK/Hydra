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

using spark_dsg::EdgeKey;
using spark_dsg::NearestVertexInfo;
using spark_dsg::NodeId;
using spark_dsg::NodeSymbol;
using spark_dsg::PlaceNodeAttributes;
using timing::ScopedTimer;

namespace {

NearestVertexInfo convertInfo(const GvdVertexInfo& parent_info) {
  NearestVertexInfo info;
  info.voxel_pos[0] = parent_info.pos[0];
  info.voxel_pos[1] = parent_info.pos[1];
  info.voxel_pos[2] = parent_info.pos[2];
  return info;
}

void fillParentInfo(const GvdLayer& layer,
                    const GvdParentTracker& tracker,
                    const GlobalIndex& index,
                    PlaceNodeAttributes& attrs) {
  const auto* voxel = layer.getVoxelPtr(index);
  if (!voxel) {
    // the compression-based extractor can have nodes pointing to archived voxels
    return;
  }

  if (!tracker.parents.count(index)) {
    LOG(ERROR) << "bad gvd voxel: " << *voxel << " @ " << index.transpose();
    return;
  }

  // save primary parent first
  attrs.voxblox_mesh_connections.clear();
  const GlobalIndex curr_parent = voxel->parent;
  auto iter = tracker.parent_vertices.find(curr_parent);
  if (iter != tracker.parent_vertices.end()) {
    attrs.voxblox_mesh_connections.push_back(convertInfo(iter->second));
  }

  // save all other basis points
  for (const auto& parent : tracker.parents.at(index)) {
    if (!tracker.parent_vertices.count(parent)) {
      continue;
    }

    const auto& parent_info = tracker.parent_vertices.at(parent);
    attrs.voxblox_mesh_connections.push_back(convertInfo(parent_info));
  }
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

GraphExtractor::GraphExtractor(const Config& config, float voxel_size)
    : config(config),
      gvd_(voxel_size, config.compression_distance_m),
      merge_policy_(config::create<MergePolicy>(config.merge_policy)) {}

GraphExtractor::~GraphExtractor() = default;

void GraphExtractor::pushIndex(const GlobalIndex& index) {
  modified_voxel_queue_.push(index);
}

void GraphExtractor::clearIndex(const GlobalIndex& index) {
  removed_voxel_queue_.push(index);
}

void GraphExtractor::archiveIndex(const GlobalIndex& index) {
  archived_voxel_queue_.push(index);
}

void GraphExtractor::extract(uint64_t timestamp_ns,
                             const GvdLayer& layer,
                             const GvdParentTracker& tracker) {
  // process index updates from the gvd integration
  gvd_.remove(removed_voxel_queue_);
  gvd_.archive(archived_voxel_queue_);

  // Remove all archived nodes that no longer have active siblings
  clearArchived();

  // Update the gvd graph and compression with unique voxels
  updateGvdGraph(timestamp_ns, layer);

  // Copy all the updates from the compressed GVD graph to the partial update
  updatePartialGraph(layer, tracker);

  // Optionally merge nearby nodes
  if (config.merge_nearby_nodes) {
    mergeNearbyNodes();
  }

  // Add heuristic edges to the compressed graph
  updateOverlapEdges();
  updateFreespaceEdges(layer);
}

void GraphExtractor::clearArchived() {
  const auto& compressed = gvd_.compressed();

  std::list<uint64_t> to_archive;
  for (const auto& [id, node] : compressed) {
    if (!node.archived()) {
      continue;
    }

    bool can_be_archived = true;
    for (const auto sibling_id : node.siblings) {
      const auto& sibling = compressed.at(sibling_id);
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
    MLOG(2) << "archiving " << id;
    gvd_.dropCompressed(id);
    archived_nodes_.insert(NodeSymbol(config.prefix, id));
  }
}

void GraphExtractor::updateGvdGraph(uint64_t timestamp_ns, const GvdLayer& layer) {
  ScopedTimer timer("places/update_gvd_graph", timestamp_ns);

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

    if (!voxel->num_extra_basis || voxel->distance < config.min_node_distance_m) {
      continue;
    }

    seen_indices.insert(index);
    gvd_.add(index, voxel->distance, voxel->num_extra_basis + 1);
  }
}

void GraphExtractor::updatePartialGraph(const GvdLayer& layer,
                                        const GvdParentTracker& tracker) {
  const auto& compressed = gvd_.compressed();
  auto iter = graph_.begin();
  while (iter != graph_.end()) {
    NodeSymbol graph_id(iter->first);
    if (compressed.count(graph_id.categoryId())) {
      ++iter;
      continue;
    } else {
      // TODO(nathan) track deleted
      iter = graph_.erase(iter);
    }
  }

  for (const auto& [node_id, node] : compressed) {
    auto result = gvd_.getCompressed(node_id, *merge_policy_);
    if (!result) {
      LOG(WARNING) << "Empty compressed node encountered: " << node_id;
      continue;
    }

    const auto graph_id = NodeSymbol(config.prefix, node_id);
    auto attrs = std::make_unique<PlaceNodeAttributes>();
    attrs->distance = result->distance;
    attrs->num_basis_points = result->num_basis_points;
    attrs->position = result->position.cast<double>();
    fillParentInfo(layer, tracker, result->index, *attrs);
    graph_.update(graph_id, std::move(attrs));
    node_index_map_.emplace(graph_id, result->index);
  }

  std::set<EdgeKey> curr_edges;
  for (const auto& [node_id, node] : compressed) {
    const NodeSymbol graph_id(config.prefix, node_id);
    for (const auto sibling : node.siblings) {
      const NodeSymbol sibling_id(config.prefix, sibling);
      const EdgeKey key(graph_id, sibling_id);
      if (curr_edges.count(key)) {
        continue;
      }

      curr_edges.insert(key);
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

      graph_.update(graph_id, sibling_id, std::move(attrs));
      // override heuristic edges with actual graph edges
      overlap_edges_.erase(key);
      freespace_edges_.erase(key);
    }
  }
}

void GraphExtractor::mergeNearbyNodes() {
  /*
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
  */
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
  findFreespaceEdges(graph_,
                     gvd,
                     node_index_map_,
                     config.freespace_edges.max_length_m,
                     config.freespace_edges.min_clearance_m,
                     proposed_edges);

  for (auto& [key, info] : proposed_edges) {
    graph_.update(key.k1, key.k2, std::move(info));
    freespace_edges_.insert(key);
  }
}

}  // namespace hydra::places
