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
#include <spark_dsg/node_symbol.h>
#include <spatial_hash/neighbor_utils.h>

#include "hydra/places/gvd_places/graph_extractor_utilities.h"
#include "hydra/places/gvd_places/gvd_parent_tracker.h"
#include "hydra/utils/printing.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra::places {

using spark_dsg::EdgeKey;
using spark_dsg::NodeId;
using spark_dsg::PlaceNodeAttributes;
using timing::ScopedTimer;

namespace {

std::optional<NodeId> getBestNode(const GraphExtractor::LocalGraph& graph,
                                  NodeId source,
                                  const std::vector<NodeId>& candidates,
                                  double max_distance_m) {
  const auto& source_attrs = graph.at(source);
  if (!source_attrs.is_active) {
    return std::nullopt;  // no best node for archived node
  }

  double best_dist = 0.0;
  std::optional<NodeId> best_node;
  const auto pos = source_attrs.position;
  for (const auto& target : candidates) {
    const auto target_attrs = graph.at(target);
    if (!target_attrs.is_active) {
      continue;  // archived node
    }

    const auto dist = (pos - target_attrs.position).norm();
    if (!best_node || dist < best_dist) {
      best_node = target;
      best_dist = dist;
    }
  }

  if (best_node && best_dist > max_distance_m) {
    best_node = std::nullopt;
  }

  return best_node;
}

void fillAttributes(const GvdMemberInfo& info, PlaceNodeAttributes& attrs) {
  attrs.distance = info.distance;
  attrs.num_basis_points = info.num_basis_points;
  attrs.position = info.position.cast<double>();

  attrs.voxblox_mesh_connections.clear();
  for (const auto& parent_pos : info.parents) {
    auto& connection = attrs.voxblox_mesh_connections.emplace_back();
    connection.voxel_pos[0] = parent_pos[0];
    connection.voxel_pos[1] = parent_pos[1];
    connection.voxel_pos[2] = parent_pos[2];
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

void GraphExtractor::archiveIndex(const GlobalIndex& index) { gvd_.archive(index); }

void GraphExtractor::extract(uint64_t timestamp_ns,
                             const GvdLayer& layer,
                             const VoxelIndexChanges& changes,
                             const GvdParentTracker& tracker) {
  ScopedTimer graph_timer("places/graph_extractor", timestamp_ns);

  // Update the gvd graph and compression with unique voxels
  updateGvdGraph(timestamp_ns, layer, tracker, changes);

  // Copy all the updates from the compressed GVD graph to the partial update
  updatePartialGraph(layer);

  // Optionally merge nearby nodes
  if (config.merge_nearby_nodes) {
    mergeNearbyNodes();
  }

  // Add heuristic edges to the compressed graph
  updateOverlapEdges();
  updateFreespaceEdges(layer);
}

void GraphExtractor::prune() {
  const auto archived = gvd_.clearArchived();
  MLOG(2) << "Cleared archived nodes [" << archived << "]";

  const auto archived_node_ids = graph_.prune();
  MLOG(2) << "Cleared graph nodes [" << archived_node_ids << "]";
  for (const auto& node_id : archived_node_ids) {
    node_index_map_.erase(node_id);
    node_attribute_map_.erase(node_id);
  }
}

void GraphExtractor::validate(uint64_t timestamp_ns,
                              const GvdLayer& layer,
                              const BlockIndices& /* archived_blocks */) const {
  std::vector<uint64_t> invalid_nodes;
  for (const auto& [node_id, node] : gvd_.uncompressed()) {
    if (node.archived) {
      continue;
    }

    const auto* voxel = layer.getVoxelPtr(node.info.index);
    if (!voxel) {
      const auto block_idx =
          spatial_hash::blockIndexFromGlobalIndex(node.info.index, 16);
      LOG(ERROR) << "Invalid node " << node_id << " is in block "
                 << showIndex(block_idx) << " (last updated at "
                 << node.info.last_updated << " vs. " << timestamp_ns << ")";
      invalid_nodes.push_back(node_id);
      continue;
    }

    if (!voxel->num_extra_basis) {
      LOG(ERROR) << "Invalid node " << node_id << " does not point to gvd voxel "
                 << *voxel;
      invalid_nodes.push_back(node_id);
    }
  }

  CHECK(invalid_nodes.empty()) << "Found active uncompressed nodes " << invalid_nodes
                               << " pointing to unallocated voxels";
}

void GraphExtractor::updateGvdGraph(uint64_t timestamp_ns,
                                    const GvdLayer& layer,
                                    const GvdParentTracker& tracker,
                                    const VoxelIndexChanges& changes) {
  ScopedTimer timer("places/update_gvd_graph", timestamp_ns);

  // process index updates from the gvd integration
  gvd_.remove(changes.removed);

  for (const auto& index : changes.added) {
    const auto voxel = layer.getVoxelPtr(index);
    CHECK(voxel) << "Invalid index " << showIndex(index) << " found!";
    if (!voxel->num_extra_basis || voxel->distance < config.min_node_distance_m) {
      continue;
    }

    const auto parents = tracker.parents(index);
    if (parents.empty()) {
      LOG(ERROR) << "Invalid GVD voxel with no parents: " << *voxel << " @ "
                 << showIndex(index);
      continue;
    }

    gvd_.add(index, voxel->distance, voxel->num_extra_basis + 1, parents, timestamp_ns);
  }
}

void GraphExtractor::updatePartialGraph(const GvdLayer& layer) {
  std::set<NodeId> stale_nodes;
  for (const auto& [node_id, node] : graph_.nodes()) {
    if (node.attributes().is_active) {
      stale_nodes.insert(node_id);
    }
  }

  std::set<EdgeKey> stale_edges;
  for (const auto& [key, _] : graph_.edges()) {
    if (!graph_.at(key.k1).is_active && !graph_.at(key.k2).is_active) {
      continue;
    }

    if (overlap_edges_.count(key) || freespace_edges_.count(key)) {
      continue;
    }

    stale_edges.insert(key);
  }

  const auto& compressed = gvd_.compressed();
  for (const auto& [node_id, node] : compressed) {
    stale_nodes.erase(node_id);
    const auto result = gvd_.getCompressed(node_id, *merge_policy_);
    CHECK(result);

    auto attrs = std::make_unique<PlaceNodeAttributes>();
    attrs->is_active = !node.archived();
    fillAttributes(*result, *attrs);
    graph_.add(node_id, std::move(attrs));
    node_index_map_[node_id] = result->index;
    node_attribute_map_[node_id] = result;
  }

  for (const auto& [node_id, node] : compressed) {
    const auto node_attrs = graph_.find(node_id);
    if (!node_attrs) {
      LOG(ERROR) << "Node " << node_id << " not in graph!";
      continue;
    }

    for (const auto sibling_id : node.siblings) {
      const auto sibling_attrs = graph_.find(sibling_id);
      if (!sibling_attrs) {
        LOG(ERROR) << "Sibling " << sibling_id << " not in graph!";
        continue;  // skip any unadded nodes
      }

      auto edge_attrs = getFreespaceEdgeInfo(layer,
                                             *node_attrs,
                                             node_index_map_.at(node_id),
                                             *sibling_attrs,
                                             node_index_map_.at(sibling_id),
                                             config.min_edge_distance_m,
                                             !sibling_attrs->is_active);
      if (!edge_attrs) {
        graph_.remove(node_id, sibling_id);
        continue;
      }

      graph_.add(node_id, sibling_id, std::move(edge_attrs));

      // override heuristic edges with actual graph edges
      const EdgeKey key(node_id, sibling_id);
      stale_edges.erase(key);
      overlap_edges_.erase(key);
      freespace_edges_.erase(key);
    }
  }

  for (const auto& node_id : stale_nodes) {
    graph_.remove(node_id);
    node_index_map_.erase(node_id);
    node_attribute_map_.erase(node_id);
  }

  for (const auto& key : stale_edges) {
    graph_.remove(key.k1, key.k2);
  }
}

void GraphExtractor::mergeNearbyNodes() {
  std::unordered_map<uint64_t, uint64_t> merges;
  std::unordered_map<uint64_t, std::unordered_set<uint64_t>> reversed_merges;
  for (const auto& [node_id, node] : gvd_.compressed()) {
    if (merges.count(node_id) || node.archived()) {
      continue;  // skip already merged or archived nodes
    }

    const auto node_info = node_attribute_map_.at(node_id);
    for (const auto sibling_id : node.siblings) {
      const auto& sibling = gvd_.compressed().at(sibling_id);
      if (sibling.archived()) {
        continue;
      }

      const auto sibling_info = node_attribute_map_.at(sibling_id);
      const auto dist = (node_info->position - sibling_info->position).norm();
      if (dist > config.node_merge_distance_m) {
        continue;
      }

      // assign merge to the node with the most basis points
      const auto lhs_is_better = merge_policy_->compare(*node_info, *sibling_info);
      const auto from_node = lhs_is_better >= 0 ? sibling_id : node_id;
      auto to_node = lhs_is_better >= 0 ? node_id : sibling_id;

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
    graph_.contract(from, to);
  }
}

void GraphExtractor::updateOverlapEdges() {
  if (!config.overlap_edges.enable) {
    return;
  }

  for (auto iter = overlap_edges_.begin(); iter != overlap_edges_.end();) {
    const auto [source, target] = *iter;
    if (!graph_.has(source) || !graph_.has(target)) {
      iter = overlap_edges_.erase(iter);  // drop edges from removed nodes
    } else if (!graph_.at(source).is_active && !graph_.at(target).is_active) {
      iter = overlap_edges_.erase(iter);  // edge between archived nodes can be fixed
    } else {
      ++iter;
    }
  }

  const auto thresh = config.overlap_edges.min_clearance_m;
  for (auto s_iter = graph_.nodes().begin(); s_iter != graph_.nodes().end(); ++s_iter) {
    for (auto t_iter = std::next(s_iter); t_iter != graph_.nodes().end(); ++t_iter) {
      const auto& [source_id, source] = *s_iter;
      const auto& [target_id, target] = *t_iter;
      if (!source.attributes().is_active && !target.attributes().is_active) {
        continue;
      }

      EdgeKey key{source_id, target_id};
      if (graph_.has(key.k1, key.k2) && !overlap_edges_.count(key)) {
        continue;
      }

      auto info = getOverlapEdgeInfo(source.attributes(), target.attributes(), thresh);
      if (info) {
        graph_.add(key.k1, key.k2, std::move(info));
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
    if (!graph_.has(source) || !graph_.has(target)) {
      iter = freespace_edges_.erase(iter);  // drop edges from removed nodes
      continue;
    }

    if (!graph_.at(source).is_active && !graph_.at(target).is_active) {
      iter = freespace_edges_.erase(iter);  // edge between archived nodes can be fixed
      continue;
    }

    auto info = getFreespaceEdgeInfo(gvd,
                                     graph_.at(source),
                                     node_index_map_.at(source),
                                     graph_.at(target),
                                     node_index_map_.at(target),
                                     config.freespace_edges.min_clearance_m,
                                     true);
    if (!info) {
      graph_.remove(source, target);
      iter = freespace_edges_.erase(iter);
    } else {
      graph_.add(source, target, std::move(info));
      ++iter;
    }
  }

  auto components = graph_.connected_components();
  if (components.size() <= 1) {
    return;  // nothing to do
  }

  auto first_component = components.front();
  for (size_t i = 1; i < components.size(); ++i) {
    const auto& component = components[i];

    bool inserted_edge = false;
    for (const auto& source : component) {
      const auto target = getBestNode(
          graph_, source, first_component, config.freespace_edges.max_length_m);
      if (!target || graph_.has(source, *target)) {
        continue;  // graph should never has edges connecting components, but...
      }

      const EdgeKey key(source, *target);
      auto info = getFreespaceEdgeInfo(gvd,
                                       graph_.at(source),
                                       node_index_map_.at(source),
                                       graph_.at(*target),
                                       node_index_map_.at(*target),
                                       config.freespace_edges.min_clearance_m,
                                       false);
      if (info) {
        inserted_edge = true;
        graph_.add(key.k1, key.k2, std::move(info));
        freespace_edges_.insert(key);
      }
    }

    if (inserted_edge) {
      // merge components if an edge was inserted
      first_component.insert(first_component.end(), component.begin(), component.end());
    }
  }
}

}  // namespace hydra::places
