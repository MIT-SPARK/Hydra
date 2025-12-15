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
#include "hydra/frontend/mesh_segmenter.h"

#include <config_utilities/config.h>
#include <config_utilities/types/conversions.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <glog/stl_logging.h>
#include <kimera_pgmo/mesh_delta.h>
#include <kimera_pgmo/mesh_traits.h>
#include <spark_dsg/bounding_box_extraction.h>
#include <spark_dsg/printing.h>

#include "hydra/utils/mesh_utilities.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using Cluster = MeshSegmenter::Cluster;
using LabelClusters = MeshSegmenter::LabelClusters;
using timing::ScopedTimer;

void declare_config(MeshSegmenter::Config& config) {
  using namespace config;
  name("MeshSegmenterConfig");
  field(config.layer_id, "layer_id");
  field(config.clustering, "clustering", false);
  enum_field(config.bounding_box_type,
             "bounding_box_type",
             {{spark_dsg::BoundingBox::Type::INVALID, "INVALID"},
              {spark_dsg::BoundingBox::Type::AABB, "AABB"},
              {spark_dsg::BoundingBox::Type::OBB, "OBB"},
              {spark_dsg::BoundingBox::Type::RAABB, "RAABB"}});
  field(config.timer_namespace, "timer_namespace");
  field(config.sinks, "sinks");
}

template <typename LList, typename RList>
void mergeList(LList& lhs, const RList& rhs) {
  std::unordered_set<size_t> seen(lhs.begin(), lhs.end());
  for (const auto idx : rhs) {
    if (seen.count(idx)) {
      continue;
    }

    lhs.push_back(idx);
    seen.insert(idx);
  }
}

inline bool nodesMatch(const SceneGraphNode& lhs_node, const SceneGraphNode& rhs_node) {
  return lhs_node.attributes<SemanticNodeAttributes>().bounding_box.contains(
      rhs_node.attributes().position);
}

inline bool nodesMatch(const Cluster& cluster, const SceneGraphNode& node) {
  return node.attributes<SemanticNodeAttributes>().bounding_box.contains(
      cluster.centroid);
}

// TODO(nathan) move node ID to not be here
MeshSegmenter::MeshSegmenter(const Config& config, const std::set<uint32_t>& labels)
    : config(config::checkValid(config)),
      next_node_id_('O', 0),
      labels_(labels),
      sinks_(Sink::instantiate(config.sinks)) {
  VLOG(2) << "[Mesh Segmenter] using labels: " << clustering::printLabels(labels_);
  for (const auto& label : labels_) {
    active_nodes_[label] = std::set<NodeId>();
  }
}

LabelClusters MeshSegmenter::detect(uint64_t stamp_ns,
                                    const kimera_pgmo::MeshDelta& delta,
                                    const kimera_pgmo::MeshOffsetInfo& offsets) {
  ScopedTimer timer(config.timer_namespace + "_detection", stamp_ns, true, 1, false);

  LabelClusters label_clusters;
  if (!delta.getNumActiveVertices()) {
    VLOG(2) << "[Mesh Segmenter] No active indices in mesh";
    return label_clusters;
  }

  const auto label_indices = clustering::getLabelIndices(labels_, delta);
  if (label_indices.empty()) {
    VLOG(2) << "[Mesh Segmenter] No vertices found matching desired labels";
    Sink::callAll(sinks_, stamp_ns, delta, label_indices);
    return label_clusters;
  }

  for (const auto& [label, indices] : label_indices) {
    if (indices.size() < config.clustering.min_cluster_size) {
      continue;
    }

    const auto result = clustering::findClusters(config.clustering, delta, indices);

    auto iter = label_clusters.insert({label, {}}).first;
    auto& clusters = iter->second;
    for (const auto& cluster_indices : result) {
      auto& cluster = clusters.emplace_back();
      for (const auto local_idx : cluster_indices) {
        cluster.indices.push_back(offsets.toGlobal(local_idx));
        cluster.centroid += delta.getVertex(local_idx).pos.cast<double>();
      }

      if (cluster_indices.size()) {
        cluster.centroid /= cluster_indices.size();
      }
    }

    VLOG(2) << "[Mesh Segmenter] Found " << clusters.size() << " cluster(s) of label "
            << label;
  }

  Sink::callAll(sinks_, stamp_ns, delta, label_indices);
  return label_clusters;
}

void MeshSegmenter::updateOldNodes(const kimera_pgmo::MeshDelta& delta,
                                   const kimera_pgmo::MeshOffsetInfo& offsets,
                                   DynamicSceneGraph& graph) {
  for (auto& [label, label_nodes] : active_nodes_) {
    auto iter = label_nodes.begin();
    while (iter != label_nodes.end()) {
      const auto node_id = *iter;
      auto& attrs = graph.getNode(node_id).attributes<ObjectNodeAttributes>();

      // remap and prune mesh connections
      VLOG(20) << "Updating node " << NodeSymbol(node_id).str() << " with connections "
               << attrs.mesh_connections;
      kimera_pgmo::MeshOffsetInfo::RemapInfo info;
      delta.updateIndices(attrs.mesh_connections, offsets, &info);
      VLOG(20) << "After update: " << attrs.mesh_connections << std::boolalpha
               << " (active: " << !info.all_archived << ")";
      if (attrs.mesh_connections.size() < config.clustering.min_cluster_size) {
        graph.removeNode(node_id);
        iter = label_nodes.erase(iter);
        continue;
      }

      attrs.is_active = !info.all_archived;
      if (!attrs.is_active) {
        iter = label_nodes.erase(iter);
      } else {
        ++iter;
      }
    }
  }

  for (const auto& [label, label_nodes] : active_nodes_) {
    VLOG(10) << "Active nodes for label " << label << ": "
             << displayNodeSymbolContainer(label_nodes);
  }
}

void MeshSegmenter::updateGraph(uint64_t timestamp_ns,
                                const kimera_pgmo::MeshDelta& active,
                                const kimera_pgmo::MeshOffsetInfo& offsets,
                                const LabelClusters& clusters,
                                DynamicSceneGraph& graph) {
  ScopedTimer timer(config.timer_namespace + "_graph_update", timestamp_ns);
  updateOldNodes(active, offsets, graph);
  if (!graph.hasMesh()) {
    LOG(ERROR) << "Unable to update graph without mesh!";
    return;
  }

  for (auto&& [label, clusters_for_label] : clusters) {
    for (const auto& cluster : clusters_for_label) {
      bool matches_prev_node = false;
      std::vector<NodeId> nodes_not_in_graph;
      for (const auto& prev_node_id : active_nodes_.at(label)) {
        const auto& prev_node = graph.getNode(prev_node_id);
        if (nodesMatch(cluster, prev_node)) {
          updateNodeInGraph(graph, cluster, prev_node, timestamp_ns);
          matches_prev_node = true;
          break;
        }
      }

      if (!matches_prev_node) {
        addNodeToGraph(graph, cluster, label, timestamp_ns);
      }

      mergeActiveNodes(graph, label);
    }
  }
}

void MeshSegmenter::mergeActiveNodes(DynamicSceneGraph& graph, uint32_t label) {
  std::set<NodeId> merged_nodes;

  auto& curr_active = active_nodes_.at(label);
  for (const auto& node_id : curr_active) {
    if (merged_nodes.count(node_id)) {
      continue;
    }
    const auto& node = graph.getNode(node_id);

    std::list<NodeId> to_merge;
    for (const auto& other_id : curr_active) {
      if (node_id == other_id) {
        continue;
      }

      if (merged_nodes.count(other_id)) {
        continue;
      }

      const auto& other = graph.getNode(other_id);
      if (nodesMatch(node, other) || nodesMatch(other, node)) {
        to_merge.push_back(other_id);
      }
    }

    auto& attrs = node.attributes<ObjectNodeAttributes>();
    for (const auto& other_id : to_merge) {
      const auto& other = graph.getNode(other_id);
      auto& other_attrs = other.attributes<ObjectNodeAttributes>();
      mergeList(attrs.mesh_connections, other_attrs.mesh_connections);
      graph.removeNode(other_id);
      merged_nodes.insert(other_id);
    }

    if (!to_merge.empty()) {
      updateObjectGeometry(*graph.mesh(), attrs);
    }
  }

  for (const auto& node_id : merged_nodes) {
    curr_active.erase(node_id);
  }
}

std::unordered_set<NodeId> MeshSegmenter::getActiveNodes() const {
  std::unordered_set<NodeId> active_nodes;
  for (const auto& label_nodes_pair : active_nodes_) {
    active_nodes.insert(label_nodes_pair.second.begin(), label_nodes_pair.second.end());
  }
  return active_nodes;
}

void MeshSegmenter::updateNodeInGraph(DynamicSceneGraph& graph,
                                      const Cluster& cluster,
                                      const SceneGraphNode& node,
                                      uint64_t timestamp) {
  auto& attrs = node.attributes<ObjectNodeAttributes>();
  attrs.last_update_time_ns = timestamp;
  attrs.is_active = true;

  mergeList(attrs.mesh_connections, cluster.indices);
  updateObjectGeometry(*graph.mesh(), attrs);
}

void MeshSegmenter::addNodeToGraph(DynamicSceneGraph& graph,
                                   const Cluster& cluster,
                                   uint32_t label,
                                   uint64_t timestamp) {
  if (cluster.indices.empty()) {
    LOG(ERROR) << "Encountered empty cluster with label" << static_cast<int>(label)
               << " @ " << timestamp << "[ns]";
    return;
  }

  auto attrs = std::make_unique<ObjectNodeAttributes>();
  attrs->last_update_time_ns = timestamp;
  attrs->is_active = true;
  attrs->semantic_label = label;
  attrs->mesh_connections.insert(
      attrs->mesh_connections.begin(), cluster.indices.begin(), cluster.indices.end());

  updateObjectGeometry(*graph.mesh(), *attrs, nullptr, config.bounding_box_type);

  graph.emplaceNode(config.layer_id, next_node_id_, std::move(attrs));
  active_nodes_.at(label).insert(next_node_id_);
  ++next_node_id_;
}

}  // namespace hydra
