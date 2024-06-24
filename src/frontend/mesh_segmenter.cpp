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

#include <glog/logging.h>
#include <kimera_pgmo/mesh_delta.h>
#define PCL_NO_PRECOMPILE
#include <pcl/segmentation/extract_clusters.h>
#undef PCL_NO_PRECOMPILE
#include <config_utilities/config.h>
#include <config_utilities/types/conversions.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/validation.h>
#include <spark_dsg/bounding_box_extraction.h>

#include "hydra/common/global_info.h"
#include "hydra/common/semantic_color_map.h"
#include "hydra/utils/mesh_utilities.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using Clusters = MeshSegmenter::Clusters;
using LabelClusters = MeshSegmenter::LabelClusters;
using KdTreeT = pcl::search::KdTree<pcl::PointXYZRGBA>;
using timing::ScopedTimer;

void declare_config(MeshSegmenter::Config& config) {
  using namespace config;
  name("MeshSegmenterConfig");
  field<CharConversion>(config.prefix, "prefix");
  // TODO(nathan) string to number conversion
  field(config.layer_id, "layer_id");
  field(config.active_index_horizon_m, "active_index_horizon_m");
  field(config.cluster_tolerance, "cluster_tolerance");
  field(config.min_cluster_size, "min_cluster_size");
  field(config.max_cluster_size, "max_cluster_size");
  enum_field(config.bounding_box_type,
             "bounding_box_type",
             {{spark_dsg::BoundingBox::Type::INVALID, "INVALID"},
              {spark_dsg::BoundingBox::Type::AABB, "AABB"},
              {spark_dsg::BoundingBox::Type::OBB, "OBB"},
              {spark_dsg::BoundingBox::Type::RAABB, "RAABB"}});
  config.labels = GlobalInfo::instance().getLabelSpaceConfig().object_labels;
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

template <typename T>
std::string printLabels(const std::set<T>& labels) {
  std::stringstream ss;
  ss << "[";
  auto iter = labels.begin();
  while (iter != labels.end()) {
    ss << static_cast<uint64_t>(*iter);
    ++iter;
    if (iter != labels.end()) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}

inline bool nodesMatch(const SceneGraphNode& lhs_node, const SceneGraphNode& rhs_node) {
  return lhs_node.attributes<SemanticNodeAttributes>().bounding_box.contains(
      rhs_node.attributes().position);
}

inline bool nodesMatch(const Cluster& cluster, const SceneGraphNode& node) {
  return node.attributes<SemanticNodeAttributes>().bounding_box.contains(
      cluster.centroid);
}

std::vector<size_t> getActiveIndices(const kimera_pgmo::MeshDelta& delta,
                                     const std::optional<Eigen::Vector3d>& pos,
                                     double horizon_m) {
  const auto indices = delta.getActiveIndices();

  std::vector<size_t> active;
  active.reserve(indices->size());
  if (!pos) {
    for (const auto& idx : *indices) {
      active.push_back(idx - delta.vertex_start);
    }

    return active;
  }

  const Eigen::Vector3d root_pos = *pos;
  for (const size_t idx : *indices) {
    const auto delta_idx = delta.getLocalIndex(idx);
    const auto& p = delta.vertex_updates->at(delta_idx);
    const Eigen::Vector3d vertex_pos(p.x, p.y, p.z);
    if ((vertex_pos - root_pos).norm() < horizon_m) {
      active.push_back(delta_idx);
    }
  }

  VLOG(2) << "[Mesh Segmenter] Active indices: " << indices->size()
          << " (used: " << active.size() << ")";
  return active;
}

LabelIndices getLabelIndices(const MeshSegmenter::Config& config,
                             const kimera_pgmo::MeshDelta& delta,
                             const std::vector<size_t>& indices) {
  CHECK(delta.hasSemantics());
  const auto& labels = delta.semantic_updates;

  LabelIndices label_indices;
  std::set<uint32_t> seen_labels;
  for (const auto idx : indices) {
    if (static_cast<size_t>(idx) >= labels.size()) {
      LOG(ERROR) << "bad index " << idx << "(of " << labels.size() << ")";
      continue;
    }

    const auto label = labels[idx];
    seen_labels.insert(label);
    if (!config.labels.count(label)) {
      continue;
    }

    auto iter = label_indices.find(label);
    if (iter == label_indices.end()) {
      iter = label_indices.emplace(label, std::vector<size_t>()).first;
    }

    iter->second.push_back(idx);
  }

  VLOG(2) << "[Mesh Segmenter] Seen labels: " << printLabels(seen_labels);
  return label_indices;
}

Clusters findClusters(const MeshSegmenter::Config& config,
                      const kimera_pgmo::MeshDelta& delta,
                      const std::vector<size_t>& indices) {
  pcl::IndicesPtr pcl_indices(new pcl::Indices(indices.begin(), indices.end()));

  KdTreeT::Ptr tree(new KdTreeT());
  tree->setInputCloud(delta.vertex_updates, pcl_indices);

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> estimator;
  estimator.setClusterTolerance(config.cluster_tolerance);
  estimator.setMinClusterSize(config.min_cluster_size);
  estimator.setMaxClusterSize(config.max_cluster_size);
  estimator.setSearchMethod(tree);
  estimator.setInputCloud(delta.vertex_updates);
  estimator.setIndices(pcl_indices);

  std::vector<pcl::PointIndices> cluster_indices;
  estimator.extract(cluster_indices);

  Clusters clusters;
  clusters.resize(cluster_indices.size());
  for (size_t k = 0; k < clusters.size(); ++k) {
    auto& cluster = clusters.at(k);
    const auto& curr_indices = cluster_indices.at(k).indices;
    for (const auto local_idx : curr_indices) {
      cluster.indices.push_back(delta.getGlobalIndex(local_idx));

      const auto& p = delta.vertex_updates->at(local_idx);
      const Eigen::Vector3d pos(p.x, p.y, p.z);
      cluster.centroid += pos;
    }

    if (curr_indices.size()) {
      cluster.centroid /= curr_indices.size();
    }
  }

  return clusters;
}

MeshSegmenter::MeshSegmenter(const Config& config)
    : config(config::checkValid(config)),
      next_node_id_(config.prefix, 0),
      sinks_(Sink::instantiate(config.sinks)) {
  VLOG(2) << "[Mesh Segmenter] using labels: " << printLabels(config.labels);
  for (const auto& label : config.labels) {
    active_nodes_[label] = std::set<NodeId>();
  }
}

LabelClusters MeshSegmenter::detect(uint64_t timestamp_ns,
                                    const kimera_pgmo::MeshDelta& delta,
                                    const std::optional<Eigen::Vector3d>& pos) {
  const auto timer_name = config.timer_namespace + "_detection";
  ScopedTimer timer(timer_name, timestamp_ns, true, 1, false);

  const auto indices = getActiveIndices(delta, pos, config.active_index_horizon_m);

  LabelClusters label_clusters;
  if (indices.empty()) {
    VLOG(2) << "[Mesh Segmenter] No active indices in mesh";
    return label_clusters;
  }

  const auto label_indices = getLabelIndices(config, delta, indices);
  if (label_indices.empty()) {
    VLOG(2) << "[Mesh Segmenter] No vertices found matching desired labels";
    Sink::callAll(sinks_, timestamp_ns, delta, indices, label_indices);
    return label_clusters;
  }

  for (const auto label : config.labels) {
    if (!label_indices.count(label)) {
      continue;
    }

    if (label_indices.at(label).size() < config.min_cluster_size) {
      continue;
    }

    const auto clusters = findClusters(config, delta, label_indices.at(label));

    VLOG(2) << "[Mesh Segmenter]  - Found " << clusters.size()
            << " cluster(s) of label " << static_cast<int>(label);
    label_clusters.insert({label, clusters});
  }

  Sink::callAll(sinks_, timestamp_ns, delta, indices, label_indices);
  return label_clusters;
}

void MeshSegmenter::archiveOldNodes(const DynamicSceneGraph& graph,
                                    size_t num_archived_vertices) {
  std::set<NodeId> archived;
  for (const auto& label : config.labels) {
    std::list<NodeId> removed_nodes;
    for (const auto& node_id : active_nodes_.at(label)) {
      if (!graph.hasNode(node_id)) {
        removed_nodes.push_back(node_id);
        continue;
      }

      auto& attrs = graph.getNode(node_id).attributes<ObjectNodeAttributes>();
      bool is_active = false;
      for (const auto index : attrs.mesh_connections) {
        if (index >= num_archived_vertices) {
          is_active = true;
          break;
        }
      }

      attrs.is_active = is_active;
      if (!attrs.is_active) {
        removed_nodes.push_back(node_id);
      }
    }

    for (const auto& node_id : removed_nodes) {
      active_nodes_[label].erase(node_id);
    }
  }
}

void MeshSegmenter::updateGraph(uint64_t timestamp_ns,
                                const LabelClusters& clusters,
                                size_t num_archived_vertices,
                                DynamicSceneGraph& graph) {
  ScopedTimer timer(config.timer_namespace + "_graph_update", timestamp_ns);
  archiveOldNodes(graph, num_archived_vertices);

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
  attrs->name = NodeSymbol(next_node_id_).getLabel();
  const auto& label_to_name = GlobalInfo::instance().getLabelToNameMap();
  auto iter = label_to_name.find(label);
  if (iter != label_to_name.end()) {
    attrs->name = iter->second;
  } else {
    VLOG(2) << "Missing semantic label from map: " << std::to_string(label);
  }

  attrs->mesh_connections.insert(
      attrs->mesh_connections.begin(), cluster.indices.begin(), cluster.indices.end());

  auto label_map = GlobalInfo::instance().getSemanticColorMap();
  if (!label_map || !label_map->isValid()) {
    label_map = GlobalInfo::instance().setRandomColormap();
    CHECK(label_map != nullptr);
  }

  attrs->color = label_map->getColorFromLabel(label);

  updateObjectGeometry(*graph.mesh(), *attrs, nullptr, config.bounding_box_type);

  graph.emplaceNode(config.layer_id, next_node_id_, std::move(attrs));
  active_nodes_.at(label).insert(next_node_id_);
  ++next_node_id_;
}

}  // namespace hydra
