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
#include "hydra/frontend/place_2d_segmenter.h"

#include <config_utilities/config.h>
#include <config_utilities/types/conversions.h>
#include <config_utilities/types/enum.h>
#include <glog/logging.h>
#include <kimera_pgmo/mesh_delta.h>

#include <memory>
#define PCL_NO_PRECOMPILE
#include <pcl/segmentation/extract_clusters.h>
#undef PCL_NO_PRECOMPILE
#include <spark_dsg/bounding_box_extraction.h>

#include "hydra/common/global_info.h"
#include "hydra/common/semantic_color_map.h"
#include "hydra/frontend/place_2d_split_logic.h"
#include "hydra/utils/place_2d_ellipsoid_math.h"

namespace hydra {

using Places = Place2dSegmenter::Places;
using LabelPlaces = Place2dSegmenter::LabelPlaces;
using LabelIndices = Place2dSegmenter::LabelIndices;
using IndicesVector = Place2dSegmenter::IndicesVector;
using OptPosition = std::optional<Eigen::Vector3d>;
using KdTreeT = pcl::search::KdTree<pcl::PointXYZRGBA>;

void mergeList(std::vector<size_t>& lhs, const std::vector<int>& rhs) {
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

Place2dSegmenter::Place2dSegmenter(const Config& config)
    : config(config), next_node_id_(config.prefix, 0), num_archived_vertices_(0) {
  VLOG(1) << "[Hydra Frontend] Detecting 2d places: " << printLabels(config.labels);
  for (const auto& label : config.labels) {
    active_places_[label] = std::set<NodeId>();
  }
}

Places Place2dSegmenter::findPlaces(const Mesh::Positions& points,
                                    const kimera_pgmo::MeshDelta& delta,
                                    const pcl::IndicesPtr& cloud_indices,
                                    double connection_ellipse_scale_factor) const {
  pcl::IndicesPtr inds(new pcl::Indices());
  inds->resize(cloud_indices->size());
  for (size_t ix = 0; ix < cloud_indices->size(); ++ix) {
    inds->at(ix) = delta.getLocalIndex(cloud_indices->at(ix));
  }

  KdTreeT::Ptr tree(new KdTreeT());
  tree->setInputCloud(delta.vertex_updates, inds);

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> estimator;
  estimator.setClusterTolerance(config.cluster_tolerance);
  estimator.setMinClusterSize(config.min_cluster_size);
  estimator.setMaxClusterSize(config.max_cluster_size);
  estimator.setSearchMethod(tree);
  estimator.setInputCloud(delta.vertex_updates);
  estimator.setIndices(inds);

  std::vector<pcl::PointIndices> cluster_indices;
  estimator.extract(cluster_indices);

  Places places;
  places.resize(cluster_indices.size());
  for (size_t k = 0; k < places.size(); ++k) {
    for (const auto& ind : cluster_indices.at(k).indices) {
      places.at(k).indices.push_back(static_cast<size_t>(delta.getGlobalIndex(ind)));
    }

    addRectInfo(points, connection_ellipse_scale_factor, places.at(k));
  }

  return places;
}

pcl::IndicesPtr getActivePlaceIndices(
    const pcl::IndicesPtr& indices,
    const std::map<uint32_t, std::set<NodeId>>& active_places,
    const kimera_pgmo::MeshDelta& delta,
    const DynamicSceneGraph& graph,
    size_t& num_archived_vertices,
    std::list<NodeId>& empty_nodes) {
  pcl::IndicesPtr active_indices;
  active_indices.reset(new IndicesVector());
  active_indices->reserve(indices->size());

  num_archived_vertices = delta.getTotalArchivedVertices();

  VLOG(5) << "[Places 2d Segmenter] n original active indices: " << indices->size();
  std::unordered_set<size_t> frozen_indices;
  for (auto kv : active_places) {
    std::set<NodeId> nodes = kv.second;
    for (auto nid : nodes) {
      auto& attrs = graph.getNode(nid).attributes<Place2dNodeAttributes>();
      size_t min_index = std::numeric_limits<size_t>::max();
      size_t max_index = 0;
      auto iter = attrs.pcl_mesh_connections.begin();
      while (iter != attrs.pcl_mesh_connections.end()) {
        if (delta.deleted_indices.count(*iter)) {
          iter = attrs.pcl_mesh_connections.erase(iter);
          continue;
        }

        auto map_iter = delta.prev_to_curr.find(*iter);
        if (map_iter != delta.prev_to_curr.end()) {
          *iter = map_iter->second;
        }

        min_index = std::min(min_index, *iter);
        max_index = std::max(max_index, *iter);
        ++iter;
      }

      attrs.pcl_min_index = min_index;
      attrs.pcl_max_index = max_index;

      const auto prev_boundary = attrs.boundary;
      const auto prev_boundary_connections = attrs.pcl_boundary_connections;
      attrs.boundary.clear();
      attrs.pcl_boundary_connections.clear();
      for (size_t i = 0; i < prev_boundary.size(); ++i) {
        if (delta.deleted_indices.count(prev_boundary_connections.at(i))) {
          continue;
        }

        auto map_iter = delta.prev_to_curr.find(prev_boundary_connections.at(i));
        if (map_iter != delta.prev_to_curr.end()) {
          attrs.boundary.push_back(prev_boundary.at(i));
          attrs.pcl_boundary_connections.push_back(map_iter->second);
        } else {
          attrs.boundary.push_back(prev_boundary.at(i));
          attrs.pcl_boundary_connections.push_back(prev_boundary_connections.at(i));
        }
      }

      if (attrs.pcl_min_index < num_archived_vertices) {
        // ^ this means that the place contains an archived vertex
        for (auto mi : attrs.pcl_mesh_connections) {
          frozen_indices.insert(mi);
        }
      }

      if (!attrs.is_active) {
        if (attrs.pcl_mesh_connections.size() == 0 || attrs.boundary.size() < 3) {
          empty_nodes.push_back(nid);
        }
      }
    }
  }

  VLOG(5) << "[Places 2d Segmenter] n frozen indices: " << frozen_indices.size();

  for (const size_t idx : *indices) {
    if (!frozen_indices.count(idx)) {
      active_indices->push_back(idx);
    }
  }

  VLOG(5) << "[Places 2d Segmenter] n final active indices: " << active_indices->size();
  return active_indices;
}

NodeIdSet Place2dSegmenter::getActiveNodes() const {
  std::unordered_set<NodeId> all_active_nodes;
  for (auto kv : active_places_) {
    all_active_nodes.insert(kv.second.begin(), kv.second.end());
  }
  return all_active_nodes;
}

void Place2dSegmenter::detect(const ReconstructionOutput&,
                              const kimera_pgmo::MeshDelta& mesh_delta,
                              const DynamicSceneGraph& graph) {
  VLOG(5) << "[Places 2d Segmenter] detect called";
  const auto active_indices = getActivePlaceIndices(mesh_delta.getActiveIndices(),
                                                    active_places_,
                                                    mesh_delta,
                                                    graph,
                                                    num_archived_vertices_,
                                                    nodes_to_remove_);

  LabelPlaces label_places;

  if (active_indices->empty()) {
    VLOG(5) << "[Places 2d Segmenter] No active indices in mesh";
    detected_label_places_ = label_places;
    return;
  }

  const auto& mesh = *CHECK_NOTNULL(graph.mesh());
  LabelIndices label_indices = getLabelIndices(mesh.labels, *active_indices);
  if (label_indices.empty()) {
    VLOG(5) << "[Places 2d Segmenter] No vertices found matching desired labels";
    detected_label_places_ = label_places;
    return;
  }

  for (const auto label : config.labels) {
    if (!label_indices.count(label)) {
      continue;
    }

    if (label_indices.at(label)->size() < config.min_cluster_size) {
      continue;
    }

    const auto initial_places = findPlaces(mesh.points,
                                           mesh_delta,
                                           label_indices.at(label),
                                           config.connection_ellipse_scale_factor);

    VLOG(5) << "[Places 2d Segmenter] got " << initial_places.size()
            << " initial places";
    std::vector<Place2d> final_places =
        decomposePlaces(mesh.points,
                        initial_places,
                        config.pure_final_place_size,
                        config.min_final_place_points,
                        config.connection_ellipse_scale_factor);

    VLOG(5) << "[Places 2d Segmenter]  - Found " << final_places.size()
            << " final places of label " << static_cast<int>(label);
    label_places.insert({label, final_places});
  }

  detected_label_places_ = label_places;
}

LabelIndices Place2dSegmenter::getLabelIndices(const Mesh::Labels& labels,
                                               const IndicesVector& indices) const {
  LabelIndices label_indices;

  std::set<uint32_t> seen_labels;
  for (const auto idx : indices) {
    if (static_cast<size_t>(idx) >= labels.size()) {
      LOG(ERROR) << "bad index " << idx << "(of " << labels.size() << ")";
      continue;
    }

    const auto label = labels.at(idx);
    seen_labels.insert(label);

    if (!config.labels.count(label)) {
      continue;
    }

    if (!label_indices.count(label)) {
      label_indices[label] = pcl::IndicesPtr(new IndicesVector());
    }

    label_indices[label]->push_back(idx);
  }

  VLOG(5) << "[Places 2d Segmenter] Seen labels: " << printLabels(seen_labels);
  return label_indices;
}

bool Place2dSegmenter::frontendAddPlaceConnection(const Place2dNodeAttributes& attrs1,
                                                  const Place2dNodeAttributes& attrs2,
                                                  EdgeAttributes& edge_attrs) {
  return shouldAddPlaceConnection(attrs1,
                                  attrs2,
                                  config.place_overlap_threshold,
                                  config.place_max_neighbor_z_diff,
                                  edge_attrs);
}

void Place2dSegmenter::updateGraph(uint64_t timestamp_ns,
                                   const ReconstructionOutput& msg,
                                   DynamicSceneGraph& graph) {
  // Remove old empty nodes
  for (const auto& nid : nodes_to_remove_) {
    graph.removeNode(nid);
  }
  nodes_to_remove_.clear();

  std::optional<Eigen::Vector3d> pos = msg.world_t_body;
  VLOG(5) << "[Places 2d Segmenter] updateGraph";
  std::map<uint32_t, std::set<NodeId>> active_places_to_check;
  for (const auto& label : config.labels) {
    active_places_to_check[label] = std::set<NodeId>();
  }
  std::map<uint32_t, std::set<NodeId>> new_active_places;
  for (const auto& label : config.labels) {
    new_active_places[label] = std::set<NodeId>();
  }

  if (!pos) {
    new_active_places = active_places_;
  } else {
    for (auto kv : active_places_) {
      std::set<NodeId> nodes = kv.second;
      for (NodeId nid : nodes) {
        auto& attrs = graph.getNode(nid).attributes<Place2dNodeAttributes>();
        if (attrs.pcl_mesh_connections.size() == 0 || attrs.boundary.size() < 3) {
          // Remove dangling places
          graph.removeNode(nid);
          continue;
        }
        if (attrs.pcl_min_index >= num_archived_vertices_) {
          graph.removeNode(nid);
        } else {
          active_places_to_check.at(kv.first).insert(nid);
        }
      }
    }
  }

  for (const auto& label_places : detected_label_places_) {
    for (const auto& place : label_places.second) {
      NodeSymbol ns = addPlaceToGraph(graph, place, label_places.first, timestamp_ns);
      active_places_to_check.at(label_places.first).insert(ns);
    }
  }

  std::set<std::pair<uint32_t, NodeId>> full_nodes;
  for (auto kv : active_places_to_check) {
    std::set<NodeId> nodes = kv.second;
    for (NodeId nid : nodes) {
      full_nodes.insert(std::pair<uint32_t, NodeId>(kv.first, nid));
    }
  }
  for (auto kv : semiactive_places_) {
    std::set<NodeId> nodes = kv.second;
    for (NodeId nid : nodes) {
      full_nodes.insert(std::pair<uint32_t, NodeId>(kv.first, nid));
    }
  }

  std::map<uint32_t, std::set<NodeId>> new_semiactive_places;
  for (const auto& label : config.labels) {
    new_semiactive_places[label] = std::set<NodeId>();
  }
  for (auto label_ns : full_nodes) {
    uint32_t label = label_ns.first;
    NodeSymbol ns1 = label_ns.second;
    auto& attrs1 = graph.getNode(ns1).attributes<Place2dNodeAttributes>();

    bool neighbors_are_fixed = true;
    for (auto label_ns2 : full_nodes) {
      NodeSymbol ns2 = label_ns2.second;
      if (ns1 == ns2) {
        continue;
      }
      const auto& attrs2 = graph.getNode(ns2).attributes<Place2dNodeAttributes>();
      EdgeAttributes ea;
      if (frontendAddPlaceConnection(attrs1, attrs2, ea)) {
        graph.insertEdge(ns1, ns2, ea.clone());
        if (attrs2.pcl_min_index >= num_archived_vertices_) {
          neighbors_are_fixed = false;
        }
      }
    }
    if (attrs1.pcl_max_index < num_archived_vertices_) {
      attrs1.has_active_mesh_indices = false;
    }
    if (!attrs1.has_active_mesh_indices && neighbors_are_fixed) {
      attrs1.is_active = false;
    } else if (!attrs1.has_active_mesh_indices) {
      new_semiactive_places.at(label).insert(ns1);
    } else {
      new_active_places.at(label).insert(ns1);
    }
  }

  active_places_ = new_active_places;
  semiactive_places_ = new_semiactive_places;
}

NodeSymbol Place2dSegmenter::addPlaceToGraph(DynamicSceneGraph& graph,
                                             const Place2d& place,
                                             uint32_t label,
                                             uint64_t timestamp) {
  if (place.indices.size() == 0) {
    LOG(ERROR) << "Encountered empty place with label" << static_cast<int>(label)
               << " @ " << timestamp << "[ns]";
    return next_node_id_;
  }

  Place2dNodeAttributes::Ptr attrs = std::make_unique<Place2dNodeAttributes>();

  pcl::PointXYZ centroid;
  place.centroid.get(centroid);
  attrs->position << centroid.x, centroid.y, centroid.z;
  attrs->is_active = true;

  attrs->semantic_label = label;
  attrs->name = NodeSymbol(next_node_id_).getLabel();
  attrs->boundary = place.boundary;
  attrs->pcl_boundary_connections.insert(attrs->pcl_boundary_connections.begin(),
                                         place.boundary_indices.begin(),
                                         place.boundary_indices.end());
  attrs->ellipse_matrix_compress = place.ellipse_matrix_compress;
  attrs->ellipse_matrix_expand = place.ellipse_matrix_expand;
  attrs->ellipse_centroid(0) = place.ellipse_centroid(0);
  attrs->ellipse_centroid(1) = place.ellipse_centroid(1);
  attrs->ellipse_centroid(2) = centroid.z;
  attrs->pcl_min_index = place.min_mesh_index;
  attrs->pcl_max_index = place.max_mesh_index;
  attrs->has_active_mesh_indices = true;

  attrs->pcl_mesh_connections.insert(
      attrs->pcl_mesh_connections.begin(), place.indices.begin(), place.indices.end());

  attrs->need_finish_merge = false;

  auto label_map = GlobalInfo::instance().getSemanticColorMap();
  if (!label_map || !label_map->isValid()) {
    label_map = GlobalInfo::instance().setRandomColormap();
    CHECK(label_map != nullptr);
  }

  attrs->color  = label_map->getColorFromLabel(label);

  graph.emplaceNode(DsgLayers::MESH_PLACES, next_node_id_, std::move(attrs));

  active_places_.at(label).insert(next_node_id_);
  active_place_timestamps_[next_node_id_] = timestamp;

  return next_node_id_++;
}

void declare_config(Place2dSegmenter::Config& config) {
  using namespace config;
  name("Place2dSegmenterConfig");
  field<CharConversion>(config.prefix, "prefix");
  field(config.cluster_tolerance, "cluster_tolerance");
  field(config.min_cluster_size, "min_cluster_size");
  field(config.max_cluster_size, "max_cluster_size");
  field(config.pure_final_place_size, "pure_final_place_size");
  field(config.min_final_place_points, "min_final_place_points");
  field(config.place_overlap_threshold, "place_overlap_threshold");
  field(config.place_max_neighbor_z_diff, "place_max_neighbor_z_diff");
  field(config.connection_ellipse_scale_factor, "connection_ellipse_scale_factor");
  config.labels = GlobalInfo::instance().getLabelSpaceConfig().surface_places_labels;
}

}  // namespace hydra
