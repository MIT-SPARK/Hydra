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
#include <config_utilities/factory.h>
#include <config_utilities/types/conversions.h>
#include <config_utilities/types/enum.h>
#include <glog/logging.h>
#include <kimera_pgmo/mesh_delta.h>
#include <spark_dsg/bounding_box_extraction.h>

#include <memory>

#include "hydra/common/global_info.h"
#include "hydra/frontend/place_2d_split_logic.h"
#include "hydra/utils/place_2d_ellipsoid_math.h"

namespace hydra {
namespace {

static const auto registration_ =
    config::RegistrationWithConfig<SurfacePlacesInterface,
                                   Place2dSegmenter,
                                   Place2dSegmenter::Config>("place_2d");

bool placeIsEmpty(const Place2dNodeAttributes& attrs) {
  return attrs.pcl_mesh_connections.size() == 0 || attrs.boundary.size() < 3;
}

}  // namespace

using Places = Place2dSegmenter::Places;
using LabelPlaces = Place2dSegmenter::LabelPlaces;
using LabelToNodes = Place2dSegmenter::LabelToNodes;

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

std::unordered_set<size_t> getFrozenSet(const LabelToNodes& active_places,
                                        const kimera_pgmo::MeshOffsetInfo& offsets,
                                        const DynamicSceneGraph& graph,
                                        std::list<NodeId>& empty_nodes) {
  std::unordered_set<size_t> frozen_indices;
  for (const auto& [label, nodes] : active_places) {
    for (const auto nid : nodes) {
      const auto node = graph.findNode(nid);
      if (!node) {
        LOG(ERROR) << "[2D Places] Found removed node: " << NodeSymbol(nid);
        continue;
      }

      auto& attrs = node->attributes<Place2dNodeAttributes>();
      remapPlace2dMesh(attrs, offsets);
      if (!attrs.is_active && placeIsEmpty(attrs)) {
        empty_nodes.push_back(nid);
        continue;
      }

      if (attrs.pcl_min_index >= offsets.archived_vertices) {
        continue;
      }

      for (auto mi : attrs.pcl_mesh_connections) {
        if (mi >= offsets.archived_vertices) {
          frozen_indices.insert(offsets.toLocalVertex(mi));
        }
      }
    }
  }

  VLOG(5) << "[2D Places] n frozen indices: " << frozen_indices.size();
  return frozen_indices;
}

Place2dSegmenter::Place2dSegmenter(const Config& config)
    : config(config),
      next_node_id_(config.prefix, 0),
      sinks_(Sink::instantiate(config.sinks)) {
  VLOG(1) << "[2D Places] Using labels: " << clustering::printLabels(config.labels);
  for (const auto& label : config.labels) {
    active_places_[label] = std::set<NodeId>();
  }
}

NodeIdSet Place2dSegmenter::getActiveNodes() const {
  std::unordered_set<NodeId> all_active_nodes;
  for (auto kv : active_places_) {
    all_active_nodes.insert(kv.second.begin(), kv.second.end());
  }
  return all_active_nodes;
}

void Place2dSegmenter::detect(const ActiveWindowOutput& msg,
                              const kimera_pgmo::MeshDelta& delta,
                              const kimera_pgmo::MeshOffsetInfo& offsets,
                              const DynamicSceneGraph& graph) {
  VLOG(5) << "[2D Places] detect called";
  VLOG(5) << "[2D Places] n original active indices: " << delta.getNumActiveVertices();
  label_places_.clear();
  const auto frozen = getFrozenSet(active_places_, offsets, graph, to_remove_);
  const auto label_indices = clustering::getLabelIndices(config.labels, delta, &frozen);
  if (label_indices.empty()) {
    VLOG(5) << "[2D Places] No vertices found matching desired labels";
    return;
  }

  // TODO(nathan) we don't actually need the mesh here...
  const auto& mesh = *CHECK_NOTNULL(graph.mesh());
  for (const auto& [label, indices] : label_indices) {
    if (indices.size() < config.clustering.min_cluster_size) {
      continue;
    }

    auto clusters = clustering::findClusters(config.clustering, delta, indices);

    Places initial_places;
    for (auto& cluster_indices : clusters) {
      auto& place = initial_places.emplace_back();
      for (const auto& idx : cluster_indices) {
        place.indices.push_back(offsets.toGlobalVertex(idx));
      }

      addRectInfo(mesh.points, config.connection_ellipse_scale_factor, place);
    }

    VLOG(5) << "[2D Places] got " << initial_places.size() << " initial places";
    auto places = decomposePlaces(mesh.points,
                                  initial_places,
                                  config.pure_final_place_size,
                                  config.min_final_place_points,
                                  config.connection_ellipse_scale_factor);
    VLOG(5) << "[2D Places] " << places.size() << " final places of label " << label;
    label_places_.insert({label, places});
  }

  Sink::callAll(sinks_, msg.timestamp_ns, delta, offsets, label_places_);
}

void Place2dSegmenter::updateGraph(uint64_t timestamp_ns,
                                   const ActiveWindowOutput& /*msg*/,
                                   const kimera_pgmo::MeshOffsetInfo& offsets,
                                   DynamicSceneGraph& graph) {
  // Remove old empty nodes
  for (const auto& nid : to_remove_) {
    graph.removeNode(nid);
  }

  to_remove_.clear();

  VLOG(5) << "[2D Places] updateGraph";
  std::map<uint32_t, std::set<NodeId>> active_places_to_check;
  for (const auto& label : config.labels) {
    active_places_to_check[label] = std::set<NodeId>();
  }

  LabelToNodes new_active_places;
  for (const auto& label : config.labels) {
    new_active_places[label] = std::set<NodeId>();
  }

  for (const auto& [label, nodes] : active_places_) {
    for (const auto nid : nodes) {
      auto& attrs = graph.getNode(nid).attributes<Place2dNodeAttributes>();
      if (placeIsEmpty(attrs)) {
        graph.removeNode(nid);  // Remove dangling places
        continue;
      }

      if (attrs.pcl_min_index >= offsets.archived_vertices) {
        graph.removeNode(nid);  // drop all previously active nodes
      } else {
        active_places_to_check.at(label).insert(nid);
      }
    }
  }

  for (const auto& [label, places] : label_places_) {
    for (const auto& place : places) {
      NodeSymbol ns = addPlaceToGraph(graph, place, label, timestamp_ns);
      active_places_to_check.at(label).insert(ns);
    }
  }

  std::set<std::pair<uint32_t, NodeId>> full_nodes;
  for (const auto& [label, nodes] : active_places_to_check) {
    for (const auto nid : nodes) {
      full_nodes.insert({label, nid});
    }
  }

  for (const auto& [label, nodes] : semiactive_places_) {
    for (const auto nid : nodes) {
      full_nodes.insert(std::pair<uint32_t, NodeId>(label, nid));
    }
  }

  std::map<uint32_t, std::set<NodeId>> new_semiactive_places;
  for (const auto& label : config.labels) {
    new_semiactive_places[label] = std::set<NodeId>();
  }

  for (const auto& [label, ns1] : full_nodes) {
    auto& attrs1 = graph.getNode(ns1).attributes<Place2dNodeAttributes>();
    attrs1.has_active_mesh_indices = attrs1.pcl_max_index >= offsets.archived_vertices;

    bool fixed_neighbors = true;
    for (const auto& [neighbor_label, ns2] : full_nodes) {
      if (ns1 == ns2) {
        continue;
      }

      EdgeAttributes ea;
      const auto& attrs2 = graph.getNode(ns2).attributes<Place2dNodeAttributes>();
      const bool has_edge = shouldAddPlaceConnection(attrs1,
                                                     attrs2,
                                                     config.place_overlap_threshold,
                                                     config.place_max_neighbor_z_diff,
                                                     ea);
      if (has_edge) {
        graph.insertEdge(ns1, ns2, ea.clone());
        fixed_neighbors &= attrs2.pcl_min_index < offsets.archived_vertices;
      }
    }

    if (!attrs1.has_active_mesh_indices && fixed_neighbors) {
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
    LOG(ERROR) << "[2D Places] Encountered empty place with label"
               << static_cast<int>(label) << " @ " << timestamp << "[ns]";
    return next_node_id_;
  }

  auto attrs = std::make_unique<Place2dNodeAttributes>();

  pcl::PointXYZ centroid;
  place.centroid.get(centroid);
  attrs->position << centroid.x, centroid.y, centroid.z;
  attrs->is_active = true;

  attrs->semantic_label = label;
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
  attrs->need_finish_merge = false;

  attrs->pcl_mesh_connections.insert(
      attrs->pcl_mesh_connections.begin(), place.indices.begin(), place.indices.end());

  graph.emplaceNode(config.layer, next_node_id_, std::move(attrs));

  active_places_.at(label).insert(next_node_id_);
  active_place_timestamps_[next_node_id_] = timestamp;
  return next_node_id_++;
}

void declare_config(Place2dSegmenter::Config& config) {
  using namespace config;
  name("Place2dSegmenterConfig");
  field(config.layer, "layer");
  field<CharConversion>(config.prefix, "prefix");
  field(config.clustering, "clustering", false);
  field(config.pure_final_place_size, "pure_final_place_size");
  field(config.min_final_place_points, "min_final_place_points");
  field(config.place_overlap_threshold, "place_overlap_threshold");
  field(config.place_max_neighbor_z_diff, "place_max_neighbor_z_diff");
  field(config.connection_ellipse_scale_factor, "connection_ellipse_scale_factor");
  field(config.sinks, "sinks");
  config.labels = GlobalInfo::instance().getLabelSpaceConfig().surface_places_labels;
}

}  // namespace hydra
