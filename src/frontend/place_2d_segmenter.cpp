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
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <kimera_pgmo/mesh_delta.h>

#include <memory>

#include "hydra/common/global_info.h"
#include "hydra/frontend/place_2d_split_logic.h"
#include "hydra/utils/place_2d_ellipsoid_math.h"

namespace hydra {

using spark_dsg::DynamicSceneGraph;
using spark_dsg::EdgeAttributes;
using spark_dsg::NodeId;
using spark_dsg::Place2dNodeAttributes;

using AttrMap = std::map<NodeId, Place2dNodeAttributes>;

namespace {

static const auto registration_ =
    config::RegistrationWithConfig<Place2dSegmenter,
                                   Place2dSegmenter,
                                   Place2dSegmenter::Config>("place_2d");

inline bool placeIsEmpty(const Place2dNodeAttributes& attrs) {
  return attrs.pcl_mesh_connections.size() == 0 || attrs.boundary.size() < 3;
}

std::unordered_set<size_t> getFrozenSet(uint64_t timestamp_ns,
                                        const kimera_pgmo::MeshOffsetInfo& offsets,
                                        AttrMap& active_places,
                                        std::list<NodeId>& to_remove) {
  std::unordered_set<size_t> frozen_indices;

  auto iter = active_places.begin();
  while (iter != active_places.end()) {
    auto& [node_id, attrs] = *iter;
    remapPlace2dMesh(attrs, offsets);

    // if remapping results in vertices disappearing, we drop the place
    if (placeIsEmpty(attrs)) {
      to_remove.push_back(node_id);
      iter = active_places.erase(iter);
      continue;
    }

    // we delete any previous place with only active mesh vertices
    if (attrs.pcl_min_index >= offsets.archived_vertices) {
      to_remove.push_back(node_id);
      iter = active_places.erase(iter);
      continue;
    }

    attrs.last_update_time_ns = timestamp_ns;
    attrs.has_active_mesh_indices = attrs.pcl_max_index >= offsets.archived_vertices;
    for (auto idx : attrs.pcl_mesh_connections) {
      if (idx >= offsets.archived_vertices) {
        frozen_indices.insert(offsets.toLocalVertex(idx));
      }
    }

    ++iter;
  }

  VLOG(5) << "[2D Places] n frozen indices: " << frozen_indices.size();
  return frozen_indices;
}

}  // namespace

Place2dSegmenter::Place2dSegmenter(const Config& config)
    : config(config::checkValid(config)),
      sinks_(Sink::instantiate(config.sinks)),
      next_node_id_(config.prefix, 0) {
  VLOG(1) << "[2D Places] Using labels: " << clustering::printLabels(config.labels);
}

void Place2dSegmenter::detect(const ActiveWindowOutput& msg,
                              const kimera_pgmo::MeshDelta& delta,
                              const kimera_pgmo::MeshOffsetInfo& offsets) {
  VLOG(5) << "[2D Places] detect called";
  VLOG(5) << "[2D Places] n original active indices: " << delta.getNumActiveVertices();
  const auto frozen =
      getFrozenSet(msg.timestamp_ns, offsets, active_places_, to_remove_);
  const auto label_indices = clustering::getLabelIndices(config.labels, delta, &frozen);
  if (label_indices.empty()) {
    VLOG(5) << "[2D Places] No vertices found matching desired labels";
    return;
  }

  for (const auto& [label, indices] : label_indices) {
    if (indices.size() < config.clustering.min_cluster_size) {
      continue;
    }

    auto clusters = clustering::findClusters(config.clustering, delta, indices);
    VLOG(5) << "[2D Places] got " << clusters.size() << " initial places";
    std::vector<Place2d> places;
    for (auto& cluster_indices : clusters) {
      Place2d place;
      for (const auto& idx : cluster_indices) {
        place.indices.push_back(offsets.toGlobalVertex(idx));
      }

      // Set up initial bounds from delta
      addRectInfo(delta, offsets, config.connection_ellipse_scale_factor, place);

      // Recursively decompose initial place into smaller places
      decomposePlace(delta,
                     offsets,
                     place,
                     config.pure_final_place_size,
                     config.min_final_place_points,
                     config.connection_ellipse_scale_factor,
                     places);
    }

    VLOG(5) << "[2D Places] " << places.size() << " final places of label " << label;
    for (const auto& place : places) {
      if (place.indices.empty()) {
        LOG(ERROR) << "[2D Places] Encountered empty place with label" << label << " @ "
                   << msg.timestamp_ns << "[ns]";
        continue;
      }

      auto& attrs = active_places_[next_node_id_];
      ++next_node_id_;

      // add attributes to active place
      place.fillAttributes(attrs);
      attrs.last_update_time_ns = msg.timestamp_ns;
      attrs.semantic_label = label;
      attrs.is_active = true;
      attrs.has_active_mesh_indices = true;
      attrs.need_finish_merge = false;
    }
  }

  Sink::callAll(sinks_, msg.timestamp_ns, delta, offsets);
}

void Place2dSegmenter::updateGraph(const ActiveWindowOutput&,
                                   const kimera_pgmo::MeshOffsetInfo& offsets,
                                   DynamicSceneGraph& graph) {
  // Remove old empty nodes and previous active nodes
  for (const auto& nid : to_remove_) {
    graph.removeNode(nid);
  }

  to_remove_.clear();

  VLOG(5) << "[2D Places] updateGraph";
  for (const auto& [nid, attrs] : active_places_) {
    // overrides all previous nodes
    graph.addOrUpdateNode(config.layer, nid, attrs.clone());
  }

  auto iter = active_places_.begin();
  while (iter != active_places_.end()) {
    bool fixed_neighbors = true;
    const auto ns1 = iter->first;
    auto& attrs1 = graph.getNode(ns1).attributes<Place2dNodeAttributes>();
    for (const auto& [ns2, attrs2] : active_places_) {
      if (ns1 == ns2) {
        continue;
      }

      auto ea = std::make_unique<EdgeAttributes>();
      const bool has_edge = shouldAddPlaceConnection(attrs1,
                                                     attrs2,
                                                     config.place_overlap_threshold,
                                                     config.place_max_neighbor_z_diff,
                                                     *ea);
      if (has_edge) {
        graph.insertEdge(ns1, ns2, std::move(ea));
        fixed_neighbors &= attrs2.pcl_min_index < offsets.archived_vertices;
      }
    }

    if (!attrs1.has_active_mesh_indices && fixed_neighbors) {
      attrs1.is_active = false;
      iter = active_places_.erase(iter);
    } else {
      ++iter;
    }
  }
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
