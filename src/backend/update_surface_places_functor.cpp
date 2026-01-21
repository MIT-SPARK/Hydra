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
#include "hydra/backend/update_surface_places_functor.h"

#include <config_utilities/config.h>
#include <glog/logging.h>

#include "hydra/backend/backend_utilities.h"
#include "hydra/places/2d_places/ellipsoid_math.h"
#include "hydra/places/2d_places/index_remapping.h"
#include "hydra/places/2d_places/place_reallocation.h"
#include "hydra/places/2d_places/place_splitting.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {
namespace {

static const auto reg = config::RegistrationWithConfig<UpdateFunctor,
                                                       Update2dPlacesFunctor,
                                                       Update2dPlacesFunctor::Config>(
    "Update2dPlacesFunctor");

inline void updateNode(const spark_dsg::Mesh& mesh,
                       NodeId node,
                       Place2dNodeAttributes& attrs) {
  const auto& connections = attrs.mesh_connections;
  if (connections.empty()) {
    LOG(ERROR) << "Found empty place2d node " << NodeSymbol(node).str();
    return;
  }

  attrs.position.setZero();
  for (const auto idx : connections) {
    attrs.position += mesh.pos(idx).cast<double>();
  }

  attrs.position /= connections.size();
  for (size_t i = 0; i < attrs.boundary.size(); ++i) {
    attrs.boundary[i] = mesh.pos(attrs.boundary_connections.at(i)).cast<double>();
  }
}

NodeAttributes::Ptr merge2dPlaceAttributes(const Update2dPlacesFunctor::Config config,
                                           const DynamicSceneGraph& graph,
                                           const std::vector<NodeId>& nodes) {
  if (nodes.empty()) {
    return nullptr;
  }

  for (const auto node_id : nodes) {
  }
  auto iter = nodes.begin();
  CHECK(graph.hasNode(*iter)) << NodeSymbol(*iter).str();
  auto attrs_ptr = graph.getNode(*iter).attributes().clone();
  auto& new_attrs =
      *CHECK_NOTNULL(dynamic_cast<Place2dNodeAttributes*>(attrs_ptr.get()));
  ++iter;

  while (iter != nodes.end()) {
    CHECK(graph.hasNode(*iter)) << NodeSymbol(*iter).str();
    const auto& from_attrs = graph.getNode(*iter).attributes<Place2dNodeAttributes>();
    utils::mergeIndices(from_attrs.mesh_connections, new_attrs.mesh_connections);
    new_attrs.has_active_mesh_indices |= from_attrs.has_active_mesh_indices;
    ++iter;
  }

  addRectInfo(*graph.mesh(), config.connection_ellipse_scale_factor, new_attrs);
  addBoundaryInfo(*graph.mesh(), new_attrs);
  return attrs_ptr;
}

}  // namespace

using kimera_pgmo::MeshOffsetInfo;
using timing::ScopedTimer;

void declare_config(Update2dPlacesFunctor::Config& config) {
  using namespace config;
  name("Update2dPlacesFunctor::Config");
  base<VerbosityConfig>(config);
  field(config.layer, "layer");
  field(config.merge_max_delta_z, "merge_max_delta_z");
  field(config.connection_overlap_threshold, "connection_overlap_threshold");
  field(config.connection_max_delta_z, "connection_max_delta_z");
  field(config.connection_ellipse_scale_factor, "connection_ellipse_scale_factor");
  field(config.merge_proposer, "merge_proposer");
}

Update2dPlacesFunctor::Update2dPlacesFunctor(const Config& config)
    : config(config), merge_proposer(config.merge_proposer) {}

UpdateFunctor::Hooks Update2dPlacesFunctor::hooks() const {
  auto my_hooks = UpdateFunctor::hooks();
  my_hooks.cleanup = [this](const auto&, auto&, auto dsg) {
    if (dsg) {
      cleanup(*dsg);
    }
  };

  my_hooks.find_merges = [this](const auto& graph, const auto& info) {
    return findMerges(graph, info);
  };

  my_hooks.merge = [this](const auto& graph, const auto& nodes) {
    cleanup_nodes.insert(nodes.begin(), nodes.end());
    return merge2dPlaceAttributes(config, graph, nodes);
  };

  my_hooks.mesh_update = [this](const auto& graph, const auto& offsets) {
    updateMeshIndices(graph, offsets);
  };

  return my_hooks;
}

void Update2dPlacesFunctor::call(const DynamicSceneGraph& unmerged,
                                 SharedDsgInfo& dsg,
                                 const UpdateInfo::ConstPtr& info) const {
  ScopedTimer spin_timer("backend/update_2d_places", info->timestamp_ns);
  const auto layer = unmerged.findLayer(config.layer);
  if (!layer) {
    return;
  }

  active_tracker.clear();  // reset from previous pass
  const auto new_loopclosure = info->loop_closure_detected;
  const auto view = new_loopclosure ? LayerView(*layer) : active_tracker.view(*layer);

  size_t num_changed = 0;
  const auto mesh = unmerged.mesh();
  for (const auto& node : view) {
    auto attrs = node.tryAttributes<Place2dNodeAttributes>();
    if (!attrs) {
      MLOG(2) << "node " << NodeSymbol(node.id).str() << " is not a 2D place!";
      continue;
    }

    // note that updateNode recomputes the centroid from the frontend mesh indices,
    // not the set of merged indices (which would break the cleanup logic).
    ++num_changed;
    updateNode(*mesh, node.id, *attrs);
    dsg.graph->setNodeAttributes(node.id, attrs->clone());
  }

  MLOG(1) << "updated " << num_changed << " node(s)";
}

MergeList Update2dPlacesFunctor::findMerges(const DynamicSceneGraph& graph,
                                            const UpdateInfo::ConstPtr& info) const {
  const auto layer = graph.findLayer(config.layer);
  if (!layer) {
    return {};
  }

  const auto new_lcd = info->loop_closure_detected;
  // freeze layer view to avoid messing with tracker
  const auto view = new_lcd ? LayerView(*layer) : active_tracker.view(*layer, true);

  MergeList nodes_to_merge;
  merge_proposer.findMerges(
      *layer,
      view,
      [this](const SceneGraphNode& lhs, const SceneGraphNode& rhs) {
        auto lhs_attrs = lhs.tryAttributes<Place2dNodeAttributes>();
        auto rhs_attrs = rhs.tryAttributes<Place2dNodeAttributes>();
        if (!lhs_attrs || !rhs_attrs) {
          MLOG(2) << "invalid 2D place nodes: " << NodeSymbol(lhs.id).str() << ", "
                  << NodeSymbol(rhs.id).str();
          return false;
        }

        return shouldMerge(*lhs_attrs, *rhs_attrs);
      },
      nodes_to_merge);
  return nodes_to_merge;
}

bool Update2dPlacesFunctor::shouldMerge(const Place2dNodeAttributes& from_attrs,
                                        const Place2dNodeAttributes& to_attrs) const {
  if (to_attrs.is_active) {
    return false;
  }

  const auto z_diff = std::abs(from_attrs.position(2) - to_attrs.position(2));
  if (z_diff > config.merge_max_delta_z) {
    return false;
  }

  const auto overlap_distance = ellipse::getEllipsoidTransverseOverlapDistance(
      from_attrs.ellipse_matrix_compress,
      from_attrs.ellipse_centroid.head(2),
      to_attrs.ellipse_matrix_compress,
      to_attrs.ellipse_centroid.head(2));
  return overlap_distance > 0.0;
}

void Update2dPlacesFunctor::cleanup(SharedDsgInfo& dsg) const {
  auto& graph = *dsg.graph;
  auto mesh = graph.mesh();

  std::lock_guard<std::mutex> lock(dsg.mutex);
  const auto places_layer = graph.findLayer(DsgLayers::MESH_PLACES);
  if (!places_layer || !mesh) {
    return;
  }

  std::set<NodeId> checked_nodes;
  utils::propagateReallocation(*mesh, *places_layer, cleanup_nodes, checked_nodes);
  for (auto& node_id : checked_nodes) {
    auto& attrs = graph.getNode(node_id).attributes<Place2dNodeAttributes>();
    if (attrs.mesh_connections.size() == 0) {
      MLOG(1) << "place " << NodeSymbol(node_id).str() << " too small";
      continue;
    }

    addRectInfo(*mesh, config.connection_ellipse_scale_factor, attrs);
    addBoundaryInfo(*mesh, attrs);
  }

  // drop existing edges for all updated nodes that no longer make sense
  for (const auto node_id : checked_nodes) {
    const auto& node = graph.getNode(node_id);
    const auto& attrs = node.attributes<Place2dNodeAttributes>();
    const auto siblings = node.siblings();
    for (const auto other_id : siblings) {
      const auto other = graph.getNode(other_id).tryAttributes<Place2dNodeAttributes>();
      if (!other) {
        continue;
      }

      EdgeAttributes ea;
      if (!shouldAddPlaceConnection(attrs,
                                    *other,
                                    config.connection_overlap_threshold,
                                    config.connection_max_delta_z,
                                    ea)) {
        // this is safe because we clone the siblings before iterating
        graph.removeEdge(node_id, other_id);
      }
    }
  }

  // pairwise search for new edges
  for (const auto node_id : checked_nodes) {
    const auto& attrs = graph.getNode(node_id).attributes<Place2dNodeAttributes>();
    for (const auto other_id : checked_nodes) {
      if (node_id == other_id) {
        continue;
      }

      const auto& other = graph.getNode(other_id).attributes<Place2dNodeAttributes>();
      EdgeAttributes ea;
      if (shouldAddPlaceConnection(attrs,
                                   other,
                                   config.connection_overlap_threshold,
                                   config.connection_max_delta_z,
                                   ea)) {
        graph.insertEdge(node_id, other_id, ea.clone());
      }
    }
  }
}

void Update2dPlacesFunctor::updateMeshIndices(const DynamicSceneGraph& graph,
                                              const MeshOffsetInfo& offsets) const {
  const auto surface_places = graph.findLayer(config.layer);
  if (!surface_places) {
    return;
  }

  const auto& layer = *surface_places;
  for (auto& [node_id, node] : layer.nodes()) {
    auto attrs = node->tryAttributes<spark_dsg::Place2dNodeAttributes>();
    if (!attrs || !attrs->has_active_mesh_indices) {
      continue;
    }

    remap2dPlaceIndices(*attrs, offsets);
  }
}

}  // namespace hydra
