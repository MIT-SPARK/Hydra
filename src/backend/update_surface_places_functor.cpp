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
#include "hydra/backend/surface_place_utilities.h"
#include "hydra/frontend/place_2d_split_logic.h"
#include "hydra/utils/place_2d_ellipsoid_math.h"
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
  const auto& connections = attrs.pcl_mesh_connections;
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
    attrs.boundary[i] = mesh.pos(attrs.pcl_boundary_connections.at(i)).cast<double>();
  }
}

}  // namespace

using timing::ScopedTimer;

void declare_config(Update2dPlacesFunctor::Config& config) {
  using namespace config;
  name("Update2dPlacesFunctor::Config");
  field(config.layer, "layer");
  field(config.merge_max_delta_z, "merge_max_delta_z");
  field(config.connection_overlap_threshold, "connection_overlap_threshold");
  field(config.connection_max_delta_z, "connection_max_delta_z");
  field(config.connection_ellipse_scale_factor, "connection_ellipse_scale_factor");
  field(config.merge_proposer, "merge_proposer");
}

NodeAttributes::Ptr merge2dPlaceAttributes(const Update2dPlacesFunctor::Config config,
                                           const DynamicSceneGraph& graph,
                                           const std::vector<NodeId>& nodes) {
  if (nodes.empty()) {
    return nullptr;
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
    utils::mergeIndices(from_attrs.pcl_mesh_connections,
                        new_attrs.pcl_mesh_connections);

    new_attrs.has_active_mesh_indices |= from_attrs.has_active_mesh_indices;
    ++iter;
  }

  addRectInfo(*graph.mesh(), config.connection_ellipse_scale_factor, new_attrs);
  addBoundaryInfo(*graph.mesh(), new_attrs);
  return attrs_ptr;
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
    return merge2dPlaceAttributes(config, graph, nodes);
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

  const auto new_loopclosure = info->loop_closure_detected;

  active_tracker.clear();  // reset from previous pass
  const auto view = new_loopclosure ? LayerView(*layer) : active_tracker.view(*layer);

  size_t num_changed = 0;
  const auto mesh = unmerged.mesh();
  for (const auto& node : view) {
    auto attrs = node.tryAttributes<Place2dNodeAttributes>();
    if (!attrs) {
      LOG(WARNING) << "Node " << NodeSymbol(node.id).str() << " is not a 2D place!";
      continue;
    }

    ++num_changed;
    updateNode(*mesh, node.id, *attrs);
    dsg.graph->setNodeAttributes(node.id, attrs->clone());
  }

  VLOG(5) << "[Hydra Backend] 2D Place update: " << num_changed << " node(s)";
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
          LOG(WARNING) << "Invalid 2D Place nodes: " << NodeSymbol(lhs.id).str() << ", "
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

  double overlap_distance = hydra::ellipse::getEllipsoidTransverseOverlapDistance(
      from_attrs.ellipse_matrix_compress,
      from_attrs.ellipse_centroid.head(2),
      to_attrs.ellipse_matrix_compress,
      to_attrs.ellipse_centroid.head(2));
  return overlap_distance > 0;
}

void Update2dPlacesFunctor::cleanup(SharedDsgInfo& dsg) const {
  std::lock_guard<std::mutex> lock(dsg.mutex);
  const auto places_layer = dsg.graph->findLayer(DsgLayers::MESH_PLACES);
  if (!places_layer) {
    return;
  }

  // Decide which places need to be split and which just need to be updated
  auto& graph = *dsg.graph;
  auto mesh = graph.mesh();

  // Clean up places that are far enough away from the active window
  // Far enough means that none of a node's neighbors or the node itself have
  // active mesh vertices
  std::set<NodeId> checked_nodes;
  for (auto& [node_id, node] : places_layer->nodes()) {
    auto attrs = node->tryAttributes<Place2dNodeAttributes>();
    if (!attrs || attrs->is_active) {
      continue;
    }

    for (const auto nid : node->siblings()) {
      auto& neighbor_attrs = graph.getNode(nid).attributes<Place2dNodeAttributes>();
      if (neighbor_attrs.is_active) {
        continue;
      }

      if (attrs->semantic_label == neighbor_attrs.semantic_label) {
        checked_nodes.insert(node_id);
        checked_nodes.insert(nid);
        utils::reallocateMeshPoints(*mesh, *attrs, neighbor_attrs);
      }
    }
  }

  for (auto& node_id : checked_nodes) {
    auto& attrs = graph.getNode(node_id).attributes<Place2dNodeAttributes>();
    if (attrs.pcl_mesh_connections.size() == 0) {
      LOG(ERROR) << "Reallocating mesh points would make empty place. Skipping.";
      continue;
    }

    addRectInfo(*mesh, config.connection_ellipse_scale_factor, attrs);
    addBoundaryInfo(*mesh, attrs);
  }

  std::vector<std::pair<NodeId, NodeId>> edges_to_remove;
  std::map<NodeId, NodeId> extra_edges_to_check;
  for (const auto& node_id : checked_nodes) {
    auto& attrs1 = graph.getNode(node_id).attributes<Place2dNodeAttributes>();

    for (const auto& neighbor_id : checked_nodes) {
      const auto& neighbor_node = graph.getNode(neighbor_id);
      auto& attrs2 = neighbor_node.attributes<Place2dNodeAttributes>();
      EdgeAttributes ea;
      if (!shouldAddPlaceConnection(attrs1,
                                    attrs2,
                                    config.connection_overlap_threshold,
                                    config.connection_max_delta_z,
                                    ea)) {
        graph.removeEdge(node_id, neighbor_id);
      } else {
        graph.insertEdge(node_id, neighbor_id, ea.clone());
      }
    }

    std::vector<std::pair<NodeId, NodeId>> sibs_edges_to_remove;
    for (auto& nid : graph.getNode(node_id).siblings()) {
      auto neighbor_node = graph.findNode(nid);
      auto& attrs2 = neighbor_node->attributes<Place2dNodeAttributes>();
      EdgeAttributes ea;
      if (!shouldAddPlaceConnection(attrs1,
                                    attrs2,
                                    config.connection_overlap_threshold,
                                    config.connection_max_delta_z,
                                    ea)) {
        sibs_edges_to_remove.push_back({node_id, nid});
      }
    }

    for (auto [source, target] : sibs_edges_to_remove) {
      graph.removeEdge(source, target);
    }
  }
}

}  // namespace hydra
