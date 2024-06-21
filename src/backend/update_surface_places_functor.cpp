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
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>

#include "hydra/backend/backend_utilities.h"
#include "hydra/backend/surface_place_utilities.h"
#include "hydra/frontend/place_2d_split_logic.h"
#include "hydra/utils/place_2d_ellipsoid_math.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using timing::ScopedTimer;
using SemanticLabel = SemanticNodeAttributes::Label;
using MergeId = std::optional<NodeId>;

NodeAttributes::Ptr merge2dPlaceAttributes(const DynamicSceneGraph& graph,
                                           const std::vector<NodeId>& nodes) {
  if (nodes.empty()) {
    return nullptr;
  }

  auto iter = nodes.begin();
  CHECK(graph.hasNode(*iter)) << NodeSymbol(*iter).getLabel();
  auto attrs_ptr = graph.getNode(*iter).attributes().clone();
  auto& new_attrs =
      *CHECK_NOTNULL(dynamic_cast<Place2dNodeAttributes*>(attrs_ptr.get()));
  ++iter;

  while (iter != nodes.end()) {
    CHECK(graph.hasNode(*iter)) << NodeSymbol(*iter).getLabel();
    const auto& from_attrs = graph.getNode(*iter).attributes<Place2dNodeAttributes>();
    utils::mergeIndices(from_attrs.pcl_mesh_connections,
                        new_attrs.pcl_mesh_connections);

    new_attrs.need_finish_merge = true;
    new_attrs.need_cleanup_splitting = true;
    new_attrs.has_active_mesh_indices |= from_attrs.has_active_mesh_indices;
    ++iter;
  }

  return attrs_ptr;
}

Update2dPlacesFunctor::Update2dPlacesFunctor(const Config& config) : config_(config) {}

void Update2dPlacesFunctor::updateNode(const spark_dsg::Mesh::Ptr& mesh,
                                       NodeId node,
                                       Place2dNodeAttributes& attrs) const {
  const auto& connections = attrs.pcl_mesh_connections;
  if (connections.empty()) {
    LOG(ERROR) << "Found empty place2d node " << NodeSymbol(node).getLabel();
    return;
  }

  pcl::CentroidPoint<pcl::PointXYZ> centroid;
  for (const auto& midx : connections) {
    const auto& point = mesh->pos(midx);
    if (std::isnan(point.x()) || std::isnan(point.y()) || std::isnan(point.z())) {
      VLOG(4) << "found nan at index: " << midx << " with point: [" << point.x() << ", "
              << point.y() << ", " << point.z() << "]";
      continue;
    }

    centroid.add(pcl::PointXYZ(point.x(), point.y(), point.z()));
  }

  if (!centroid.getSize()) {
    VLOG(2) << "Invalid centroid for 2D place " << NodeSymbol(node).getLabel();
    return;
  }

  pcl::PointXYZ pcl_pos;
  centroid.get(pcl_pos);
  attrs.position << pcl_pos.x, pcl_pos.y, pcl_pos.z;

  for (size_t i = 0; i < attrs.boundary.size(); ++i) {
    const auto& point = mesh->pos(attrs.pcl_boundary_connections.at(i));
    attrs.boundary[i] << point.x(), point.y(), point.z();
  }
}

bool Update2dPlacesFunctor::shouldMerge(const Place2dNodeAttributes& from_attrs,
                                        const Place2dNodeAttributes& to_attrs) const {
  if (std::abs(from_attrs.position(2) - to_attrs.position(2)) >
      config_.merge_max_delta_z) {
    return false;
  }

  double overlap_distance = hydra::ellipse::getEllipsoidTransverseOverlapDistance(
      from_attrs.ellipse_matrix_compress,
      from_attrs.ellipse_centroid.head(2),
      to_attrs.ellipse_matrix_compress,
      to_attrs.ellipse_centroid.head(2));
  return overlap_distance > 0;
}

MergeId Update2dPlacesFunctor::proposeMerge(const SceneGraphLayer& layer,
                                            const SceneGraphNode& node) const {
  const auto& from_attrs = node.attributes<Place2dNodeAttributes>();
  const auto iter = node_finders.find(from_attrs.semantic_label);
  if (iter == node_finders.end()) {
    return std::nullopt;
  }

  std::list<NodeId> candidates;
  (*iter).second->find(from_attrs.position,
                       num_merges_to_consider,
                       !from_attrs.is_active,
                       [&candidates](NodeId object_id, size_t, double) {
                         candidates.push_back(object_id);
                       });

  for (const auto& id : candidates) {
    if (layer.hasEdge(node.id, id)) {
      continue;  // avoid merging siblings
    }

    const auto& to_attrs = layer.getNode(id).attributes<Place2dNodeAttributes>();
    if (shouldMerge(from_attrs, to_attrs)) {
      return id;
    }
  }

  return std::nullopt;
}

void Update2dPlacesFunctor::cleanup(SharedDsgInfo& dsg) const {
  std::map<NodeId, std::set<NodeId>> node_neighbors;
  std::vector<std::pair<NodeId, Place2d>> place_2ds;

  // Get/copy info for places that need cleanup
  std::unique_lock<std::mutex> lock(dsg.mutex);
  const SceneGraphLayer& places_layer = dsg.graph->getLayer(DsgLayers::MESH_PLACES);
  utils::getPlace2dAndNeighors(places_layer, place_2ds, node_neighbors);
  lock.unlock();

  // Decide which places need to be split and which just need to be updated
  auto& graph = *dsg.graph;
  auto mesh = graph.mesh();

  // existing node id for each place
  std::vector<std::pair<NodeId, Place2d>> nodes_to_update;
  // node id that new nodes split from (necessary to copy other
  // place info)
  std::vector<std::pair<NodeId, std::vector<Place2d>>> nodes_to_add;
  utils::getNecessaryUpdates(*mesh,
                             config_.min_points,
                             config_.min_size,
                             config_.connection_ellipse_scale_factor,
                             place_2ds,
                             nodes_to_update,
                             nodes_to_add);
  auto edge_map = utils::buildEdgeMap(nodes_to_add,
                                      config_.connection_overlap_threshold,
                                      config_.connection_max_delta_z);

  lock.lock();
  // Update attributes for place nodes that did not need to split after merge
  utils::updateExistingNodes(nodes_to_update, graph);

  // Insert new nodes that are formed by splitting existing nodes (and delete
  // previous node)
  std::map<std::tuple<size_t, size_t>, NodeId> new_id_map;
  next_node_id_ = utils::insertNewNodes(nodes_to_add,
                                        config_.connection_overlap_threshold,
                                        config_.connection_max_delta_z,
                                        next_node_id_,
                                        graph,
                                        new_id_map);

  // Add edges between new nodes
  utils::addNewNodeEdges(nodes_to_add, edge_map, new_id_map, graph);

  std::unordered_map<NodeId, bool> checked_nodes;
  // Clean up places that are far enough away from the active window
  // Far enough means that none of a node's neighbors or the node itself have
  // active mesh vertices
  for (auto& id_node_pair : places_layer.nodes()) {
    auto& attrs = id_node_pair.second->attributes<Place2dNodeAttributes>();
    if (attrs.need_cleanup_splitting && !attrs.is_active) {
      bool has_active_neighbor = false;
      for (NodeId nid : id_node_pair.second->siblings()) {
        auto& neighbor_attrs = graph.getNode(nid).attributes<Place2dNodeAttributes>();
        if (attrs.semantic_label != neighbor_attrs.semantic_label) {
          continue;
        }
        if (neighbor_attrs.has_active_mesh_indices) {
          has_active_neighbor = true;
          break;
        }
      }
      checked_nodes[id_node_pair.first] =
          !has_active_neighbor && !attrs.has_active_mesh_indices;
      for (NodeId nid : id_node_pair.second->siblings()) {
        auto& neighbor_attrs = graph.getNode(nid).attributes<Place2dNodeAttributes>();

        auto search = checked_nodes.find(nid);
        if (search == checked_nodes.end()) {
          checked_nodes.insert({nid, false});
        }

        if (neighbor_attrs.is_active) {
          continue;
        }

        if (attrs.semantic_label == neighbor_attrs.semantic_label) {
          utils::reallocateMeshPoints(mesh->points, attrs, neighbor_attrs);
        }
      }
    }
  }

  for (auto& id_finalize : checked_nodes) {
    auto& attrs = graph.getNode(id_finalize.first).attributes<Place2dNodeAttributes>();

    if (attrs.pcl_mesh_connections.size() == 0) {
      LOG(ERROR) << "Reallocating mesh points would make empty place. Skipping.";
      continue;
    }

    addRectInfo(mesh->points, config_.connection_ellipse_scale_factor, attrs);
    addBoundaryInfo(mesh->points, attrs);
  }

  std::vector<std::pair<NodeId, NodeId>> edges_to_remove;
  std::map<NodeId, NodeId> extra_edges_to_check;
  for (auto& id_finalize : checked_nodes) {
    NodeId id = id_finalize.first;
    bool finalize = id_finalize.second;
    auto& attrs1 = graph.getNode(id).attributes<Place2dNodeAttributes>();

    for (auto& neighbor_id_final : checked_nodes) {
      NodeId nid = neighbor_id_final.first;
      auto& neighbor_node = graph.getNode(nid);
      auto& attrs2 = neighbor_node.attributes<Place2dNodeAttributes>();
      EdgeAttributes ea;
      if (!shouldAddPlaceConnection(attrs1,
                                    attrs2,
                                    config_.connection_overlap_threshold,
                                    config_.connection_max_delta_z,
                                    ea)) {
        graph.removeEdge(id, nid);
      } else {
        graph.insertEdge(id, nid, ea.clone());
      }
    }

    std::vector<std::pair<NodeId, NodeId>> sibs_edges_to_remove;
    for (auto& nid : graph.getNode(id).siblings()) {
      auto neighbor_node = graph.findNode(nid);
      Place2dNodeAttributes& attrs2 =
          neighbor_node->attributes<Place2dNodeAttributes>();
      EdgeAttributes ea;
      if (!shouldAddPlaceConnection(attrs1,
                                    attrs2,
                                    config_.connection_overlap_threshold,
                                    config_.connection_max_delta_z,
                                    ea)) {
        sibs_edges_to_remove.push_back({id, nid});
      }
    }

    for (auto n1_n2 : sibs_edges_to_remove) {
      graph.removeEdge(n1_n2.first, n1_n2.second);
    }

    if (finalize) {
      attrs1.need_cleanup_splitting = false;
    }
  }
}

UpdateFunctor::Hooks Update2dPlacesFunctor::hooks() const {
  auto my_hooks = UpdateFunctor::hooks();
  my_hooks.cleanup = [this](const UpdateInfo::ConstPtr&, SharedDsgInfo* dsg) {
    if (dsg) {
      cleanup(*dsg);
    }
  };

  my_hooks.merge = [](const DynamicSceneGraph& graph,
                      const std::vector<NodeId>& nodes) {
    return merge2dPlaceAttributes(graph, nodes);
  };

  return my_hooks;
}

MergeList Update2dPlacesFunctor::call(const DynamicSceneGraph& unmerged,
                                      SharedDsgInfo& dsg,
                                      const UpdateInfo::ConstPtr& info) const {
  ScopedTimer spin_timer("backend/update_2d_places", info->timestamp_ns);
  if (!unmerged.hasLayer(layer_id_)) {
    return {};
  }

  const auto mesh = unmerged.mesh();
  const auto new_loopclosure = info->loop_closure_detected;
  const auto& layer = unmerged.getLayer(layer_id_);
  makeSemanticNodeFinders(layer, node_finders);
  const auto view = new_loopclosure ? LayerView(layer) : active_tracker.view(layer);

  size_t num_changed = 0;
  for (const auto& node : view) {
    ++num_changed;
    auto& attrs = node.attributes<Place2dNodeAttributes>();
    updateNode(mesh, node.id, attrs);
    dsg.graph->setNodeAttributes(node.id, attrs.clone());
  }

  MergeList nodes_to_merge;
  if (info->allow_node_merging && config_.allow_places_merge) {
    for (const auto& node : view) {
      const auto to_merge = proposeMerge(layer, node);
      if (to_merge) {
        nodes_to_merge.push_back({node.id, *to_merge});
      }
    }
  }

  VLOG(5) << "[Hydra Backend] 2D Place update: " << num_changed << " node(s)";
  active_tracker.clear();
  return nodes_to_merge;
}

void declare_config(Update2dPlacesFunctor::Config& config) {
  using namespace config;
  name("Places2dConfig");
  field(config.allow_places_merge, "allow_places_merge");
  field(config.merge_max_delta_z, "merge_max_delta_z");
  field(config.min_points, "min_points");
  field(config.min_size, "min_size");
  field(config.connection_overlap_threshold, "connection_overlap_threshold");
  field(config.connection_max_delta_z, "connection_max_delta_z");
  field(config.connection_ellipse_scale_factor, "connection_ellipse_scale_factor");
}

}  // namespace hydra
