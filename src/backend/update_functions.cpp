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
#include "hydra/backend/update_functions.h"

#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <spark_dsg/bounding_box_extraction.h>
#include <spark_dsg/node_symbol.h>
#include <spark_dsg/scene_graph_types.h>

#include "hydra/frontend/place_2d_split_logic.h"
#include "hydra/rooms/room_finder.h"
#include "hydra/utils/mesh_utilities.h"
#include "hydra/utils/place_2d_ellipsoid_math.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {
namespace dsg_updates {

using timing::ScopedTimer;
using Node = SceneGraphNode;
using NodeColor = SemanticNodeAttributes::ColorVector;
using SemanticLabel = SemanticNodeAttributes::Label;

void filterPlace(DynamicSceneGraph& graph,
                 std::list<NodeId>& valid_candidates,
                 const NodeId& base_node,
                 const NodeId& candidate,
                 std::unordered_set<NodeId> layer_nodes) {
  // there must not be an edge between nodes and the candidate must not be merged
  if (!graph.hasEdge(base_node, candidate) && layer_nodes.count(candidate)) {
    valid_candidates.push_back(candidate);
  }
  return;
}

UpdateObjectsFunctor::UpdateObjectsFunctor() {}

UpdateFunctor::Hooks UpdateObjectsFunctor::hooks() const {
  auto my_hooks = UpdateFunctor::hooks();
  my_hooks.merge = [this](const DynamicSceneGraph& graph, NodeId from, NodeId to) {
    mergeAttributes(graph, from, to);
  };
  my_hooks.should_merge = [this](const NodeAttributes* lhs, const NodeAttributes* rhs) {
    return dispatchMergeCheck<ObjectNodeAttributes>(
        lhs,
        rhs,
        std::bind(&UpdateObjectsFunctor::shouldMerge,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));
  };
  my_hooks.node_update = [this](const UpdateInfo&,
                                const spark_dsg::Mesh::Ptr mesh,
                                NodeId node,
                                NodeAttributes* attrs) {
    auto oattrs = dynamic_cast<ObjectNodeAttributes*>(attrs);
    if (!oattrs || !mesh) {
      return;
    }

    updateObject(mesh, node, *oattrs);
  };

  return my_hooks;
}

size_t UpdateObjectsFunctor::makeNodeFinders(const SceneGraphLayer& layer) const {
  std::map<SemanticLabel, std::unordered_set<NodeId>> label_node_map;
  size_t archived = 0;
  for (const auto& id_node_pair : layer.nodes()) {
    auto& attrs = id_node_pair.second->attributes<SemanticNodeAttributes>();

    if (use_active_flag && attrs.is_active) {
      continue;
    }

    ++archived;
    auto iter = label_node_map.find(attrs.semantic_label);
    if (iter == label_node_map.end()) {
      iter = label_node_map.insert({attrs.semantic_label, {}}).first;
    }

    iter->second.insert(id_node_pair.first);
  }

  // creating nodefinders
  node_finders.clear();
  for (const auto& label_ids_pair : label_node_map) {
    node_finders.emplace(
        label_ids_pair.first,
        std::make_unique<NearestNodeFinder>(layer, label_ids_pair.second));
  }

  return archived;
}

void UpdateObjectsFunctor::updateObject(const spark_dsg::Mesh::Ptr& mesh,
                                        NodeId node,
                                        ObjectNodeAttributes& attrs) const {
  const auto& connections = attrs.mesh_connections;
  if (connections.empty()) {
    VLOG(2) << "Found empty object node " << NodeSymbol(node).getLabel();
    return;
  }

  std::vector<size_t> indices;
  if (invalid_indices) {
    for (const auto idx : connections) {
      if (!invalid_indices->count(idx)) {
        indices.push_back(idx);
      }
    }
  } else {
    indices.assign(connections.begin(), connections.end());
  }

  if (!updateObjectGeometry(*mesh, attrs, &indices)) {
    VLOG(2) << "Invalid centroid for object " << NodeSymbol(node).getLabel();
  }
}

bool UpdateObjectsFunctor::shouldMerge(const ObjectNodeAttributes& from_attrs,
                                       const ObjectNodeAttributes& to_attrs) const {
  return from_attrs.bounding_box.isInside(to_attrs.position) ||
         to_attrs.bounding_box.isInside(from_attrs.position);
}

std::optional<NodeId> UpdateObjectsFunctor::proposeObjectMerge(
    const SceneGraphLayer& layer,
    const ObjectNodeAttributes& from_attrs,
    bool skip_first) const {
  const auto iter = node_finders.find(from_attrs.semantic_label);
  if (iter == node_finders.end()) {
    return std::nullopt;
  }

  std::list<NodeId> candidates;
  (*iter).second->find(from_attrs.position,
                       num_merges_to_consider,
                       skip_first,
                       [&candidates](NodeId object_id, size_t, double) {
                         candidates.push_back(object_id);
                       });

  for (const auto& id : candidates) {
    const auto& to_attrs = layer.getNode(id)->get().attributes<ObjectNodeAttributes>();
    if (shouldMerge(from_attrs, to_attrs)) {
      return id;
    }
  }

  return std::nullopt;
}

MergeMap UpdateObjectsFunctor::call(SharedDsgInfo& dsg, const UpdateInfo& info) const {
  ScopedTimer spin_timer("backend/update_objects", info.timestamp_ns);
  std::unique_lock<std::mutex> lock(dsg.mutex);
  const auto& graph = *dsg.graph;
  if (!graph.hasLayer(DsgLayers::OBJECTS)) {
    return {};
  }

  const auto& layer = graph.getLayer(DsgLayers::OBJECTS);
  const auto mesh = graph.mesh();

  const size_t archived = makeNodeFinders(layer);

  size_t active = 0;
  std::map<NodeId, NodeId> nodes_to_merge;
  for (const auto& id_node_pair : layer.nodes()) {
    const NodeId node_id = id_node_pair.first;
    auto& attrs = id_node_pair.second->attributes<ObjectNodeAttributes>();
    if (!info.loop_closure_detected && !attrs.is_active && use_active_flag) {
      // skip the node if it is archived, there was no loop closure and we've okayed
      // skipping non-active nodes
      continue;
    }

    ++active;
    updateObject(mesh, node_id, attrs);

    if (!info.allow_node_merging) {
      continue;
    }

    // we only skip the first proposed object if the considered object is a potential
    // merge target. this happens if and only if:
    // - the object is an archived object and a loop closure is being processed
    // - we are not paying attention to the active flag
    const bool skip_first = !use_active_flag || !attrs.is_active;
    const auto to_merge = proposeObjectMerge(layer, attrs, skip_first);
    if (to_merge) {
      nodes_to_merge[node_id] = *to_merge;
    }
  }

  VLOG(2) << "[Hydra Backend] Object update: " << archived << " archived and " << active
          << " active";
  return nodes_to_merge;
}

void UpdateObjectsFunctor::mergeAttributes(const DynamicSceneGraph& graph,
                                           NodeId from,
                                           NodeId to) const {
  if (!allow_connection_merging) {
    return;
  }

  const auto from_node = graph.getNode(from);
  if (!from_node) {
    return;
  }

  const auto to_node = graph.getNode(to);
  if (!to_node) {
    return;
  }

  const auto& from_attrs = from_node->get().attributes<ObjectNodeAttributes>();
  auto& to_attrs = to_node->get().attributes<ObjectNodeAttributes>();
  // sort and merge
  std::vector<size_t> to_indices(to_attrs.mesh_connections.begin(),
                                 to_attrs.mesh_connections.end());
  std::vector<size_t> from_indices(from_attrs.mesh_connections.begin(),
                                   from_attrs.mesh_connections.end());
  to_attrs.mesh_connections.clear();
  std::sort(to_indices.begin(), to_indices.end());
  std::sort(from_indices.begin(), from_indices.end());
  std::set_union(to_indices.begin(),
                 to_indices.end(),
                 from_indices.begin(),
                 from_indices.end(),
                 std::back_inserter(to_attrs.mesh_connections));

  auto mesh = graph.mesh();
  updateObject(mesh, to, to_attrs);
  to_attrs.is_active = true;
}

UpdateFunctor::Hooks UpdatePlacesFunctor::hooks() const {
  auto my_hooks = UpdateFunctor::hooks();
  my_hooks.should_merge = [this](const NodeAttributes* lhs, const NodeAttributes* rhs) {
    return dispatchMergeCheck<PlaceNodeAttributes>(
        lhs,
        rhs,
        std::bind(&UpdatePlacesFunctor::shouldMerge,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));
  };
  my_hooks.node_update = [this](const UpdateInfo& info,
                                const spark_dsg::Mesh::Ptr,
                                NodeId node,
                                NodeAttributes* attrs) {
    if (!attrs || !info.places_values) {
      return;
    }

    updatePlace(*info.places_values, node, *attrs);
  };

  return my_hooks;
}

size_t UpdatePlacesFunctor::makeNodeFinder(const SceneGraphLayer& layer) const {
  node_finder.reset();  // clear previous node finder

  std::unordered_set<NodeId> layer_nodes;
  for (const auto& id_node_pair : layer.nodes()) {
    if (!id_node_pair.second->attributes().is_active) {
      layer_nodes.insert(id_node_pair.first);
    }
  }

  if (layer_nodes.empty()) {
    return layer_nodes.size();
  }

  node_finder = std::make_unique<NearestNodeFinder>(layer, layer_nodes);
  return layer_nodes.size();
}

void UpdatePlacesFunctor::updatePlace(const gtsam::Values& values,
                                      NodeId node,
                                      NodeAttributes& attrs) const {
  if (!values.exists(node)) {
    VLOG(5) << "[Hydra Backend] missing place " << NodeSymbol(node).getLabel()
            << " from places factors.";
    return;
  }

  attrs.position = values.at<gtsam::Pose3>(node).translation();
  // TODO(nathan) consider updating distance via parents + deformation graph
}

std::optional<NodeId> UpdatePlacesFunctor::proposePlaceMerge(
    const SceneGraphLayer& layer,
    NodeId from_node,
    const PlaceNodeAttributes& from_attrs,
    bool skip_first) const {
  std::list<NodeId> candidates;
  node_finder->find(from_attrs.position,
                    num_merges_to_consider,
                    skip_first,
                    [&candidates](NodeId place_id, size_t, double) {
                      candidates.push_back(place_id);
                    });

  for (const auto& id : candidates) {
    if (layer.hasEdge(from_node, id)) {
      continue;  // avoid merging siblings
    }

    const auto& to_attrs = layer.getNode(id)->get().attributes<PlaceNodeAttributes>();
    if (shouldMerge(from_attrs, to_attrs)) {
      return id;
    }
  }

  return std::nullopt;
}

bool UpdatePlacesFunctor::shouldMerge(const PlaceNodeAttributes& from_attrs,
                                      const PlaceNodeAttributes& to_attrs) const {
  if ((from_attrs.position - to_attrs.position).norm() > pos_threshold_m) {
    return false;
  }

  if (std::abs(from_attrs.distance - to_attrs.distance) > distance_tolerance_m) {
    return false;
  }

  return true;
}

void UpdatePlacesFunctor::filterMissing(DynamicSceneGraph& graph,
                                        const std::list<NodeId> missing_nodes) const {
  if (missing_nodes.empty()) {
    return;
  }

  VLOG(5) << "[Places Layer]: could not update "
          << displayNodeSymbolContainer(missing_nodes);

  for (const auto& node_id : missing_nodes) {
    if (!graph.hasNode(node_id)) {
      continue;
    }

    const Node& node = graph.getNode(node_id).value();
    if (!node.attributes().is_active && !node.hasSiblings()) {
      VLOG(2) << "[Places Layer]: removing node " << NodeSymbol(node_id).getLabel();
      graph.removeNode(node_id);
    }
  }
}

std::map<NodeId, NodeId> UpdatePlacesFunctor::call(SharedDsgInfo& dsg,
                                                   const UpdateInfo& info) const {
  ScopedTimer spin_timer("backend/update_places", info.timestamp_ns);
  std::unique_lock<std::mutex> lock(dsg.mutex);
  auto& graph = *dsg.graph;

  if (!graph.hasLayer(DsgLayers::PLACES) || !info.places_values) {
    return {};
  }

  if (info.places_values->size() == 0 && !info.allow_node_merging) {
    return {};
  }

  const auto& layer = graph.getLayer(DsgLayers::PLACES);
  const auto& places_values = *info.places_values;
  const size_t archived = makeNodeFinder(layer);

  std::list<NodeId> missing_nodes;
  std::map<NodeId, NodeId> nodes_to_merge;
  size_t num_active = 0;
  for (const auto& id_node_pair : layer.nodes()) {
    const NodeSymbol node_id(id_node_pair.first);
    if (node_id.category() != 'p') {
      continue;
    }

    auto& attrs = id_node_pair.second->attributes<PlaceNodeAttributes>();
    if (!attrs.is_active && !info.loop_closure_detected) {
      continue;
    }

    ++num_active;

    if (!places_values.exists(node_id)) {
      // this happens for the GT version
      missing_nodes.push_back(node_id);
    } else {
      updatePlace(places_values, node_id, attrs);
    }

    if (!info.allow_node_merging || !node_finder) {
      // avoid looking for merges when disallowed or there are no archived places
      continue;
    }

    // we only skip the first proposed place if the considered place is not active
    const auto to_merge = proposePlaceMerge(layer, node_id, attrs, !attrs.is_active);
    if (to_merge) {
      nodes_to_merge[node_id] = *to_merge;
    }
  }

  VLOG(2) << "[Hydra Backend] Places update: " << archived << " archived and "
          << num_active << " active";
  filterMissing(graph, missing_nodes);
  return nodes_to_merge;
}

UpdateRoomsFunctor::UpdateRoomsFunctor(const RoomFinderConfig& config)
    : room_finder(new RoomFinder(config)) {}

UpdateRoomsFunctor::~UpdateRoomsFunctor() {}

void UpdateRoomsFunctor::rewriteRooms(const SceneGraphLayer* new_rooms,
                                      DynamicSceneGraph& graph) const {
  std::vector<NodeId> to_remove;
  const auto& prev_rooms = graph.getLayer(DsgLayers::ROOMS);
  for (const auto& id_node_pair : prev_rooms.nodes()) {
    to_remove.push_back(id_node_pair.first);
  }

  for (const auto node_id : to_remove) {
    graph.removeNode(node_id);
  }

  if (!new_rooms) {
    return;
  }

  for (auto&& [id, node] : new_rooms->nodes()) {
    graph.emplaceNode(DsgLayers::ROOMS, id, node->attributes().clone());
  }

  for (const auto& id_edge_pair : new_rooms->edges()) {
    const auto& edge = id_edge_pair.second;
    graph.insertEdge(edge.source, edge.target, edge.info->clone());
  }
}

std::map<NodeId, NodeId> UpdateRoomsFunctor::call(SharedDsgInfo& dsg,
                                                  const UpdateInfo& info) const {
  if (!room_finder) {
    return {};
  }

  SceneGraphLayer::Ptr places_clone;
  {  // start dsg critical section
    std::unique_lock<std::mutex> lock(dsg.mutex);
    ScopedTimer timer("backend/clone_places", info.timestamp_ns, true, 1, false);
    places_clone = dsg.graph->getLayer(DsgLayers::PLACES).clone([](const auto& node) {
      return NodeSymbol(node.id).category() == 'p';
    });
  }  // end dsg critical section

  ScopedTimer timer("backend/room_detection", info.timestamp_ns, true, 1, false);
  // TODO(nathan) pass in timestamp?
  auto rooms = room_finder->findRooms(*places_clone);

  {  // start dsg critical section
    std::unique_lock<std::mutex> lock(dsg.mutex);
    rewriteRooms(rooms.get(), *dsg.graph);
    room_finder->addRoomPlaceEdges(*dsg.graph);
  }  // end dsg critical section

  return {};
}

UpdateBuildingsFunctor::UpdateBuildingsFunctor(const NodeColor& color,
                                               SemanticLabel label)
    : building_color(color), building_semantic_label(label) {}

std::map<NodeId, NodeId> UpdateBuildingsFunctor::call(SharedDsgInfo& dsg,
                                                      const UpdateInfo& info) const {
  ScopedTimer timer("backend/building_detection", info.timestamp_ns, true, 1, false);

  const NodeSymbol building_id('B', 0);
  std::unique_lock<std::mutex> lock(dsg.mutex);
  const auto& rooms = dsg.graph->getLayer(DsgLayers::ROOMS);

  if (!rooms.numNodes()) {
    if (dsg.graph->hasNode(building_id)) {
      dsg.graph->removeNode(building_id);
    }

    return {};
  }

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto& id_node_pair : rooms.nodes()) {
    centroid += id_node_pair.second->attributes().position;
  }
  centroid /= rooms.numNodes();

  if (!dsg.graph->hasNode(building_id)) {
    SemanticNodeAttributes::Ptr attrs(new SemanticNodeAttributes());
    attrs->position = centroid;
    attrs->color = building_color;
    attrs->semantic_label = building_semantic_label;
    attrs->name = building_id.getLabel();
    dsg.graph->emplaceNode(DsgLayers::BUILDINGS, building_id, std::move(attrs));
  } else {
    dsg.graph->getNode(building_id)->get().attributes().position = centroid;
  }

  for (const auto& id_node_pair : rooms.nodes()) {
    dsg.graph->insertEdge(building_id, id_node_pair.first);
  }

  return {};
}

std::map<NodeId, NodeId> updateAgents(SharedDsgInfo& dsg, const UpdateInfo& info) {
  if (!info.complete_agent_values || info.complete_agent_values->size() == 0) {
    return {};
  }

  ScopedTimer timer("backend/agent_update", info.timestamp_ns, true, 1, false);
  std::unique_lock<std::mutex> lock(dsg.mutex);
  auto& graph = *dsg.graph;

  const LayerId desired_layer = DsgLayers::AGENTS;

  for (const auto& prefix_layer_pair : graph.dynamicLayersOfType(desired_layer)) {
    std::set<NodeId> missing_nodes;

    for (const auto& node : prefix_layer_pair.second->nodes()) {
      auto& attrs = node->attributes<AgentNodeAttributes>();
      if (!info.complete_agent_values->exists(attrs.external_key)) {
        missing_nodes.insert(node->id);
        continue;
      }

      gtsam::Pose3 agent_pose =
          info.complete_agent_values->at<gtsam::Pose3>(attrs.external_key);
      attrs.position = agent_pose.translation();
      attrs.world_R_body = Eigen::Quaterniond(agent_pose.rotation().matrix());
    }

    if (!missing_nodes.empty()) {
      LOG(WARNING) << "Layer " << DsgLayers::AGENTS << "(" << prefix_layer_pair.first
                   << "): could not update "
                   << displayNodeSymbolContainer(missing_nodes);
    }
  }

  return {};
}

Update2dPlacesFunctor::Update2dPlacesFunctor(const Places2dConfig& config)
    : config_(config){};

Update2dPlacesFunctor::~Update2dPlacesFunctor(){};

size_t Update2dPlacesFunctor::makeNodeFinders(const SceneGraphLayer& layer) const {
  std::map<SemanticLabel, std::unordered_set<NodeId>> label_node_map;
  size_t archived = 0;
  for (const auto& id_node_pair : layer.nodes()) {
    auto& attrs = id_node_pair.second->attributes<SemanticNodeAttributes>();

    if (use_active_flag && attrs.is_active) {
      continue;
    }

    ++archived;
    auto iter = label_node_map.find(attrs.semantic_label);
    if (iter == label_node_map.end()) {
      iter = label_node_map.insert({attrs.semantic_label, {}}).first;
    }

    iter->second.insert(id_node_pair.first);
  }

  // creating nodefinders
  node_finders.clear();
  for (const auto& label_ids_pair : label_node_map) {
    node_finders.emplace(
        label_ids_pair.first,
        std::make_unique<NearestNodeFinder>(layer, label_ids_pair.second));
  }

  return archived;
}

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
    const auto& point = mesh->pos(midx.idx);
    if (std::isnan(point.x()) || std::isnan(point.y()) || std::isnan(point.z())) {
      VLOG(4) << "found nan at index: " << midx.idx << " with point: [" << point.x()
              << ", " << point.y() << ", " << point.z() << "]";
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
    const auto& point = mesh->pos(attrs.pcl_boundary_connections.at(i).idx);
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

std::optional<NodeId> Update2dPlacesFunctor::proposeMerge(
    const SceneGraphLayer& layer,
    const NodeId from_node,
    const size_t num_archived,
    const Place2dNodeAttributes& from_attrs,
    bool skip_first) const {
  const auto iter = node_finders.find(from_attrs.semantic_label);
  if (iter == node_finders.end()) {
    return std::nullopt;
  }

  std::list<NodeId> candidates;
  (*iter).second->find(from_attrs.position,
                       num_merges_to_consider,
                       skip_first,
                       [&candidates](NodeId object_id, size_t, double) {
                         candidates.push_back(object_id);
                       });

  for (const auto& id : candidates) {
    if (layer.hasEdge(from_node, id)) {
      continue;  // avoid merging siblings
    }
    const auto& to_attrs = layer.getNode(id)->get().attributes<Place2dNodeAttributes>();
    if (shouldMerge(from_attrs, to_attrs)) {
      return id;
    }
  }

  return std::nullopt;
}

void Update2dPlacesFunctor::mergeAttributes(const DynamicSceneGraph& graph,
                                            NodeId from,
                                            NodeId to) const {
  const auto from_node = graph.getNode(from);
  if (!from_node) {
    return;
  }

  const auto to_node = graph.getNode(to);
  if (!to_node) {
    return;
  }

  const auto& from_attrs = from_node->get().attributes<Place2dNodeAttributes>();
  auto& to_attrs = to_node->get().attributes<Place2dNodeAttributes>();
  // sort and merge
  std::vector<MeshIndex> to_indices(to_attrs.pcl_mesh_connections.begin(),
                                    to_attrs.pcl_mesh_connections.end());
  std::vector<MeshIndex> from_indices(from_attrs.pcl_mesh_connections.begin(),
                                      from_attrs.pcl_mesh_connections.end());
  to_attrs.pcl_mesh_connections.clear();
  std::sort(to_indices.begin(), to_indices.end());
  std::sort(from_indices.begin(), from_indices.end());
  std::set_union(to_indices.begin(),
                 to_indices.end(),
                 from_indices.begin(),
                 from_indices.end(),
                 std::back_inserter(to_attrs.pcl_mesh_connections));
  to_attrs.need_finish_merge = true;
  to_attrs.has_active_mesh_indices =
      from_attrs.has_active_mesh_indices || to_attrs.has_active_mesh_indices;
  to_attrs.need_cleanup_splitting = true;
}

void getPlace2dAndNeighors(const SceneGraphLayer& places_layer,
                           std::vector<std::pair<NodeId, Place2d>>& place_2ds,
                           std::map<NodeId, std::set<NodeId>>& node_neighbors) {
  for (auto& id_node_pair : places_layer.nodes()) {
    auto& attrs = id_node_pair.second->attributes<Place2dNodeAttributes>();
    if (attrs.need_finish_merge) {
      Place2d p;
      p.indices.insert(p.indices.end(),
                       attrs.pcl_mesh_connections.begin(),
                       attrs.pcl_mesh_connections.end());
      place_2ds.push_back(std::pair(id_node_pair.first, p));
      node_neighbors.insert({id_node_pair.first, id_node_pair.second->siblings()});
    }
  }
}

void getNecessaryUpdates(
    const spark_dsg::Mesh& mesh,
    const int min_points,
    const double min_size,
    const double connection_ellipse_scale_factor,
    std::vector<std::pair<NodeId, Place2d>>& place_2ds,
    std::vector<std::pair<NodeId, Place2d>>& nodes_to_update,
    std::vector<std::pair<NodeId, std::vector<Place2d>>>& nodes_to_add) {
  for (auto& id_place_pair : place_2ds) {
    addRectInfo(mesh.points, connection_ellipse_scale_factor, id_place_pair.second);

    if (id_place_pair.second.indices.size() > min_points &&
        id_place_pair.second.cut_plane.norm() > min_size) {
      std::vector<Place2d> split_places =
          decomposePlace(mesh.points,
                         id_place_pair.second,
                         min_size,
                         min_points,
                         connection_ellipse_scale_factor);
      for (Place2d& p : split_places) {
        addBoundaryInfo(mesh.points, p);
      }
      nodes_to_add.push_back(std::pair(id_place_pair.first, split_places));
    } else {
      addBoundaryInfo(mesh.points, id_place_pair.second);
      size_t min_ix = SIZE_MAX;
      size_t max_ix = 0;
      for (auto midx : id_place_pair.second.indices) {
        min_ix = std::min(min_ix, midx.idx);
        max_ix = std::max(max_ix, midx.idx);
      }
      id_place_pair.second.min_mesh_index = min_ix;
      id_place_pair.second.max_mesh_index = max_ix;
      nodes_to_update.push_back(id_place_pair);
    }
  }
}

std::map<std::tuple<size_t, size_t, size_t, size_t>, double> buildEdgeMap(
    const std::vector<std::pair<NodeId, std::vector<Place2d>>>& nodes_to_add,
    double place_overlap_threshold,
    double place_neighbor_z_diff) {
  std::map<std::tuple<size_t, size_t, size_t, size_t>, double> edge_map;
  // compute which pairs of new nodes will need to have an edge added
  // Currently checks all split merged nodes. Maybe just check neighbors-of-neighbors?
  for (size_t og_ix = 0; og_ix < nodes_to_add.size(); ++og_ix) {
    for (size_t split_ix = 0; split_ix < nodes_to_add.at(og_ix).second.size();
         ++split_ix) {
      Place2d p1 = nodes_to_add.at(og_ix).second.at(split_ix);
      for (size_t og_jx = 0; og_jx <= og_ix; ++og_jx) {
        for (size_t split_jx = 0; split_jx < nodes_to_add.at(og_jx).second.size();
             ++split_jx) {
          Place2d p2 = nodes_to_add.at(og_jx).second.at(split_jx);
          double weight;
          bool connected = shouldAddPlaceConnection(
              p1, p2, place_overlap_threshold, place_neighbor_z_diff, weight);
          edge_map.insert({std::make_tuple(og_ix, split_ix, og_jx, split_jx),
                           connected ? weight : 0});
        }
      }
    }
  }
  return edge_map;
}

void updateExistingNodes(const std::vector<std::pair<NodeId, Place2d>>& nodes_to_update,
                         DynamicSceneGraph& graph) {
  for (auto& id_place_pair : nodes_to_update) {
    Place2dNodeAttributes& attrs =
        graph.getNode(id_place_pair.first)->get().attributes<Place2dNodeAttributes>();
    Place2d place = id_place_pair.second;
    pcl::PointXYZ centroid;
    place.centroid.get(centroid);
    attrs.position << centroid.x, centroid.y, centroid.z;

    attrs.boundary = place.boundary;
    attrs.pcl_boundary_connections.clear();
    attrs.pcl_boundary_connections.insert(attrs.pcl_boundary_connections.begin(),
                                          place.boundary_indices.begin(),
                                          place.boundary_indices.end());
    attrs.ellipse_matrix_compress = place.ellipse_matrix_compress;
    attrs.ellipse_matrix_expand = place.ellipse_matrix_expand;
    attrs.ellipse_centroid(0) = place.ellipse_centroid(0);
    attrs.ellipse_centroid(1) = place.ellipse_centroid(1);
    attrs.ellipse_centroid(2) = centroid.z;
    attrs.pcl_min_index = place.min_mesh_index;
    attrs.pcl_max_index = place.max_mesh_index;

    attrs.pcl_mesh_connections.clear();
    attrs.pcl_mesh_connections.insert(
        attrs.pcl_mesh_connections.begin(), place.indices.begin(), place.indices.end());

    attrs.need_finish_merge = false;
  }
}

NodeSymbol insertNewNodes(
    const std::vector<std::pair<NodeId, std::vector<Place2d>>>& nodes_to_add,
    const double place_overlap_threshold,
    const double place_max_neighbor_z_diff,
    NodeSymbol next_node_symbol,
    DynamicSceneGraph& graph,
    std::map<std::tuple<size_t, size_t>, NodeId>& new_id_map) {
  // insert new nodes that needed to be split
  int og_ix = 0;
  for (auto& id_places_pair : nodes_to_add) {
    const auto node = graph.getNode(id_places_pair.first);
    auto& attrs_og = node->get().attributes<Place2dNodeAttributes>();

    int split_ix = 0;
    for (Place2d place : id_places_pair.second) {
      NodeSymbol node_id_for_place = next_node_symbol++;
      Place2dNodeAttributes::Ptr attrs = std::make_unique<Place2dNodeAttributes>();
      pcl::PointXYZ centroid;
      place.centroid.get(centroid);
      attrs->position << centroid.x, centroid.y, centroid.z;
      attrs->is_active = false;

      attrs->semantic_label = attrs_og.semantic_label;
      attrs->name = NodeSymbol(node_id_for_place).getLabel();
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

      attrs->pcl_mesh_connections.insert(attrs->pcl_mesh_connections.begin(),
                                         place.indices.begin(),
                                         place.indices.end());
      attrs->color = attrs_og.color;

      attrs->has_active_mesh_indices = attrs_og.has_active_mesh_indices;
      attrs->need_cleanup_splitting = true;
      attrs->need_finish_merge = false;

      graph.emplaceNode(DsgLayers::MESH_PLACES, node_id_for_place, std::move(attrs));
      new_id_map.insert({std::make_tuple(og_ix, split_ix), node_id_for_place});

      Place2dNodeAttributes& attrs_added =
          graph.getNode(node_id_for_place)->get().attributes<Place2dNodeAttributes>();
      // Check for connections between this place and the original place's siblings
      std::vector<std::pair<NodeId, NodeId>> edges_to_add;
      for (NodeId neighbor : node->get().siblings()) {
        EdgeAttributes ea;
        Place2dNodeAttributes attrs_neighbor =
            graph.getNode(neighbor)->get().attributes<Place2dNodeAttributes>();

        if (!attrs_neighbor.need_finish_merge &&
            shouldAddPlaceConnection(attrs_added,
                                     attrs_neighbor,
                                     place_overlap_threshold,
                                     place_max_neighbor_z_diff,
                                     ea)) {
          edges_to_add.push_back({neighbor, node_id_for_place});
        }
      }
      for (std::pair<NodeId, NodeId> edge : edges_to_add) {
        graph.insertEdge(edge.first, edge.second);  // TODO add edge attributes
      }

      ++split_ix;
    }
    ++og_ix;
    graph.removeNode(id_places_pair.first);
  }
  return next_node_symbol;
}

void addNewNodeEdges(
    const std::vector<std::pair<NodeId, std::vector<Place2d>>> nodes_to_add,
    const std::map<std::tuple<size_t, size_t, size_t, size_t>, double> edge_map,
    const std::map<std::tuple<size_t, size_t>, NodeId> new_id_map,
    DynamicSceneGraph& graph) {
  // It looks like this loop would take a long time, but all of these sizes should be
  // extremely small (e.g. 2ish)
  for (size_t og_ix = 0; og_ix < nodes_to_add.size(); ++og_ix) {
    for (size_t split_ix = 0; split_ix < nodes_to_add.at(og_ix).second.size();
         ++split_ix) {
      Place2d p1 = nodes_to_add.at(og_ix).second.at(split_ix);
      for (size_t og_jx = 0; og_jx <= og_ix; ++og_jx) {
        for (size_t split_jx = 0; split_jx < nodes_to_add.at(og_jx).second.size();
             ++split_jx) {
          double weight =
              edge_map.at(std::make_tuple(og_ix, split_ix, og_jx, split_jx));
          if (weight > 0) {
            NodeId n1 = new_id_map.at(std::make_tuple(og_ix, split_ix));
            NodeId n2 = new_id_map.at(std::make_tuple(og_jx, split_jx));
            EdgeAttributes ea;
            ea.weight = weight;
            ea.weighted = true;
            graph.insertEdge(n1, n2, ea.clone());
          }
        }
      }
    }
  }
}

void reallocateMeshPoints(const std::vector<Place2d::PointT>& points,
                          Place2dNodeAttributes& attrs1,
                          Place2dNodeAttributes& attrs2) {
  Eigen::Vector2d delta = attrs2.position.head(2) - attrs1.position.head(2);
  Eigen::Vector2d d = attrs1.position.head(2) + delta / 2;

  std::vector<MeshIndex> p1_new_indices;
  std::vector<MeshIndex> p2_new_indices;

  for (auto midx : attrs1.pcl_mesh_connections) {
    // pcl::PointXYZRGBA pclp = points.at(midx.idx);
    // Eigen::Vector2d p(pclp.x, pclp.y);
    Eigen::Vector2d p = points.at(midx.idx).head(2).cast<double>();
    if ((p - d).dot(delta) > 0) {
      p2_new_indices.push_back(midx);
    } else {
      p1_new_indices.push_back(midx);
    }
  }
  for (auto midx : attrs2.pcl_mesh_connections) {
    // pcl::PointXYZRGBA pclp = points.at(midx.idx);
    // Eigen::Vector2d p(pclp.x, pclp.y);
    Eigen::Vector2d p = points.at(midx.idx).head(2).cast<double>();
    if ((p - d).dot(delta) > 0) {
      p2_new_indices.push_back(midx);
    } else {
      p1_new_indices.push_back(midx);
    }
  }

  if (p1_new_indices.size() == 0 || p2_new_indices.size() == 0) {
    LOG(ERROR) << "Reallocating mesh points would make empty place. Skippings.";
    return;
  }
  attrs1.pcl_mesh_connections = p1_new_indices;
  attrs2.pcl_mesh_connections = p2_new_indices;

  // Say there are active mesh indices if either involved node has them.
  // In theory we could actually check if any of the reallocated vertices changes a
  // place's activeness for a small speed improvement, but not sure how much it matters
  attrs1.has_active_mesh_indices =
      attrs1.has_active_mesh_indices || attrs2.has_active_mesh_indices;
  attrs2.has_active_mesh_indices =
      attrs1.has_active_mesh_indices || attrs2.has_active_mesh_indices;
}

NodeId cleanupPlaces2d(const Places2dConfig& config,
                       SharedDsgInfo* dsg,
                       NodeId next_node_id_) {
  if (!dsg) {
    return next_node_id_;
  }

  std::map<NodeId, std::set<NodeId>> node_neighbors;
  std::vector<std::pair<NodeId, Place2d>> place_2ds;

  // Get/copy info for places that need cleanup
  std::unique_lock<std::mutex> lock(dsg->mutex);
  const SceneGraphLayer& places_layer = dsg->graph->getLayer(DsgLayers::PLACES);
  getPlace2dAndNeighors(places_layer, place_2ds, node_neighbors);
  lock.unlock();

  // Decide which places need to be split and which just need to be updated
  auto& graph = *dsg->graph;
  spark_dsg::Mesh::Ptr mesh = graph.mesh();
  std::vector<std::pair<NodeId, Place2d>>
      nodes_to_update;  // existing node id for each place
  std::vector<std::pair<NodeId, std::vector<Place2d>>>
      nodes_to_add;  // node id that new nodes split from (necessary to copy other
                     // place info)
  getNecessaryUpdates(*mesh,
                      config.min_points,
                      config.min_size,
                      config.connection_ellipse_scale_factor,
                      place_2ds,
                      nodes_to_update,
                      nodes_to_add);
  std::map<std::tuple<size_t, size_t, size_t, size_t>, double> edge_map = buildEdgeMap(
      nodes_to_add, config.connection_overlap_threshold, config.connection_max_delta_z);

  lock.lock();
  // Update attributes for place nodes that did not need to split after merge
  updateExistingNodes(nodes_to_update, graph);

  // Insert new nodes that are formed by splitting existing nodes (and delete
  // previous node)
  std::map<std::tuple<size_t, size_t>, NodeId> new_id_map;
  next_node_id_ = insertNewNodes(nodes_to_add,
                                 config.connection_overlap_threshold,
                                 config.connection_max_delta_z,
                                 next_node_id_,
                                 graph,
                                 new_id_map);

  // Add edges between new nodes
  addNewNodeEdges(nodes_to_add, edge_map, new_id_map, graph);

  std::unordered_map<NodeId, bool> checked_nodes;
  // Clean up places that are far enough away from the active window
  // Far enough means that none of a node's neighbors or the node itself have
  // active mesh vertices
  for (auto& id_node_pair : places_layer.nodes()) {
    auto& attrs = id_node_pair.second->attributes<Place2dNodeAttributes>();
    if (attrs.need_cleanup_splitting) {
      bool has_active_neighbor = false;
      for (NodeId nid : id_node_pair.second->siblings()) {
        Place2dNodeAttributes& neighbor_attrs =
            graph.getNode(nid)->get().attributes<Place2dNodeAttributes>();
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
        Place2dNodeAttributes& neighbor_attrs =
            graph.getNode(nid)->get().attributes<Place2dNodeAttributes>();

        auto search = checked_nodes.find(nid);
        if (search == checked_nodes.end()) {
          checked_nodes.insert({nid, false});
        }
        if (attrs.semantic_label == neighbor_attrs.semantic_label) {
          reallocateMeshPoints(mesh->points, attrs, neighbor_attrs);
        }
      }
    }
  }

  for (auto& id_finalize : checked_nodes) {
    auto& attrs =
        graph.getNode(id_finalize.first)->get().attributes<Place2dNodeAttributes>();

    if (attrs.pcl_mesh_connections.size() == 0) {
      LOG(ERROR) << "Reallocating mesh points would make empty place. Skipping.";
      continue;
    }

    addRectInfo(mesh->points, config.connection_ellipse_scale_factor, attrs);
    addBoundaryInfo(mesh->points, attrs);
  }

  std::vector<std::pair<NodeId, NodeId>> edges_to_remove;
  std::map<NodeId, NodeId> extra_edges_to_check;
  for (auto& id_finalize : checked_nodes) {
    NodeId id = id_finalize.first;
    bool finalize = id_finalize.second;
    Place2dNodeAttributes& attrs1 =
        graph.getNode(id)->get().attributes<Place2dNodeAttributes>();

    for (auto& neighbor_id_final : checked_nodes) {
      NodeId nid = neighbor_id_final.first;
      auto neighbor_node = graph.getNode(nid);
      Place2dNodeAttributes& attrs2 =
          neighbor_node->get().attributes<Place2dNodeAttributes>();
      EdgeAttributes ea;
      if (!shouldAddPlaceConnection(attrs1,
                                    attrs2,
                                    config.connection_overlap_threshold,
                                    config.connection_max_delta_z,
                                    ea)) {
        graph.removeEdge(id, nid);
      } else {
        graph.insertEdge(id, nid, ea.clone());
      }
    }

    if (finalize) {
      attrs1.need_cleanup_splitting = false;
    }
  }
  return next_node_id_;
}

UpdateFunctor::Hooks Update2dPlacesFunctor::hooks() const {
  auto my_hooks = UpdateFunctor::hooks();
  my_hooks.merge = [this](const DynamicSceneGraph& graph, NodeId from, NodeId to) {
    mergeAttributes(graph, from, to);  // updates attributes in `to`
  };
  my_hooks.should_merge = [this](const NodeAttributes* lhs, const NodeAttributes* rhs) {
    return dispatchMergeCheck<Place2dNodeAttributes>(
        lhs,
        rhs,
        std::bind(&Update2dPlacesFunctor::shouldMerge,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));
  };
  my_hooks.node_update = [this](const UpdateInfo&,
                                const spark_dsg::Mesh::Ptr mesh,
                                NodeId node,
                                NodeAttributes* attrs) {
    auto oattrs = dynamic_cast<Place2dNodeAttributes*>(attrs);
    if (!oattrs || !mesh) {
      return;
    }

    updateNode(mesh, node, *oattrs);
  };

  my_hooks.cleanup = [this](const UpdateInfo&, SharedDsgInfo* dsg) {
    next_node_id_ = cleanupPlaces2d(config_, dsg, next_node_id_);
  };

  return my_hooks;
}

MergeMap Update2dPlacesFunctor::call(SharedDsgInfo& dsg, const UpdateInfo& info) const {
  ScopedTimer spin_timer("backend/update_2d_places", info.timestamp_ns);
  std::unique_lock<std::mutex> lock(dsg.mutex);
  const auto& graph = *dsg.graph;
  if (!graph.hasLayer(layer_id_)) {
    return {};
  }

  const auto& layer = graph.getLayer(layer_id_);
  const size_t archived = makeNodeFinders(layer);
  spark_dsg::Mesh::Ptr mesh = graph.mesh();

  size_t active = 0;
  std::map<NodeId, NodeId> nodes_to_merge;
  for (auto&& [node_id, node] : layer.nodes()) {
    auto& attrs = node->attributes<Place2dNodeAttributes>();
    if (!info.loop_closure_detected && !attrs.is_active && use_active_flag) {
      // skip the node if it is archived, there was no loop closure and we've okayed
      // skipping non-active nodes
      continue;
    }

    ++active;
    updateNode(mesh, node_id, attrs);
    if (!info.allow_node_merging || !config_.allow_places_merge) {
      continue;
    }

    // we only skip the first proposed object if the considered object is a
    // potential merge target. this happens if and only if:
    // - the object is an archived object and a loop closure is being processed
    // - we are not paying attention to the active flag
    const bool skip_first = !use_active_flag || !attrs.is_active;
    const auto to_merge =
        proposeMerge(layer, node_id, info.num_archived_vertices, attrs, skip_first);
    if (to_merge) {
      nodes_to_merge[node_id] = *to_merge;
    }
  }

  VLOG(5) << "[Hydra Backend] 2D Place update: " << archived << " archived and "
          << active << " active";
  return nodes_to_merge;
}
}  // namespace dsg_updates
}  // namespace hydra
