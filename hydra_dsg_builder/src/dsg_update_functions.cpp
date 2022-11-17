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
#include "hydra_dsg_builder/dsg_update_functions.h"

#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>
#include <hydra_utils/timing_utilities.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <spark_dsg/bounding_box_extraction.h>

namespace hydra {
namespace dsg_updates {

using incremental::SharedDsgInfo;
using timing::ScopedTimer;
using topology::NearestNodeFinder;
using Node = SceneGraphNode;
using Centroid = pcl::CentroidPoint<pcl::PointXYZ>;
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

void UpdateObjectsFunctor::makeNodeFinders(const SceneGraphLayer& layer) const {
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

  VLOG(5) << "[Hydra Backend] Using " << archived << " archived objects for update";
}

void UpdateObjectsFunctor::updateObject(const MeshVertices::Ptr& mesh,
                                        const std::vector<size_t>& connections,
                                        NodeId node,
                                        ObjectNodeAttributes& attrs) const {
  if (connections.empty()) {
    VLOG(2) << "Found empty object node " << NodeSymbol(node).getLabel();
    return;
  }

  pcl::IndicesPtr indices;
  indices.reset(new std::vector<int>(connections.begin(), connections.end()));

  attrs.bounding_box = bounding_box::extract(mesh, attrs.bounding_box.type, indices);

  Centroid centroid;
  for (const auto& idx : *indices) {
    const auto& point = mesh->at(idx);
    if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
      VLOG(4) << "found nan at index: " << idx << " with point: [" << point.x << ", "
              << point.y << ", " << point.z << "]";
      continue;
    }

    centroid.add(pcl::PointXYZ(point.x, point.y, point.z));
  }

  if (!centroid.getSize()) {
    VLOG(2) << "Invalid centroid for object " << NodeSymbol(node).getLabel();
    return;
  }

  pcl::PointXYZ pcl_pos;
  centroid.get(pcl_pos);
  attrs.position << pcl_pos.x, pcl_pos.y, pcl_pos.z;
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

std::map<NodeId, NodeId> UpdateObjectsFunctor::call(SharedDsgInfo& dsg,
                                                    const UpdateInfo& info) const {
  ScopedTimer spin_timer("backend/update_objects", info.timestamp_ns);
  std::unique_lock<std::mutex> lock(dsg.mutex);
  const auto& graph = *dsg.graph;
  if (!graph.hasLayer(DsgLayers::OBJECTS)) {
    return {};
  }

  const auto& layer = graph.getLayer(DsgLayers::OBJECTS);
  MeshVertices::Ptr mesh = graph.getMeshVertices();

  makeNodeFinders(layer);

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
    std::vector<size_t> connections = graph.getMeshConnectionIndices(node_id);
    updateObject(mesh, connections, node_id, attrs);

    if (!info.allow_node_merging) {
      continue;
    }

    // we only skip the first proposed object if the considered object is a potential
    // merge target. this happens if and onlf if:
    // - the object is an archived object and a loop closure is being processed
    // - we are not paying attention to the active flag
    const bool skip_first = !use_active_flag || !attrs.is_active;
    const auto to_merge = proposeObjectMerge(layer, attrs, skip_first);
    if (to_merge) {
      nodes_to_merge[node_id] = *to_merge;
    }
  }

  VLOG(5) << "[Hydra Backend] Using " << active << " active objects for update";

  return nodes_to_merge;
}

void UpdatePlacesFunctor::makeNodeFinder(const SceneGraphLayer& layer) const {
  std::unordered_set<NodeId> layer_nodes;
  for (const auto& id_node_pair : layer.nodes()) {
    if (!id_node_pair.second->attributes().is_active) {
      layer_nodes.insert(id_node_pair.first);
    }
  }

  VLOG(5) << "[Hydra Backend] Using " << layer_nodes.size() << " archived places for update";
  if (layer_nodes.empty()) {
    return;
  }

  node_finder = std::make_unique<NearestNodeFinder>(layer, layer_nodes);
}

void UpdatePlacesFunctor::updatePlace(const gtsam::Values& places_values,
                                      NodeId node_id,
                                      PlaceNodeAttributes& attrs) const {
  attrs.position = places_values.at<gtsam::Pose3>(node_id).translation();
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

  VLOG(6) << "[Places Layer]: could not update "
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

  makeNodeFinder(layer);

  std::list<NodeId> missing_nodes;
  std::map<NodeId, NodeId> nodes_to_merge;
  size_t num_active = 0;
  for (const auto& id_node_pair : layer.nodes()) {
    const auto node_id = id_node_pair.first;
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
  VLOG(5) << "[Hydra Backend] Using " << num_active << " active places during update";

  filterMissing(graph, missing_nodes);
  return nodes_to_merge;
}

UpdateRoomsFunctor::UpdateRoomsFunctor(const incremental::RoomFinder::Config& config)
    : room_finder(new incremental::RoomFinder(config)) {}

std::map<NodeId, NodeId> UpdateRoomsFunctor::call(SharedDsgInfo& dsg,
                                                  const UpdateInfo& info) const {
  if (!room_finder) {
    return {};
  }

  ScopedTimer timer("backend/room_detection", info.timestamp_ns, true, 1, false);

  // TODO(nathan) this is kinda clunky
  std::unordered_set<NodeId> place_ids;
  {  // start critical section
    std::unique_lock<std::mutex> lock(dsg.mutex);
    const auto& places = dsg.graph->getLayer(DsgLayers::PLACES);
    for (const auto& id_node_pair : places.nodes()) {
      place_ids.insert(id_node_pair.first);
    }
  }  // end critical section

  VLOG(3) << "Detecting rooms for " << place_ids.size() << " nodes";
  room_finder->findRooms(dsg, place_ids);

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

}  // namespace dsg_updates
}  // namespace hydra
