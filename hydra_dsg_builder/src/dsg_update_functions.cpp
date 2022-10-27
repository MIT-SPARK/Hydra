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
#include <hydra_topology/nearest_neighbor_utilities.h>
#include <hydra_utils/timing_utilities.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <spark_dsg/bounding_box_extraction.h>

namespace hydra {
namespace dsg_updates {

using incremental::SharedDsgInfo;
using timing::ScopedTimer;
using topology::NearestNodeFinder;
using MeshVertices = DynamicSceneGraph::MeshVertices;
using Node = SceneGraphNode;
using Centroid = pcl::CentroidPoint<pcl::PointXYZ>;
using NodeColor = SemanticNodeAttributes::ColorVector;
using SemanticLabel = SemanticNodeAttributes::Label;

void filterObject(DynamicSceneGraph& graph,
                  std::list<NodeId>& valid_candidates,
                  const NodeId& base_node,
                  const NodeId& candidate,
                  const std::unordered_set<NodeId>& semantic_set) {
  // there must not be an edge between nodes
  // candidate must be in semantic map
  if (!graph.hasEdge(base_node, candidate) && semantic_set.count(candidate)) {
    valid_candidates.push_back(candidate);
  }
  return;
}

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

std::map<NodeId, NodeId> UpdateObjectsFunctor::call(SharedDsgInfo& dsg,
                                                    const UpdateInfo& info) const {
  ScopedTimer spin_timer("backend/update_objects", info.timestamp_ns);
  std::unique_lock<std::mutex> lock(dsg.mutex);
  auto& graph = *dsg.graph;
  if (!graph.hasLayer(DsgLayers::OBJECTS)) {
    return {};
  }

  const auto& layer = graph.getLayer(DsgLayers::OBJECTS);
  MeshVertices::Ptr mesh = graph.getMeshVertices();

  std::map<NodeId, NodeId> nodes_to_merge;
  std::list<NodeId> valid_candidates;
  std::map<SemanticLabel, std::unordered_set<NodeId>> semantic_nodes_map;
  std::map<SemanticLabel, std::unique_ptr<NearestNodeFinder>> labeled_node_finders;

  // creating semantic nodes map
  if (info.loop_closure_detected || archived_object_ids.empty()) {
    for (const auto& id_node_pair : layer.nodes()) {
      auto& attrs = id_node_pair.second->attributes<ObjectNodeAttributes>();
      if (!semantic_nodes_map.count(attrs.semantic_label)) {
        semantic_nodes_map[attrs.semantic_label] = std::unordered_set<NodeId>();
      }
      semantic_nodes_map[attrs.semantic_label].insert(id_node_pair.first);
    }
  } else {
    for (const auto& id_node_pair : layer.nodes()) {
      auto& attrs = id_node_pair.second->attributes<ObjectNodeAttributes>();
      if (!semantic_nodes_map.count(attrs.semantic_label)) {
        semantic_nodes_map[attrs.semantic_label] = std::unordered_set<NodeId>();
      }
      if (archived_object_ids.count(id_node_pair.first)) {
        semantic_nodes_map[attrs.semantic_label].insert(id_node_pair.first);
      }
    }
  }

  // creating nodefinders
  for (const auto& label_nodes_pair : semantic_nodes_map) {
    // NearestNodeFinder node_finder(layer,label_nodes_pair.second);
    labeled_node_finders.emplace(
        label_nodes_pair.first,
        std::make_unique<NearestNodeFinder>(layer, label_nodes_pair.second));
  }

  for (const auto& id_node_pair : layer.nodes()) {
    // initial node must be active
    if (!info.loop_closure_detected && archived_object_ids.count(id_node_pair.first)) {
      continue;
    }
    auto& attrs = id_node_pair.second->attributes<ObjectNodeAttributes>();

    std::vector<size_t> connections =
        graph.getMeshConnectionIndices(id_node_pair.first);
    if (connections.empty()) {
      VLOG(2) << "Found empty object node "
              << NodeSymbol(id_node_pair.first).getLabel();
      continue;
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
      VLOG(2) << "Invalid centroid for object "
              << NodeSymbol(id_node_pair.first).getLabel();
      continue;
    }

    pcl::PointXYZ pcl_pos;
    centroid.get(pcl_pos);
    attrs.position << pcl_pos.x, pcl_pos.y, pcl_pos.z;

    if (info.allow_node_merging) {
      // TODO(yun) faster and smarter way to find overlap?
      valid_candidates.clear();

      labeled_node_finders[attrs.semantic_label]->find(
          attrs.position,
          1,     // Only need single closest node
          true,  // Skipping self
          [&](NodeId object_id, size_t, double) {
            filterObject(graph,
                         valid_candidates,
                         id_node_pair.first,
                         object_id,
                         semantic_nodes_map[attrs.semantic_label]);
          });

      for (const auto& node_target_id : valid_candidates) {
        const Node& node_target = layer.getNode(node_target_id).value();
        auto& attrs_target = node_target.attributes<ObjectNodeAttributes>();
        // Check for overlap
        if (attrs.bounding_box.isInside(attrs_target.position)) {
          const bool curr_bigger =
              attrs.bounding_box.volume() > attrs_target.bounding_box.volume();
          VLOG(2) << "Merging " << NodeSymbol(id_node_pair.first).getLabel() << " ["
                  << attrs.bounding_box.volume() << "] "
                  << (curr_bigger ? " <- " : " -> ")
                  << NodeSymbol(node_target_id).getLabel() << " ["
                  << attrs_target.bounding_box.volume() << "]";

          // erase smaller node so that it isn't assigned for another merge
          if (curr_bigger) {
            nodes_to_merge[node_target_id] = id_node_pair.first;
            semantic_nodes_map[attrs.semantic_label].erase(node_target_id);
          } else {
            nodes_to_merge[id_node_pair.first] = node_target_id;
            semantic_nodes_map[attrs.semantic_label].erase(id_node_pair.first);
          }
          break;
          // TODO(Yun) Merge ones with larger overlap? For now assume more
          // will be merged next round
        }
      }
    }
  }

  return nodes_to_merge;
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
  std::unordered_set<NodeId> layer_nodes;
  // create the NNFinder
  for (const auto& id_node_pair : layer.nodes()) {
    auto& attrs = id_node_pair.second->attributes<PlaceNodeAttributes>();
    if (info.loop_closure_detected || !attrs.is_active) {
      layer_nodes.insert(id_node_pair.first);
    }
  }

  std::list<NodeId> valid_candidates;
  std::unordered_set<NodeId> missing_nodes;
  std::vector<NodeId> updated_nodes;
  std::map<NodeId, NodeId> nodes_to_merge;
  std::unique_ptr<NearestNodeFinder> node_finder =
      std::make_unique<NearestNodeFinder>(layer, layer_nodes);

  for (const auto& id_node_pair : layer.nodes()) {
    auto& attrs = id_node_pair.second->attributes<PlaceNodeAttributes>();
    if (!places_values.exists(id_node_pair.first)) {
      missing_nodes.insert(id_node_pair.first);
    } else {
      // TODO(nathan) consider updating distance via parents + deformation graph
      attrs.position = places_values.at<gtsam::Pose3>(id_node_pair.first).translation();
    }

    // TODO(yun) faster and smarter way to find overlap?
    // Iterate over active nodes(smaller group) and use
    // NNFinder on inactive nodes (larger group)
    if (!info.allow_node_merging || (!attrs.is_active && !info.loop_closure_detected)) {
      continue;  // don't try to merge nodes when not allowed or inactive
    }

    valid_candidates.clear();
    node_finder->find(attrs.position, 1, true, [&](NodeId object_id, size_t, double) {
      filterPlace(graph, valid_candidates, id_node_pair.first, object_id, layer_nodes);
    });

    for (const auto& node_target_id : valid_candidates) {
      if (graph.hasEdge(id_node_pair.first, node_target_id)) {
        // Do not merge nodes already connected by an edge
        continue;
      }

      const Node& node_target = layer.getNode(node_target_id).value();
      const auto& attrs_target = node_target.attributes<PlaceNodeAttributes>();

      if ((attrs.position - attrs_target.position).norm() > pos_threshold_m) {
        continue;
      }

      if (std::abs(attrs.distance - attrs_target.distance) > distance_tolerance_m) {
        continue;
      }

      if (attrs_target.is_active) {
        // try to prefer merging active into non-active
        nodes_to_merge[node_target_id] = id_node_pair.first;
        layer_nodes.erase(node_target_id);
      } else {
        nodes_to_merge[id_node_pair.first] = node_target_id;
        layer_nodes.erase(id_node_pair.first);
      }

      break;
    }
  }

  if (!missing_nodes.empty()) {
    VLOG(6) << "[Places Layer]: could not update "
            << displayNodeSymbolContainer(missing_nodes);
    for (const auto& place_id : missing_nodes) {
      if (layer.hasNode(place_id)) {
        const Node& missing_node = layer.getNode(place_id).value();
        const auto& attrs = missing_node.attributes<PlaceNodeAttributes>();
        if (!attrs.is_active && !missing_node.hasSiblings()) {
          VLOG(4) << "[Places Layer]: removing node "
                  << NodeSymbol(place_id).getLabel();
          graph.removeNode(place_id);
        }
      }
    }
  }

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
  if (!info.pgmo_values || info.pgmo_values->size() == 0) {
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
      if (!info.pgmo_values->exists(attrs.external_key)) {
        missing_nodes.insert(node->id);
        continue;
      }

      gtsam::Pose3 agent_pose = info.pgmo_values->at<gtsam::Pose3>(attrs.external_key);
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
