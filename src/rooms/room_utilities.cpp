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
#include "hydra/rooms/room_utilities.h"

#include <yaml-cpp/yaml.h>

namespace hydra {

Eigen::Vector3d getRoomPosition(const SceneGraphLayer& places,
                                const std::unordered_set<NodeId>& cluster,
                                const DistanceAdaptor& get_distance) {
  if (cluster.empty()) {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d room_position = Eigen::Vector3d::Zero();
  for (const auto& place : cluster) {
    room_position += getNodePosition(places, place);
  }
  room_position /= cluster.size();

  double best_radius = 0.0;
  Eigen::Vector3d best_position = Eigen::Vector3d::Zero();
  double best_distance = std::numeric_limits<double>::infinity();

  bool room_in_freespace = false;
  for (const auto& place : cluster) {
    const auto& node = places.getNode(place);
    const auto& position = node.attributes().position;
    const double room_distance = (room_position - position).norm();
    const double distance = get_distance(node);
    const double distance_to_freespace = room_distance - distance;

    if (distance_to_freespace < best_distance) {
      best_distance = distance_to_freespace;
      best_radius = distance;
      best_position = position;
    }
  }

  if (room_in_freespace) {
    return room_position;
  }

  // project room centroid to edge of free-space radius
  Eigen::Vector3d unit_vector = (room_position - best_position).normalized();
  return best_position + unit_vector * best_radius;
}

void addEdgesToRoomLayer(const SceneGraphLayer& places,
                         const std::map<NodeId, size_t>& labels,
                         const std::map<size_t, NodeId>& label_to_room_map,
                         SceneGraphLayer& rooms) {
  for (const auto& id_node_pair : places.nodes()) {
    const auto place = id_node_pair.first;
    const auto label = labels.find(place);
    if (label == labels.end()) {
      continue;
    }

    const auto parent = label_to_room_map.find(label->second);
    if (parent == label_to_room_map.end()) {
      continue;
    }

    for (const auto& sibling : id_node_pair.second->siblings()) {
      const auto sibling_label = labels.find(sibling);
      if (sibling_label == labels.end()) {
        continue;
      }

      const auto sibling_parent = label_to_room_map.find(sibling_label->second);
      if (sibling_parent == label_to_room_map.end()) {
        continue;
      }

      if (parent->second == sibling_parent->second) {
        continue;
      }

      // repeated inserts don't matter
      rooms.insertEdge(parent->second, sibling_parent->second);
    }
  }
}

void addEdgesToRoomLayer(DynamicSceneGraph& graph,
                         const std::set<NodeId>& active_rooms) {
  for (const auto node_id : active_rooms) {
    const auto& node = graph.getNode(node_id);
    const auto room_siblings = node.siblings();
    for (const auto other : room_siblings) {
      if (active_rooms.count(other)) {
        // clean potential edges
        graph.removeEdge(node_id, other);
      }
    }

    for (const auto child_id : node.children()) {
      const auto& child = graph.getNode(child_id);
      if (child.layer.partition) {
        continue;  // skip searching over agent nodes
      }

      for (const auto& sibling_id : child.siblings()) {
        const auto& sibling = graph.getNode(sibling_id);
        const auto parent = sibling.getParent();
        if (!parent) {
          continue;
        }

        if (parent == node_id) {
          // technically self-edges get rejected, but skipping makes more sense
          continue;
        }

        // repeated inserts don't matter
        graph.insertEdge(node_id, *parent);
      }
    }
  }
}

RoomExtents::RoomExtents(
    const std::vector<std::vector<spark_dsg::BoundingBox>>& room_extents)
    : room_bounding_boxes(room_extents) {}

RoomExtents::RoomExtents(const std::filesystem::path& path_to_yaml) {
  YAML::Node root = YAML::LoadFile(path_to_yaml);
  std::vector<std::vector<spark_dsg::BoundingBox>> result;

  for (const auto& key_group : root) {
    auto& group_node = key_group.second;
    auto& group = result.emplace_back();

    for (const auto& box_node : group_node) {
      // Extract center
      const auto& center_node = box_node["center"];
      Eigen::Vector3f center(center_node[0].as<float>(),
                             center_node[1].as<float>(),
                             center_node[2].as<float>());

      // Extract extents
      const auto& extents_node = box_node["extents"];
      Eigen::Vector3f dimensions(extents_node[0].as<float>(),
                                 extents_node[1].as<float>(),
                                 extents_node[2].as<float>());

      // Extract rotation
      const auto& rot_node = box_node["rotation"];
      Eigen::Quaternionf rotation(rot_node["w"].as<float>(),
                                  rot_node["x"].as<float>(),
                                  rot_node["y"].as<float>(),
                                  rot_node["z"].as<float>());

      group.emplace_back(dimensions, center, rotation);
    }
  }

  room_bounding_boxes = result;
}

RoomExtents::QueryResult RoomExtents::getRoomForPoint(Eigen::Vector3d point) const {
  for (size_t room_idx = 0; room_idx < room_bounding_boxes.size(); ++room_idx) {
    for (const auto& bb : room_bounding_boxes.at(room_idx)) {
      if (bb.contains(point)) {
        return {true, room_idx};
      }
    }
  }
  return {false, 0};
}

}  // namespace hydra
