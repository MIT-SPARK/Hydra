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
#include "hydra/backend/updates/update_gt_rooms_functor.h"

#include <config_utilities/config.h>
#include <config_utilities/types/conversions.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>
#include <spark_dsg/scene_graph.h>
#include <yaml-cpp/yaml.h>

#include "hydra/utils/timing_utilities.h"

namespace hydra {
namespace {

using namespace spark_dsg;

const auto reg = config::RegistrationWithConfig<UpdateFunctor,
                                                UpdateGtRoomsFunctor,
                                                UpdateGtRoomsFunctor::Config>(
    "UpdateGtRoomsFunctor");

void clearRooms(SceneGraph& graph) {
  std::vector<NodeId> rooms;
  for (const auto& node : graph.getLayer(DsgLayers::ROOMS).nodes()) {
    rooms.push_back(node.id);
  }

  for (const auto room : rooms) {
    graph.removeNode(room);
  }
}

std::map<NodeId, size_t> assignRooms(const SceneGraphLayer& places,
                                     const RoomExtents& extents) {
  std::map<NodeId, size_t> labels;
  for (const auto& node : places.nodes()) {
    const auto attrs = node.tryAttributes<PlaceNodeAttributes>();
    if (attrs && !attrs->real_place) {
      continue;
    }

    const auto& position = node.attributes().position;
    if (!position.allFinite()) {
      continue;
    }

    const auto room = extents.getRoomForPoint(position);
    if (room.valid) {
      labels.emplace(node.id, room.index);
    }
  }

  return labels;
}

}  // namespace

RoomExtents::RoomExtents(const RoomExtents::BoundingBoxes& room_extents)
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

void declare_config(UpdateGtRoomsFunctor::Config& config) {
  using namespace config;
  name("UpdateGtRoomsFunctor::Config");
  field<Path::Absolute>(config.ground_truth_rooms_path, "ground_truth_rooms_path");
  field(config.places_layer, "places_layer");
  field<CharConversion>(config.room_prefix, "prefix");
  field(config.sinks, "sinks");
  checkCondition(!config.ground_truth_rooms_path.empty(),
                 "ground_truth_rooms_path must be non-empty!");
  checkCondition(!config.places_layer.empty(), "places_layer must be non-empty!");
}

UpdateGtRoomsFunctor::UpdateGtRoomsFunctor(const Config& config)
    : config(config::checkValid(config)),
      room_extents_(this->config.ground_truth_rooms_path),
      sinks_(Sink::instantiate(this->config.sinks)) {}

void UpdateGtRoomsFunctor::call(const spark_dsg::SceneGraph&,
                                SharedDsgInfo& dsg,
                                const UpdateInfo::ConstPtr& info) const {
  using namespace spark_dsg;
  auto& graph = *dsg.graph;
  const auto places = graph.findLayer(config.places_layer);
  if (!places) {
    return;
  }

  timing::ScopedTimer timer("backend/gt_rooms", info->timestamp_ns);
  const auto labels = assignRooms(*places, room_extents_);
  std::map<size_t, std::vector<NodeId>> clusters;
  for (const auto& [node, room] : labels) {
    clusters[room].push_back(node);
  }

  clearRooms(graph);
  for (const auto& [index, members] : clusters) {
    auto attrs = std::make_unique<RoomNodeAttributes>();
    attrs->semantic_label = 0;
    attrs->position.setZero();
    for (const auto node : members) {
      attrs->position += places->getNode(node).attributes().position;
    }

    attrs->position /= members.size();
    const NodeSymbol room_id(config.room_prefix, index);
    graph.emplaceNode(DsgLayers::ROOMS, room_id, std::move(attrs));
    for (const auto node : members) {
      graph.insertEdge(room_id, node, nullptr, true);
    }
  }

  for (const auto& edge : places->edges()) {
    const auto source = labels.find(edge.source);
    const auto target = labels.find(edge.target);
    if (source == labels.end() || target == labels.end() ||
        source->second == target->second) {
      continue;
    }

    graph.insertEdge(NodeSymbol(config.room_prefix, source->second),
                     NodeSymbol(config.room_prefix, target->second));
  }

  Sink::callAll(sinks_, info->timestamp_ns, room_extents_);
}

}  // namespace hydra
