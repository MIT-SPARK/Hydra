#include "hydra_eval/room_utils.h"

#include <voxblox/core/layer.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/utils/planning_utils.h>

namespace hydra {

using kimera::BoundingBox;
using kimera::DynamicSceneGraph;
using kimera::KimeraDsgLayers;
using kimera::NodeId;
using kimera::PlaceNodeAttributes;
using kimera::SceneGraphLayer;
using kimera::SceneGraphNode;
using kimera::SemanticNodeAttributes;

Eigen::Vector3f loadVector(const YAML::Node& node) {
  Eigen::Vector3f to_return;
  to_return << node[0].as<float>(), node[1].as<float>(), node[2].as<float>();
  return to_return;
}

RoomBoundingBoxes loadBoundingBoxesFromYaml(const std::string& filepath) {
  YAML::Node root = YAML::LoadFile(filepath);

  const double angle_degrees = !root["angle"] ? 0.0 : root["angle"].as<double>();
  const double angle = angle_degrees * M_PI / 180.0;

  RoomBoundingBoxes boxes;
  for (size_t room = 0; room < root["rooms"].size(); ++room) {
    std::vector<BoundingBox> room_boxes;
    for (const auto& bbox : root["rooms"][room]) {
      Eigen::Vector3f pos = loadVector(bbox["pos"]);
      Eigen::Vector3f scale = loadVector(bbox["scale"]);
      Eigen::Quaternionf rot(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));

      room_boxes.push_back(BoundingBox(BoundingBox::Type::RAABB,
                                       -scale / 2.0f,
                                       scale / 2.0f,
                                       pos,
                                       rot.toRotationMatrix()));
    }

    boxes[room] = room_boxes;
  }
  return boxes;
}

void fillRoomIndicesFromDsg(const DynamicSceneGraph& graph,
                            double voxel_size,
                            size_t voxels_per_side,
                            RoomVoxelIndices& room_indices) {
  voxblox::Layer<voxblox::TsdfVoxel> layer(voxel_size, voxels_per_side);

  const SceneGraphLayer& rooms = graph.getLayer(KimeraDsgLayers::ROOMS).value();
  const SceneGraphLayer& places = graph.getLayer(KimeraDsgLayers::PLACES).value();

  for (const auto& id_node_pair : rooms.nodes()) {
    const NodeId room = id_node_pair.first;
    room_indices[room] = voxblox::LongIndexSet();

    for (const auto& child : id_node_pair.second->children()) {
      const SceneGraphNode& place = places.getNode(child).value();
      const auto& attrs = place.attributes<PlaceNodeAttributes>();

      voxblox::HierarchicalIndexMap indices;
      voxblox::utils::getSphereAroundPoint(
          layer, attrs.position.cast<float>(), attrs.distance, &indices);

      for (const auto& block_idx_pair : indices) {
        for (const auto& voxel_idx : block_idx_pair.second) {
          room_indices[room].insert(voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
              block_idx_pair.first, voxel_idx, voxels_per_side));
        }
      }
    }
  }
}

visualization_msgs::Marker drawRoomIndices(const DynamicSceneGraph& graph,
                                           const RoomVoxelIndices& indices,
                                           double voxel_size,
                                           size_t voxels_per_side) {
  voxblox::Layer<voxblox::TsdfVoxel> layer(voxel_size, voxels_per_side);

  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "world";
  marker.ns = "room_freespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.scale.x = voxel_size;
  marker.scale.y = voxel_size;
  marker.scale.z = voxel_size;

  for (const auto& id_index_pair : indices) {
    const auto& attrs = graph.getNode(id_index_pair.first)
                            .value()
                            .get()
                            .attributes<SemanticNodeAttributes>();
    std_msgs::ColorRGBA color;
    color.r = attrs.color(0) / 255.0;
    color.g = attrs.color(1) / 255.0;
    color.b = attrs.color(2) / 255.0;
    color.a = 0.7;

    for (const auto& index : id_index_pair.second) {
      voxblox::BlockIndex block_idx;
      voxblox::VoxelIndex voxel_idx;
      voxblox::getBlockAndVoxelIndexFromGlobalVoxelIndex(
          index, layer.voxels_per_side(), &block_idx, &voxel_idx);

      if (!layer.hasBlock(block_idx)) {
        continue;
      }

      auto pos =
          layer.getBlockByIndex(block_idx).computeCoordinatesFromVoxelIndex(voxel_idx);
      geometry_msgs::Point point;
      point.x = pos(0);
      point.y = pos(1);
      point.z = pos(2);
      marker.points.push_back(point);
      marker.colors.push_back(color);
    }
  }
  return marker;
}

}  // namespace hydra
