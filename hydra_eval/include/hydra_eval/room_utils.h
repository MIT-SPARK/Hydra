#pragma once
#include <kimera_dsg/bounding_box.h>
#include <kimera_dsg/dynamic_scene_graph.h>
#include <visualization_msgs/Marker.h>
#include <voxblox/core/block_hash.h>
#include <yaml-cpp/yaml.h>

#include <map>
#include <vector>

namespace hydra {

using RoomBoundingBoxes = std::map<size_t, std::vector<kimera::BoundingBox>>;
using RoomVoxelIndices = std::map<kimera::NodeId, voxblox::LongIndexSet>;

RoomBoundingBoxes loadBoundingBoxesFromYaml(const std::string& filepath);

void fillRoomIndicesFromDsg(const kimera::DynamicSceneGraph& graph,
                           double voxel_size,
                           size_t voxels_per_side,
                           RoomVoxelIndices& room_indices);

visualization_msgs::Marker drawRoomIndices(const kimera::DynamicSceneGraph& graph,
                                           const RoomVoxelIndices& indices);

}  // namespace hydra
