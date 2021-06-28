#pragma once
#include <kimera_dsg/scene_graph_types.h>
#include <voxblox/core/color.h>  // just for getroomcolor

namespace kimera {


// Hardcoded for now
static constexpr int kPlaceSemanticLabel = 23u;
static constexpr int kRoomSemanticLabel = 21u;
static constexpr int kBuildingSemanticLabel = 22u;
static constexpr float kEsdfTruncation = 0.3;

inline voxblox::Color getRoomColor(const NodeId& room_id) {
  return voxblox::rainbowColorMap(static_cast<double>(room_id % 20) / 20.0);
}

}  // namespace kimera
