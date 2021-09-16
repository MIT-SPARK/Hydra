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
  static std::vector<double> taps{0.0, 0.1, 0.8, 0.35, 0.55, 0.9, 0.05, 0.7, 0.2, 0.65};
  return voxblox::rainbowColorMap(taps.at(room_id % taps.size()));
}

}  // namespace kimera
