#pragma once
#include <kimera_dsg/pcl_types.h>
#include <kimera_dsg/scene_graph_types.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/point_cloud.h>
#include <voxblox/core/color.h>  // just for getroomcolor
#include <vector>

namespace kimera {

// TODO(Toni): create a structure to hold both Centroids and ObjectPointClouds
// associated to the centroids in the same object...
typedef pcl::CentroidPoint<Point> Centroid;
typedef std::vector<Centroid> Centroids;

// Hardcoded for now
static constexpr int kPlaceSemanticLabel = 23u;
static constexpr int kRoomSemanticLabel = 21u;
static constexpr int kBuildingSemanticLabel = 22u;
static constexpr float kEsdfTruncation = 0.3;

inline voxblox::Color getRoomColor(const NodeId& room_id) {
  return voxblox::rainbowColorMap(static_cast<double>(room_id % 20) / 20.0);
}

}  // namespace kimera
