#pragma once

#include <string>

#include <ros/ros.h>

#include "kimera_scene_graph/common.h"

namespace kimera {

class SceneGraph;

class PlacesRoomConnectivityFinder {
 public:
  PlacesRoomConnectivityFinder(const ros::NodeHandle& nh_private,
                               const float& skeleton_z_level,
                               const std::string& world_frame);
  ~PlacesRoomConnectivityFinder() = default;

 public:
  void findPlacesRoomConnectivity(SceneGraph* scene_graph);

  void linkPlaceToRoom(const NodeId& room_id,
                       const NodeId& place_id,
                       SceneGraph* scene_graph);

 private:
  ros::NodeHandle nh_private_;
  ros::Publisher segmented_sparse_graph_pub_;

  std::string world_frame_;

  // TODO(Toni): remove
  float skeleton_z_level_;
};

}  // namespace kimera
