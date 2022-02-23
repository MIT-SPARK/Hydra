#pragma once
#include "kimera_batch_dsg/common.h"
#include "kimera_batch_dsg/connectivity_utils.h"

#include <kimera_dsg/scene_graph.h>
#include <kimera_semantics/common.h>
#include <ros/ros.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <voxblox/mesh/mesh.h>

namespace kimera {

class RoomFinder {
 public:
  /**
   * @brief RoomFinder
   * @param nh_private
   * @param world_frame
   * @param esdf_slice_level Height at which the ESDF slice is computed for room
   * clustering.
   */
  RoomFinder(const ros::NodeHandle& nh_private,
             const std::string& world_frame,
             vxb::FloatingPoint esdf_slice_level,
             bool visualize = false);

  ~RoomFinder() = default;

  /**
   * @brief findRooms Uses Semantic ESDF
   * @param esdf_layer
   * @param[out] Scene Graph to be updated with Room layer.
   * @return
   */
  RoomHullMap findRooms(const vxb::Layer<vxb::EsdfVoxel>& esdf_layer,
                        SceneGraph* scene_graph);

  // private:
  /**
   * @brief publishTruncatedEsdf For visualization only. Publishes the truncated
   * ESDF slice to visualize the layout of the room (previous to segmentation).
   * @param esdf_pcl Pointcloud extracted from the ESDF layer
   */
  // void publishTruncatedEsdf(const IntensityPointCloud::Ptr& esdf_pcl);

 private:
  ros::NodeHandle nh_private_;
  ros::Publisher pcl_pub_;
  ros::Publisher esdf_truncated_pub_;

  std::string world_frame_;

  // Height where to cut the ESDF for room segmentation
  vxb::FloatingPoint esdf_slice_level_;

  // Counter
  NodeSymbol next_room_id_;

  bool visualize_;
};

}  // namespace kimera
