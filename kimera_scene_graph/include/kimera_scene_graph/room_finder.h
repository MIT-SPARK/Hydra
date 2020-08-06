#pragma once

#include <ros/ros.h>

// for subscribers
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>

#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <voxblox/mesh/mesh.h>

#include <kimera_semantics/common.h>

#include "kimera_scene_graph/common.h"

namespace kimera {

class SceneGraph;

template <class T>
typename pcl::PointCloud<T>::Ptr passThroughFilter1D(
    const typename pcl::PointCloud<T>::Ptr& input,
    const std::string& dimension,
    const float& min,
    const float& max,
    const bool& negative_limits = false) {
  typename pcl::PointCloud<T>::Ptr cloud_filtered(new pcl::PointCloud<T>);
  // Create the filtering object
  pcl::PassThrough<T> pass;
  pass.setInputCloud(input);
  pass.setFilterFieldName(dimension);
  pass.setFilterLimits(min, max);
  pass.setFilterLimitsNegative(negative_limits);
  pass.filter(*cloud_filtered);
  return cloud_filtered;
}

template <class T>
typename pcl::PointCloud<T>::Ptr downsamplePcl(
    const typename pcl::PointCloud<T>::Ptr& input,
    const float& leaf_size = 0.05f,
    const bool& approx_downsampling = false) {
  typename pcl::PointCloud<T>::Ptr downsampled_pcl(new pcl::PointCloud<T>);
  if (approx_downsampling) {
    pcl::ApproximateVoxelGrid<T> downsampler;
    downsampler.setLeafSize(leaf_size, leaf_size, leaf_size);
    downsampler.setInputCloud(input);
    downsampler.filter(*downsampled_pcl);
  } else {
    pcl::VoxelGrid<T> downsampler;
    downsampler.setInputCloud(input);
    downsampler.setLeafSize(leaf_size, leaf_size, leaf_size);
    downsampler.filter(*downsampled_pcl);
  }
  return downsampled_pcl;
}

class RoomFinder {
 public:
  /**
   * @brief RoomFinder
   * @param nh_private
   * @param world_frame
   * @param esdf_slice_level Height at which the ESDF slice is computed for room
   * clustering.
   * @param skeleton_z_level
   */
  RoomFinder(const ros::NodeHandle& nh_private,
             const std::string& world_frame,
             const vxb::FloatingPoint& esdf_slice_level,
             const float& skeleton_z_level);
  ~RoomFinder() = default;

  /**
   * @brief findRooms Uses Semantic ESDF
   * @param esdf_layer
   * @param[out] Scene Graph to be updated with Room layer.
   * @return
   */
  IntensityPointCloud::Ptr findRooms(
      const vxb::Layer<vxb::EsdfVoxel>& esdf_layer,
      SceneGraph* scene_graph);

 private:
  /**
   * @brief updateSceneGraph Updates the scene graph with the room instances.
   * It does not estimate inter-room connectivity, nor parent/children
   * connectivity
   * @param[in] room_centroids Centroids of each room, as found by findRooms
   * @param[in] room_pcls Pointclouds associated to the room (this is a first
   * approximation of the room layout.
   * @param[out] scene_graph Scene graph to be updated
   */
  void updateSceneGraph(const Centroids& room_centroids,
                        const std::vector<ColorPointCloud::Ptr>& room_pcls,
                        SceneGraph* scene_graph);

  // Visualization functions
  /**
   * @brief publishTruncatedEsdf For visualization only. Publishes the truncated
   * ESDF slice to visualize the layout of the room (previous to segmentation).
   * @param esdf_pcl Pointcloud extracted from the ESDF layer
   */
  void publishTruncatedEsdf(const IntensityPointCloud::Ptr& esdf_pcl);

 private:
  ros::NodeHandle nh_private_;
  ros::Publisher pcl_pub_;
  ros::Publisher esdf_truncated_pub_;

  std::string world_frame_;

  // Params
  //! Height where to cut the ESDF for room segmentation
  vxb::FloatingPoint esdf_slice_level_;
  //! For visualization
  float skeleton_z_level_;

  // Counter
  NodeId next_room_id_ = 1;  // do not init to 0 (special null id).

  bool visualize_;
};

}  // namespace kimera
