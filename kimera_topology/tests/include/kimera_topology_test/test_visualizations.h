#pragma once
#include <ros/ros.h>

#include <kimera_topology/voxblox_types.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Geometry>

namespace kimera {
namespace topology {
namespace test_helpers {

class TestVisualizer {
 public:
  TestVisualizer();

  virtual ~TestVisualizer() = default;

  void visualizePointcloud(const voxblox::Pointcloud& pointcloud) const;

  void visualizeGvd(const Layer<GvdVoxel>& layer) const;

  void visualizeTsdf(const Layer<TsdfVoxel>& layer, double truncation_distance) const;

  void visualize(const voxblox::Layer<voxblox::EsdfVoxel>& layer,
                 const voxblox::Transformation& world_T_camera,
                 size_t pose_id,
                 const Eigen::Vector2i& resolution,
                 double fov,
                 double max_depth,
                 double min_esdf_depth,
                 double max_esdf_depth) const;

  void visualize(const voxblox::Layer<GvdVoxel>& layer,
                 const voxblox::Transformation& world_T_camera,
                 size_t pose_id,
                 const Eigen::Vector2i& resolution,
                 double fov,
                 double max_depth,
                 double min_esdf_depth,
                 double max_esdf_depth) const;

  void waitForVisualization() const;

  double camera_marker_scale;
  double camera_line_scale;
  double focal_plane_depth;
  ros::Publisher esdf_pub;
  ros::Publisher tsdf_pub;
  ros::Publisher gvd_pub;
  ros::Publisher marker_pub;
  ros::Publisher camera_pub;
  ros::Publisher esdf_block_pub;
  ros::Publisher pointcloud_pub;
};

}  // namespace test_helpers
}  // namespace topology
}  // namespace kimera
