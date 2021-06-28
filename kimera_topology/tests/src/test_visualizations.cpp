#include "kimera_topology_test/test_visualizations.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <tf2_eigen/tf2_eigen.h>
#include <voxblox/utils/evaluation_utils.h>
#include <voxblox_ros/ptcloud_vis.h>

#include <limits>

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
using namespace voxblox;

namespace kimera {
namespace topology {
namespace test_helpers {

Marker getDefaultMarker(int id, double scale, const std::string& ns) {
  Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.action = Marker::ADD;
  marker.id = id;
  marker.ns = ns;
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.a = 1.0;
  return marker;
}

void setColor(Marker& marker, double r, double g, double b, double a = 1.0) {
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
}

Eigen::MatrixXd getCameraBearings(const Eigen::Quaterniond& rotation,
                                  const Eigen::Vector2i& resolution,
                                  double fov) {
  Eigen::MatrixXd points(3, 4);
  points.block<3, 1>(0, 0);

  const double u = static_cast<double>(resolution(0));
  const double v = static_cast<double>(resolution(1));
  const double f = u / (2.0 * std::tan(fov / 2.0));

  Eigen::Matrix3d R = rotation.toRotationMatrix();

  Eigen::Vector3d lulv;
  lulv << 1.0, -u / (2.0 * f), -v / (2.0 * f);
  points.block<3, 1>(0, 0) = (R * lulv.normalized());

  Eigen::Vector3d luhv;
  luhv << 1.0, -u / (2.0 * f), (v - 1.0) / (2.0 * f);
  points.block<3, 1>(0, 1) = (R * luhv.normalized());

  Eigen::Vector3d huhv;
  huhv << 1.0, (u - 1.0) / (2.0 * f), (v - 1.0) / (2.0 * f);
  points.block<3, 1>(0, 2) = (R * huhv.normalized());

  Eigen::Vector3d hulv;
  hulv << 1.0, (u - 1.0) / (2.0 * f), -v / (2.0 * f);
  hulv.normalize();
  points.block<3, 1>(0, 3) = (R * hulv.normalized());

  return points;
}

MarkerArray makeCameraMarker(const Eigen::Quaterniond& rotation,
                             const Eigen::Vector3d& translation,
                             const Eigen::Vector2i& resolution,
                             size_t pose_id,
                             double fov,
                             double max_distance,
                             double focal_plane_depth,
                             double marker_scale,
                             double line_scale) {
  Eigen::MatrixXd camera_points = getCameraBearings(rotation, resolution, fov);

  Marker points_marker = getDefaultMarker(3 * pose_id, marker_scale, "camera");
  setColor(points_marker, 1.0, 0.0, 0.0);
  points_marker.type = Marker::SPHERE_LIST;

  Marker lines_marker = getDefaultMarker(3 * pose_id + 1, line_scale, "camera");
  lines_marker.type = Marker::LINE_LIST;
  setColor(lines_marker, 0.0, 0.0, 1.0);

  Marker ray_marker = getDefaultMarker(3 * pose_id + 2, line_scale / 2.0, "camera");
  ray_marker.type = Marker::LINE_LIST;
  setColor(ray_marker, 0.0, 0.0, 0.0);

  geometry_msgs::Point center;
  tf2::convert(translation, center);
  points_marker.points.push_back(center);

  for (int c = 0; c < camera_points.cols(); ++c) {
    int next_idx = (c + 1) % 4;
    Eigen::Vector3d curr_pos =
        focal_plane_depth * camera_points.block<3, 1>(0, c) + translation;
    Eigen::Vector3d next_pos =
        focal_plane_depth * camera_points.block<3, 1>(0, next_idx) + translation;
    Eigen::Vector3d ray_pos =
        max_distance * camera_points.block<3, 1>(0, c) + translation;

    geometry_msgs::Point curr_point;
    tf2::convert(curr_pos, curr_point);

    geometry_msgs::Point next_point;
    tf2::convert(next_pos, next_point);

    geometry_msgs::Point ray_point;
    tf2::convert(ray_pos, ray_point);

    points_marker.points.push_back(curr_point);

    lines_marker.points.push_back(center);
    lines_marker.points.push_back(curr_point);
    lines_marker.points.push_back(curr_point);
    lines_marker.points.push_back(next_point);

    ray_marker.points.push_back(center);
    ray_marker.points.push_back(ray_point);
  }

  MarkerArray msg;
  msg.markers.push_back(points_marker);
  msg.markers.push_back(lines_marker);
  msg.markers.push_back(ray_marker);
  return msg;
}

template <typename VoxelType>
Marker makeEsdfMarker(
    const Layer<VoxelType>& layer,
    double min_distance_threshold = 0.0,
    double max_distance_threshold = std::numeric_limits<FloatingPoint>::infinity()) {
  Marker marker = getDefaultMarker(0, layer.voxel_size(), "esdf");
  marker.type = Marker::CUBE_LIST;
  setColor(marker, 0.662, .0313, .7607);

  BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);

  for (const auto& idx : blocks) {
    const auto& block = layer.getBlockByIndex(idx);
    for (size_t i = 0; i < block.num_voxels(); ++i) {
      const auto& voxel = block.getVoxelByLinearIndex(i);
      if (!voxblox::utils::isObservedVoxel(voxel)) {
        continue;
      }

      if (std::abs(voxel.distance) <= min_distance_threshold) {
        continue;
      }

      if (std::abs(voxel.distance) >= max_distance_threshold) {
        continue;
      }

      Point center = block.computeCoordinatesFromLinearIndex(i);
      geometry_msgs::Point marker_center;
      marker_center.x = center.x();
      marker_center.y = center.y();
      marker_center.z = center.z();
      marker.points.push_back(marker_center);
    }
  }

  return marker;
}

inline Eigen::Vector3d getOffset(double side_length,
                                 bool x_high,
                                 bool y_high,
                                 bool z_high) {
  Eigen::Vector3d offset;
  offset(0) = x_high ? side_length : 0.0;
  offset(1) = y_high ? side_length : 0.0;
  offset(2) = z_high ? side_length : 0.0;
  return offset;
}

geometry_msgs::Point getPointFromMatrix(const Eigen::MatrixXd& matrix, int col) {
  geometry_msgs::Point point;
  tf2::convert(matrix.block<3, 1>(0, col).eval(), point);
  return point;
}

void fillMarkerFromBlock(Marker& marker,
                         const Eigen::Vector3d& position,
                         double side_length) {
  Eigen::MatrixXd corners(3, 8);
  for (int c = 0; c < corners.cols(); ++c) {
    // x: lsb, y: second lsb, z: third lsb
    corners.block<3, 1>(0, c) =
        position +
        getOffset(side_length, ((c & 0x01) != 0), ((c & 0x02) != 0), ((c & 0x04) != 0));
  }

  for (int c = 0; c < corners.cols(); ++c) {
    // edges are 1-bit pertubations
    int x_neighbor = c | 0x01;
    int y_neighbor = c | 0x02;
    int z_neighbor = c | 0x04;
    if (c != x_neighbor) {
      marker.points.push_back(getPointFromMatrix(corners, c));
      marker.points.push_back(getPointFromMatrix(corners, x_neighbor));
    }
    if (c != y_neighbor) {
      marker.points.push_back(getPointFromMatrix(corners, c));
      marker.points.push_back(getPointFromMatrix(corners, y_neighbor));
    }
    if (c != z_neighbor) {
      marker.points.push_back(getPointFromMatrix(corners, c));
      marker.points.push_back(getPointFromMatrix(corners, z_neighbor));
    }
  }
}

template <typename LayerType>
Marker makeBlocksMarker(const LayerType& layer) {
  Marker marker = getDefaultMarker(0, 0.005, "esdf_blocks");
  marker.type = Marker::LINE_LIST;
  setColor(marker, 0.662, .0313, 0.7607, 0.8);

  BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);

  for (const auto& idx : blocks) {
    const auto& block = layer.getBlockByIndex(idx);
    Eigen::Vector3f block_pos = block.origin();
    fillMarkerFromBlock(marker, block_pos.cast<double>(), block.block_size());
  }

  return marker;
}

TestVisualizer::TestVisualizer()
    : camera_marker_scale(0.05), camera_line_scale(0.02), focal_plane_depth(0.3) {
  ros::NodeHandle nh;

  esdf_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("esdf", 10, true);
  esdf_block_pub = nh.advertise<Marker>("esdf_block_marker", 10, true);
  marker_pub = nh.advertise<Marker>("esdf_marker", 10, true);
  camera_pub = nh.advertise<MarkerArray>("camera", 10, true);
  pointcloud_pub = nh.advertise<Marker>("input_pointcloud", 10, true);
  tsdf_pub = nh.advertise<Marker>("tsdf_marker", 10, true);
  gvd_pub = nh.advertise<Marker>("gvd_marker", 10, true);

  ros::param::get("~camera_marker_scale", camera_marker_scale);
  ros::param::get("~camera_line_scale", camera_line_scale);
  ros::param::get("~focal_plane_depth", focal_plane_depth);
}

void TestVisualizer::visualize(const Layer<EsdfVoxel>& layer,
                               const Transformation& world_T_camera,
                               size_t pose_id,
                               const Eigen::Vector2i& resolution,
                               double fov,
                               double max_depth,
                               double min_esdf_depth,
                               double max_esdf_depth) const {
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  voxblox::createDistancePointcloudFromEsdfLayer(layer, &pointcloud);
  pcl_conversions::toPCL(ros::Time::now(), pointcloud.header.stamp);
  pointcloud.header.frame_id = "world";
  esdf_pub.publish(pointcloud);

  Marker marker = makeEsdfMarker(layer, min_esdf_depth, max_esdf_depth);
  marker_pub.publish(marker);

  Marker block_marker = makeBlocksMarker(layer);
  esdf_block_pub.publish(block_marker);

  MarkerArray camera =
      makeCameraMarker(world_T_camera.getEigenQuaternion().cast<double>(),
                       world_T_camera.getPosition().cast<double>().eval(),
                       resolution,
                       pose_id,
                       fov,
                       max_depth,
                       focal_plane_depth,
                       camera_marker_scale,
                       camera_line_scale);
  camera_pub.publish(camera);
}

void TestVisualizer::visualizePointcloud(const voxblox::Pointcloud& pointcloud) const {
  Marker marker = getDefaultMarker(0, 0.01, "input_pointcloud");
  marker.type = Marker::SPHERE_LIST;
  setColor(marker, 0.0, 0.0, 1.0, 0.6);

  for (const auto& point : pointcloud) {
    geometry_msgs::Point msg_point;
    msg_point.x = point.x();
    msg_point.y = point.y();
    msg_point.z = point.z();
    marker.points.push_back(msg_point);
  }

  pointcloud_pub.publish(marker);
}

void TestVisualizer::visualizeTsdf(const Layer<TsdfVoxel>& layer, double truncation_distance) const {
  Marker marker = makeEsdfMarker(layer, -0.1, truncation_distance);
  tsdf_pub.publish(marker);
}

void TestVisualizer::visualizeGvd(const Layer<GvdVoxel>& layer) const {
  Marker marker = getDefaultMarker(0, layer.voxel_size(), "gvd_cells");
  marker.type = Marker::CUBE_LIST;
  setColor(marker, 1.0, 0.0, 0.0, 0.6);

  BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);

  for (const auto& idx : blocks) {
    const auto& block = layer.getBlockByIndex(idx);
    for (size_t i = 0; i < block.num_voxels(); ++i) {
      const auto& voxel = block.getVoxelByLinearIndex(i);
      if (!voxel.is_voronoi) {
        continue;
      }

      Point center = block.computeCoordinatesFromLinearIndex(i);
      geometry_msgs::Point marker_center;
      marker_center.x = center.x();
      marker_center.y = center.y();
      marker_center.z = center.z();
      marker.points.push_back(marker_center);
    }
  }

  gvd_pub.publish(marker);
}

void TestVisualizer::visualize(const Layer<GvdVoxel>& layer,
                               const Transformation& world_T_camera,
                               size_t pose_id,
                               const Eigen::Vector2i& resolution,
                               double fov,
                               double max_depth,
                               double min_esdf_depth,
                               double max_esdf_depth) const {
  Marker marker = makeEsdfMarker(layer, min_esdf_depth, max_esdf_depth);
  marker_pub.publish(marker);

  Marker block_marker = makeBlocksMarker(layer);
  esdf_block_pub.publish(block_marker);

  MarkerArray camera =
      makeCameraMarker(world_T_camera.getEigenQuaternion().cast<double>(),
                       world_T_camera.getPosition().cast<double>().eval(),
                       resolution,
                       pose_id,
                       fov,
                       max_depth,
                       focal_plane_depth,
                       camera_marker_scale,
                       camera_line_scale);
  camera_pub.publish(camera);
}

void TestVisualizer::waitForVisualization() const {
  LOG(WARNING) << "Spinning until ctrl-c encountered";
  ros::spin();
}

}  // namespace test_helpers
}  // namespace topology
}  // namespace kimera
