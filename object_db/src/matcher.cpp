#include "object_db/matcher.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr
object_registration::KeypointsMatcher::calculateKeypoints(
    const sensor_msgs::PointCloud& input_cloud_msg) {
  // Convert sensor_msgs to pcl point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < input_cloud_msg.points.size(); ++i) {
    pcl::PointXYZ point(input_cloud_msg.points[i].x,
                        input_cloud_msg.points[i].y,
                        input_cloud_msg.points[i].z);
    input_cloud->push_back(point);
  }
  return calculateKeypoints(input_cloud);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
object_registration::KeypointsMatcher::calculateKeypoints(
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
  // Create keypoints cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(
      new pcl::PointCloud<pcl::PointXYZI>);
  harris_detector_->setInputCloud(input_cloud);

  // Compute Harris keypoints
  harris_detector_->compute(*keypoints);

  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_xyz(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < keypoints->size(); ++i) {
    pcl::PointXYZ cpoint;
    cpoint.x = keypoints->points[i].x;
    cpoint.y = keypoints->points[i].y;
    cpoint.z = keypoints->points[i].z;
    keypoints_xyz->points.push_back(cpoint);
  }

  return keypoints_xyz;
}

void object_registration::KeypointsMatcher::generateCorrespondences(
    pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr dst_cloud,
    Eigen::Matrix<double, 3, Eigen::Dynamic>* src,
    Eigen::Matrix<double, 3, Eigen::Dynamic>* dst) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_xyz_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < src_cloud->size(); ++i) {
    src_xyz_cloud->push_back({src_cloud->points[i].x,
                              src_cloud->points[i].y,
                              src_cloud->points[i].z});
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr dst_xyz_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < dst_cloud->size(); ++i) {
    dst_xyz_cloud->push_back({dst_cloud->points[i].x,
                              dst_cloud->points[i].y,
                              dst_cloud->points[i].z});
  }

  generateCorrespondences(src_xyz_cloud, dst_xyz_cloud, src, dst);
}

void object_registration::KeypointsMatcher::generateCorrespondences(
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr dst_cloud,
    Eigen::Matrix<double, 3, Eigen::Dynamic>* src,
    Eigen::Matrix<double, 3, Eigen::Dynamic>* dst) {
  // Calculate size of correspondences
  size_t N = src_cloud->size() * dst_cloud->size();
  src->resize(3, N);
  dst->resize(3, N);

  // Generate brute force correspondences
  int count = 0;
  for (size_t i = 0; i < src_cloud->size(); ++i) {
    for (size_t j = 0; j < dst_cloud->size(); ++j) {
      src->col(count) << src_cloud->points[i].x, src_cloud->points[i].y,
          src_cloud->points[i].z;
      dst->col(count) << dst_cloud->points[j].x, dst_cloud->points[j].y,
          dst_cloud->points[j].z;
      count++;
    }
  }
}
