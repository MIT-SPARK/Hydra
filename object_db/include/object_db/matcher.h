#pragma once

#include <pcl/keypoints/harris_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud.h>

#include <Eigen/Core>
#include <cstdlib>
#include <random>
#include <vector>

namespace object_registration {

struct MatcherParams {
  unsigned int num_threads = 4;
  double radius = 0.1;
  double nms_threshold = 1e-6;
  bool non_max_suppression = true;
  bool set_refine = true;
};

class KeypointsMatcher {
 protected:
  // Harris detector member
  pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::Ptr harris_detector_;

  // Default harris parameters
  MatcherParams params_;

 public:
  KeypointsMatcher() : KeypointsMatcher(true, false, 0.3, 1e-4, 4) {}

  KeypointsMatcher(MatcherParams params)
      : KeypointsMatcher(params.non_max_suppression,
                         params.set_refine,
                         params.radius,
                         params.nms_threshold,
                         params.num_threads) {}

  KeypointsMatcher(bool use_non_max_suppression,
                   bool use_refine,
                   float radius,
                   float nms_threshold,
                   int num_threads) {
    // Update variables
    params_.non_max_suppression = use_non_max_suppression;
    params_.set_refine = use_refine;
    params_.radius = radius;
    params_.nms_threshold = nms_threshold;
    params_.num_threads = num_threads;

    // Reset Harris corner detector
    harris_detector_ =
        std::make_unique<pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>>(
            pcl::HarrisKeypoint3D<pcl::PointXYZ,
                                  pcl::PointXYZI>::ResponseMethod::HARRIS,
            params_.radius,
            params_.nms_threshold);
    harris_detector_->setNonMaxSupression(params_.non_max_suppression);
    harris_detector_->setRefine(params_.set_refine);
    harris_detector_->setNumberOfThreads(params_.num_threads);
  }

  void updateParameters(bool use_non_max_suppression,
                        bool use_refine,
                        float radius,
                        float nms_threshold,
                        int max_threads) {
    // Update variables
    params_.non_max_suppression = use_non_max_suppression;
    params_.set_refine = use_refine;
    params_.radius = radius;
    params_.nms_threshold = nms_threshold;
    params_.num_threads = max_threads;
  }

  /**
   * Given a sensor_msgs::PointCloud, return an eigen matrix representing
   * correspondences
   * @param input_cloud_msg
   * @return
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr calculateKeypoints(
      const sensor_msgs::PointCloud& input_cloud_msg);

  /**
   * Given a PCL point cloud (point xyz), return an eigen matrix representing
   * correspondences
   * @param input_cloud
   * @return
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr calculateKeypoints(
      pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);

  /**
   * Given two pcl point cloud storing keypoints, generate good correspondences.
   *
   * Currently using a brute force approach.
   * @param src_cloud
   * @param cloud_2
   */
  void generateCorrespondences(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr dst_cloud,
                               Eigen::Matrix<double, 3, Eigen::Dynamic>* src,
                               Eigen::Matrix<double, 3, Eigen::Dynamic>* dst);

  /**
   * Given two pcl point cloud storing keypoints, generate good correspondences.
   *
   * Currently using a brute force approach.
   * @param src_cloud
   * @param cloud_2
   */
  void generateCorrespondences(pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr dst_cloud,
                               Eigen::Matrix<double, 3, Eigen::Dynamic>* src,
                               Eigen::Matrix<double, 3, Eigen::Dynamic>* dst);
};
}  // namespace object_registration
