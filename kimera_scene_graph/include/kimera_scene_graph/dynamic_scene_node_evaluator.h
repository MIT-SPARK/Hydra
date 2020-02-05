#pragma once

#include <unordered_map>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <graph_cmr_ros/SMPL.h>
#include <graph_cmr_ros/SMPLList.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "kimera_scene_graph/CentroidErrorRequest.h"
#include "kimera_scene_graph/scene_node.h"
#include "kimera_scene_graph/semantic_ros_publishers.h"

#include "kimera_scene_graph/CentroidErrorRequest.h"
#include "kimera_scene_graph/dynamic_scene_node.h"

class DynamicSceneGraphEvaluator {
 public:
  DynamicSceneGraphEvaluator(const ros::NodeHandle& nh,
                             const ros::NodeHandle& nh_private);
  virtual ~DynamicSceneGraphEvaluator() = default;

 public:
  /**
   * @brief Service wrapper to calculate and return the current error values.
   * @param request: The empty request filled in by the ros service call
   * @param response: The response with all error means, variances, and max
   * values.
   * @return True always.
   */
  bool errorServiceCall(
      kimera_scene_graph::CentroidErrorRequest::Request& request,
      kimera_scene_graph::CentroidErrorRequest::Response& response) {
    LOG(INFO) << "DynamicSceneNode: Optimized Error Requested";
    computeOptimizedError(response);
    response.raw_error = raw_err_ / num_meshes_;
    response.raw_variance = calcVariance(raw_err_, raw_sq_err_, num_meshes_);
    response.max_raw_error = max_raw_err_;
    response.raw_count = num_meshes_;
    return true;
  }

  /**
   * @brief computeOptimizedError prunes graphs, runs the optimization and
   * computes the
   *  mean, variance, and maximum centroid error of the result.
   * @param res: The service response to be filled in by the function.
   */
  void computeOptimizedError(
      kimera_scene_graph::CentroidErrorRequest::Response& res);

  /**
   * @brief humanCallback wraps the humanCallback function for the
   * dynamicSceneNode and tracks the
   *   corresponding ground-truth data
   * @param msg: The SMPLList message with the corresponding humans
   */
  void humanCallback(const graph_cmr_ros::SMPLList::ConstPtr& msg);

  /**
   * @brief getAllTransforms finds all transformations to the human frames at
   * the given timestep
   */
  bool getAllTransforms(std::vector<tf::StampedTransform>& out_transforms,
                        ros::Time& time) {
    // All human frames are labelled object_n
    size_t i = 0;
    while (true) {
      tf::StampedTransform new_transform;
      try {
        listener_.lookupTransform(
            world_frame_, "object_" + std::to_string(i), time, new_transform);
        out_transforms.push_back(new_transform);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        break;
      }
      i++;
    }

    // Ensure that all transforms were read in properly.
    return i == num_humans_;
  }

  /**
   * @brief smallestCentroidDistance
   * @param node: The node to find the smallest distance for.
   * @return -1 if the node's timestamp is not in the ground-truth directory.
   * The distance between the node and
   *  the closest ground-truth node for its timestamp otherwise.
   */
  float smallestCentroidDistance(kimera::DynamicSceneNode& node) {
    // Get the correct ground-truth.
    ros::Time time(node.msg_.header.stamp.sec, node.msg_.header.stamp.nsec);
    double key = time.toSec();
    const auto& gt_loc = time_to_gt_.find(key);
    if (gt_loc == time_to_gt_.end()) return -1;
    std::vector<tf::StampedTransform>& gt_human_transforms =
        time_to_gt_.at(key);

    auto& t = node.pose_.translation();
    tf::Vector3 center(t.x(), t.y(), t.z() - human_height_ / 2.0);
    float min_distance = 10000;
    for (auto human : gt_human_transforms) {
      tf::Vector3 diff = center - human.getOrigin();
      float distance = diff.length();
      if (distance < min_distance) {
        min_distance = distance;
      }
    }
    return min_distance;
  }

  /**
   * @brief calcVariance computes the sample variance
   * @param err: The summed error for all samples.
   * @param err_sq: The sum of squared errors for all samples.
   * @param sample_size: The number or samples.
   * @return The variance of the samples.
   */
  inline float calcVariance(float err, float err_sq, int sample_size) {
    float avg_sq = err_sq / sample_size;
    float avg = err / sample_size;
    return (avg_sq - pow(avg, 2)) * (sample_size / (sample_size - 1));
  }

 public:
  std::unordered_map<double, std::vector<tf::StampedTransform>> time_to_gt_;
  kimera::DynamicSceneGraph dynamic_scene_graph_;
  ros::NodeHandle nh_, nh_private_;
  ros::Subscriber human_sub_;
  ros::ServiceServer optimized_centroid_check_srv_;
  tf::TransformListener listener_;

  // ROS Params
  std::string human_topic_ = "humans";
  double human_height_ = 2.1;
  std::string world_frame_ = "world";
  int num_humans_ = 1;

 protected:
  // Record Keeping
  float raw_err_ = 0;
  float raw_sq_err_ = 0;
  float max_raw_err_ = 0;
  int num_meshes_ = 0;
};
