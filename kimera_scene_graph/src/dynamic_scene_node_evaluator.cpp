#include "kimera_scene_graph/dynamic_scene_node_evaluator.h"

DynamicSceneGraphEvaluator::DynamicSceneGraphEvaluator(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& nh_private)
    : dynamic_scene_graph_(nh, nh_private),
      optimized_centroid_check_srv_(),
      human_sub_(),
      nh_(nh),
      nh_private_(nh_private) {
  nh_private_.param("human_topic", human_topic_, human_topic_);
  nh_private_.param("human_height", human_height_, human_height_);
  nh_private_.param("world_frame", world_frame_, world_frame_);
  nh_private_.param("num_humans", num_humans_, num_humans_);
  optimized_centroid_check_srv_ = nh_private_.advertiseService(
      "compute_optimized_error",
      &DynamicSceneGraphEvaluator::errorServiceCall,
      this);
  human_sub_ = nh_private_.subscribe<graph_cmr_ros::SMPLList>(
      human_topic_, 1, &DynamicSceneGraphEvaluator::humanCallback, this);
}

void DynamicSceneGraphEvaluator::humanCallback(
    const graph_cmr_ros::SMPLList::ConstPtr& msg) {
  if (msg->human_meshes.size() > 0) {
    std::vector<tf::StampedTransform> gt_human_transforms;
    ros::Time time(msg->human_meshes[0].header.stamp.sec,
                   msg->human_meshes[0].header.stamp.nsec);

    // Ensure that we could get all transforms before adding them to
    // ground-truth.
    if (!getAllTransforms(gt_human_transforms, time)) {
      ROS_ERROR("ERROR: Could not get all object transforms.");
      return;
    }

    time_to_gt_[time.toSec()] = gt_human_transforms;

    for (auto mesh : msg->human_meshes) {
      kimera::DynamicSceneNode human_node;
      dynamic_scene_graph_.dynamicSceneNodeFromSMPL(mesh, human_node);
      float dist = smallestCentroidDistance(human_node);

      // Record Keeping for raw smpl meshes.
      raw_err_ += dist;
      raw_sq_err_ += dist * dist;
      num_meshes_++;
      if (dist > max_raw_err_) {
        max_raw_err_ = dist;
      }
    }
  }
}

void DynamicSceneGraphEvaluator::computeOptimizedError(
    kimera_scene_graph::CentroidErrorRequest::Response& res) {
  // Compute "Precision" of how close we are to the nearest human.
  float total_err = 0;
  float total_sq_err = 0;
  float max_err = 0;
  size_t count = 0;
  kimera::DynamicNodeList nodes;

  // Compute the statics on the optimized data.
  dynamic_scene_graph_.optimizedGraph(nodes);

  for (size_t i = 0; i < nodes.size(); i++) {
    for (size_t j = 0; j < nodes[i].size(); j++) {
      float error = smallestCentroidDistance(nodes[i][j]);

      // There was an error with getting the ground-truth.
      if (error < 0) continue;

      if (error > max_err) {
        max_err = error;
      }
      total_err += error;
      total_sq_err += pow(error, 2);
      count++;
    }
  }

  res.optimized_error = total_err / count;
  res.optimized_variance = calcVariance(total_err, total_sq_err, count);
  res.max_optimized_error = max_err;
  res.optimized_count = count;
}
