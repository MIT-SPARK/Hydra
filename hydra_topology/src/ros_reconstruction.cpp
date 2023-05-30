/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra_topology/ros_reconstruction.h"

#include <geometry_msgs/TransformStamped.h>
#include <hydra_utils/ros_utilities.h>
#include <tf2_eigen/tf2_eigen.h>
#include <voxblox_ros/conversions.h>

namespace hydra {

using message_filters::Subscriber;
using pose_graph_tools::PoseGraph;
using pose_graph_tools::PoseGraphEdge;
using pose_graph_tools::PoseGraphNode;
using RosPointcloud = RosReconstruction::Pointcloud;

inline geometry_msgs::Pose tfToPose(const geometry_msgs::Transform& transform) {
  geometry_msgs::Pose pose;
  pose.position.x = transform.translation.x;
  pose.position.y = transform.translation.y;
  pose.position.z = transform.translation.z;
  pose.orientation = transform.rotation;
  return pose;
}

RosReconstruction::RosReconstruction(const ros::NodeHandle& nh,
                                     const RobotPrefixConfig& prefix,
                                     const OutputQueue::Ptr& output_queue)
    : ReconstructionModule(
          prefix,
          load_config<ReconstructionConfig>(nh),
          output_queue ? output_queue : std::make_shared<OutputQueue>()),
      nh_(nh) {
  ros_config_ = load_config<RosReconstructionConfig>(nh);
  if (ros_config_.use_pose_graph) {
    pcl_sync_sub_.reset(new Subscriber<RosPointcloud>(nh_, "pointcloud", 5));
    pose_graph_sub_.reset(new Subscriber<PoseGraph>(nh_, "pose_graph", 5));
    sync_.reset(new Sync(Policy(10), *pcl_sync_sub_, *pose_graph_sub_));
    sync_->registerCallback(
        boost::bind(&RosReconstruction::inputCallback, this, _1, _2));
  } else {
    pcl_sub_ = nh_.subscribe<Pointcloud>(
        "pointcloud", 5, &RosReconstruction::pclCallback, this);
    tf_listener_.reset(new tf2_ros::TransformListener(buffer_));
  }

  if (!ros_config_.enable_output_queue && !output_queue) {
    // reset output queue so we don't waste memory with queued packets
    output_queue_.reset();
  }

  if (ros_config_.visualize_reconstruction) {
    visualizer_.reset(new topology::TopologyServerVisualizer("~"));
  }

  if (ros_config_.publish_mesh) {
    mesh_pub_ = nh_.advertise<voxblox_msgs::Mesh>("mesh", 10);
  }

  output_callbacks_.push_back(
      [this](const ReconstructionModule&, const ReconstructionOutput& output) {
        this->visualize(output);
      });
}

void RosReconstruction::inputCallback(const RosPointcloud::ConstPtr& cloud,
                                      const PoseGraph::ConstPtr& pose_graph) {
  if (pose_graph->nodes.empty()) {
    ROS_ERROR("Received empty pose graph!");
    return;
  }

  ReconstructionInput::Ptr input(new ReconstructionInput());
  input->timestamp_ns = cloud->header.stamp;
  input->pose_graph = pose_graph;

  input->pointcloud.reset(new voxblox::Pointcloud());
  input->pointcloud_colors.reset(new voxblox::Colors());
  voxblox::convertPointcloud(
      *cloud, nullptr, input->pointcloud.get(), input->pointcloud_colors.get());

  // TODO(nathan) this assumes that the pose graph message time stamps are
  // well formed (i.e. the header stamp matches the last pose stamp)
  tf2::convert(pose_graph->nodes.back().pose.position, input->world_t_body);
  tf2::convert(pose_graph->nodes.back().pose.orientation, input->world_R_body);

  queue_->push(input);
}

void RosReconstruction::pclCallback(const RosPointcloud::ConstPtr& cloud) {
  ros::Time curr_time;
  // pcl timestamps are in microseconds
  curr_time.fromNSec(cloud->header.stamp * 1000);
  if (num_poses_received_ > 0) {
    ros::Time prev_time;
    prev_time.fromNSec(prev_time_);
    if ((curr_time - prev_time).toSec() < ros_config_.pointcloud_separation_s) {
      return;
    }
  }

  // TODO(nathan) make this lookup better
  geometry_msgs::TransformStamped transform;
  try {
    transform =
        buffer_.lookupTransform(config_.world_frame, config_.robot_frame, curr_time);
  } catch (const tf2::TransformException& ex) {
    ROS_WARN_STREAM(ex.what());
    return;
  }

  ReconstructionInput::Ptr input(new ReconstructionInput());
  input->timestamp_ns = curr_time.toNSec();

  input->pointcloud.reset(new voxblox::Pointcloud());
  input->pointcloud_colors.reset(new voxblox::Colors());
  voxblox::convertPointcloud(
      *cloud, nullptr, input->pointcloud.get(), input->pointcloud_colors.get());

  geometry_msgs::Pose curr_pose = tfToPose(transform.transform);
  tf2::convert(curr_pose.position, input->world_t_body);
  tf2::convert(curr_pose.orientation, input->world_R_body);

  queue_->push(input);
}

void RosReconstruction::visualize(const ReconstructionOutput& output) {
  if (ros_config_.publish_mesh && output.mesh) {
    mesh_pub_.publish(output.mesh->mesh);
  }

  if (visualizer_) {
    visualizer_->visualize(gvd_integrator_->getGraphExtractor(),
                           gvd_integrator_->getGraph(),
                           *gvd_,
                           *tsdf_);
  }
}

}  // namespace hydra
