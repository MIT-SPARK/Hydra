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
#include "hydra_ros/pipeline/ros_reconstruction.h"

#include <geometry_msgs/TransformStamped.h>
#include <hydra/places/gvd_integrator.h>
#include <hydra_msgs/ActiveLayer.h>
#include <hydra_msgs/ActiveMesh.h>
#include <hydra_msgs/QueryFreespace.h>
#include <tf2_eigen/tf2_eigen.h>
#include <voxblox_msgs/Mesh.h>
#include <voxblox_ros/conversions.h>
#include <voxblox_ros/mesh_vis.h>

#include "hydra_ros/config/ros_utilities.h"

namespace hydra {

DECLARE_STRUCT_NAME(RosReconstructionConfig);
DECLARE_STRUCT_NAME(ReconstructionConfig);

using pose_graph_tools::PoseGraph;
using pose_graph_tools::PoseGraphEdge;
using pose_graph_tools::PoseGraphNode;
using RosPointcloud = RosReconstruction::Pointcloud;
using places::CompressionGraphExtractor;

inline geometry_msgs::Pose tfToPose(const geometry_msgs::Transform& transform) {
  geometry_msgs::Pose pose;
  pose.position.x = transform.translation.x;
  pose.position.y = transform.translation.y;
  pose.position.z = transform.translation.z;
  pose.orientation = transform.rotation;
  return pose;
}

std::string showQuaternion(const Eigen::Quaterniond& q) {
  std::stringstream ss;
  ss << "{w: " << q.w() << ", x: " << q.x() << ", y: " << q.y() << ", z: " << q.z()
     << "}";
  return ss.str();
}

bool RosReconstruction::handleFreespaceSrv(hydra_msgs::QueryFreespace::Request& req,
                                           hydra_msgs::QueryFreespace::Response& res) {
  if (req.x.size() != req.y.size() || req.x.size() != req.z.size()) {
    return false;
  }

  if (req.x.empty()) {
    return true;
  }

  ReconstructionModule::PositionMatrix points(3, req.x.size());
  for (size_t i = 0; i < req.x.size(); ++i) {
    points(0, i) = req.x[i];
    points(1, i) = req.y[i];
    points(2, i) = req.z[i];
  }

  const auto result = inFreespace(points, req.freespace_distance_m);
  for (const auto flag : result) {
    res.in_freespace.push_back(flag ? 1 : 0);
  }
  return true;
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

  buffer_.reset(new tf2_ros::Buffer(ros::Duration(ros_config_.tf_buffer_size_s)));

  if (!ros_config_.kimera_extrinsics_file.empty()) {
    const auto node = YAML::LoadFile(ros_config_.kimera_extrinsics_file);
    const auto elements = node["T_BS"]["data"].as<std::vector<double>>();
    CHECK_EQ(elements.size(), 16u) << "Invalid transform!";
    Eigen::Matrix4d body_T_camera;
    for (size_t r = 0; r < 4; r++) {
      for (size_t c = 0; c < 4; c++) {
        body_T_camera(r, c) = elements.at(4 * r + c);
      }
    }
    config_.body_R_camera = Eigen::Quaterniond(body_T_camera.block<3, 3>(0, 0));
    config_.body_t_camera = body_T_camera.block<3, 1>(0, 3);
    LOG(WARNING) << "Loaded extrinsics from Kimera: R="
                 << showQuaternion(config_.body_R_camera)
                 << ", t=" << config_.body_t_camera.transpose();
  }

  LOG(WARNING) << "Using pointcloud and TF as input!";
  pcl_sub_ = nh_.subscribe<Pointcloud>(
      "pointcloud", 10, &RosReconstruction::handlePointcloud, this);
  pose_graph_sub_ =
      nh_.subscribe("pose_graph", 1000, &RosReconstruction::handlePoseGraph, this);
  tf_listener_.reset(new tf2_ros::TransformListener(*buffer_));
  pointcloud_thread_.reset(new std::thread(&RosReconstruction::pointcloudSpin, this));

  if (!ros_config_.enable_output_queue && !output_queue) {
    // reset output queue so we don't waste memory with queued packets
    output_queue_.reset();
  }

  if (ros_config_.visualize_reconstruction) {
    visualizer_.reset(new TopologyServerVisualizer(ros_config_.topology_visualizer_ns));
  }

  if (ros_config_.publish_mesh) {
    mesh_pub_ = nh_.advertise<voxblox_msgs::Mesh>("mesh", 10);
  }

  freespace_server_ = nh_.advertiseService(
      "query_freespace", &RosReconstruction::handleFreespaceSrv, this);

  output_callbacks_.push_back(
      [this](const ReconstructionModule&, const ReconstructionOutput& output) {
        this->visualize(output);
      });
}

RosReconstruction::~RosReconstruction() {
  stop();

  if (pointcloud_thread_) {
    VLOG(2) << "[Hydra Reconstruction] stopping pointcloud input thread";
    pointcloud_thread_->join();
    pointcloud_thread_.reset();
    VLOG(2) << "[Hydra Reconstruction] stopped pointcloud input thread";
  }

  VLOG(2) << "[Hydra Reconstruction] pointcloud queue: " << pointcloud_queue_.size();
}

void RosReconstruction::handlePointcloud(const RosPointcloud::ConstPtr& cloud) {
  // pcl timestamps are in microseconds
  ros::Time curr_time;
  curr_time.fromNSec(cloud->header.stamp * 1000);

  VLOG(1) << "[Hydra Reconstruction] Got raw pointcloud input @ " << curr_time.toNSec()
          << " [ns]";
  if (num_poses_received_ > 0) {
    if (last_time_received_ && ((curr_time - *last_time_received_).toSec() <
                                ros_config_.pointcloud_separation_s)) {
      return;
    }

    last_time_received_.reset(new ros::Time(curr_time));
  }

  VLOG(1) << "[Hydra Reconstruction] Got ROS input @ " << curr_time.toNSec() << " [ns]";

  // TODO(nathan) consider setting prev_time_ here?
  pointcloud_queue_.push(cloud);
}

void RosReconstruction::handlePoseGraph(const PoseGraph::ConstPtr& pose_graph) {
  if (pose_graph->nodes.empty()) {
    ROS_ERROR("Received empty pose graph!");
    return;
  }

  std::unique_lock<std::mutex> lock(pose_graph_mutex_);
  pose_graphs_.push_back(pose_graph);
}

void RosReconstruction::pointcloudSpin() {
  while (!should_shutdown_) {
    bool has_data = pointcloud_queue_.poll();
    if (!has_data) {
      continue;
    }

    const auto cloud = pointcloud_queue_.pop();

    ros::Time curr_time;
    curr_time.fromNSec(cloud->header.stamp * 1000);

    VLOG(1) << "[Hydra Reconstruction] popped pointcloud input @ " << curr_time.toNSec()
            << " [ns]";

    ros::WallRate tf_wait_rate(1.0 / ros_config_.tf_wait_duration_s);

    // note that this is okay in a separate thread from the callback queue because tf2
    // is threadsafe
    bool have_transform = false;
    std::string err_str;
    for (size_t i = 0; i < 5; ++i) {
      if (buffer_->canTransform(config_.world_frame,
                                config_.robot_frame,
                                curr_time,
                                ros::Duration(0),
                                &err_str)) {
        have_transform = true;
        break;
      }

      if (should_shutdown_) {
        return;
      }

      tf_wait_rate.sleep();
    }

    if (!have_transform) {
      ROS_WARN_STREAM("Failed to get tf from "
                      << config_.robot_frame << " to " << config_.world_frame << " @ "
                      << curr_time.toNSec() << " [ns]. Reason: " << err_str);
      continue;
    }

    geometry_msgs::TransformStamped transform;
    try {
      transform =
          buffer_->lookupTransform(config_.world_frame, config_.robot_frame, curr_time);
    } catch (const tf2::TransformException& ex) {
      LOG(ERROR) << "Failed to look up: " << config_.world_frame << " to "
                 << config_.robot_frame;
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

    {  // start pose graph critical section
      std::unique_lock<std::mutex> lock(pose_graph_mutex_);
      input->pose_graphs = pose_graphs_;
      pose_graphs_.clear();
    }  // end pose graph critical section

    queue_->push(input);
  }
}

void RosReconstruction::visualize(const ReconstructionOutput& output) {
  if (ros_config_.publish_mesh && output.mesh) {
    hydra_msgs::ActiveMesh::ConstPtr msg(new hydra_msgs::ActiveMesh());
    auto mesh = const_cast<hydra_msgs::ActiveMesh&>(*msg);
    mesh_pub_.publish(msg);
  }

  if (visualizer_) {
    visualizer_->visualize(gvd_integrator_->getGraph(),
                           gvd_integrator_->getGvdGraph(),
                           *gvd_,
                           *tsdf_,
                           output.timestamp_ns,
                           mesh_.get());
    if (config_.gvd.graph_extractor.use_compression_extractor) {
      visualizer_->visualizeExtractor(output.timestamp_ns,
                                      dynamic_cast<CompressionGraphExtractor&>(
                                          gvd_integrator_->getGraphExtractor()));
    }
  }
}

}  // namespace hydra
