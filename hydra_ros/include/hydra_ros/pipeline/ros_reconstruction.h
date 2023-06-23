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
#pragma once
#include <geometry_msgs/Pose.h>
#include <hydra/reconstruction/reconstruction_module.h>
#include <hydra_msgs/QueryFreespace.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pose_graph_tools/PoseGraph.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include "hydra_ros/config/ros_utilities.h"
#include "hydra_ros/visualizer/topology_server_visualizer.h"

namespace hydra {

struct RosReconstructionConfig {
  bool visualize_reconstruction = true;
  std::string topology_visualizer_ns = "~";
  bool publish_mesh = false;
  bool enable_output_queue = false;
  double pointcloud_separation_s = 0.1;
  double tf_wait_duration_s = 0.1;
  std::string kimera_extrinsics_file = "";
  double tf_buffer_size_s = 30.0;
};

template <typename Visitor>
void visit_config(const Visitor& v, RosReconstructionConfig& config) {
  v.visit("visualize_reconstruction", config.visualize_reconstruction);
  v.visit("topology_visualizer_ns", config.topology_visualizer_ns);
  v.visit("publish_reconstruction_mesh", config.publish_mesh);
  v.visit("enable_reconstruction_output_queue", config.enable_output_queue);
  v.visit("pointcloud_separation_s", config.pointcloud_separation_s);
  v.visit("tf_wait_duration_s", config.tf_wait_duration_s);
  v.visit("kimera_extrinsics_file", config.kimera_extrinsics_file);
  v.visit("tf_buffer_size_s", config.tf_buffer_size_s);
}

}  // namespace hydra

DECLARE_CONFIG_OSTREAM_OPERATOR(hydra, RosReconstructionConfig);

namespace hydra {

using pose_graph_tools::PoseGraph;

class RosReconstruction : public ReconstructionModule {
 public:
  using Pointcloud = pcl::PointCloud<pcl::PointXYZRGB>;
  using PointcloudQueue = InputQueue<Pointcloud::ConstPtr>;

  RosReconstruction(const ros::NodeHandle& nh,
                    const RobotPrefixConfig& prefix,
                    const OutputQueue::Ptr& output_queue = nullptr);

  virtual ~RosReconstruction();

  void handlePointcloud(const Pointcloud::ConstPtr& cloud);

  void handlePoseGraph(const PoseGraph::ConstPtr& pose_graph);

  bool handleFreespaceSrv(hydra_msgs::QueryFreespace::Request& req,
                          hydra_msgs::QueryFreespace::Response& res);

 protected:
  void pointcloudSpin();

  void visualize(const ReconstructionOutput& output);

  ros::NodeHandle nh_;
  RosReconstructionConfig ros_config_;

  ros::Subscriber pcl_sub_;
  ros::Subscriber pose_graph_sub_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  PointcloudQueue pointcloud_queue_;
  std::unique_ptr<std::thread> pointcloud_thread_;
  std::unique_ptr<ros::Time> last_time_received_;
  std::mutex pose_graph_mutex_;
  std::list<PoseGraph::ConstPtr> pose_graphs_;

  // visualizer
  std::unique_ptr<TopologyServerVisualizer> visualizer_;
  ros::Publisher mesh_pub_;

  // freespace query
  ros::ServiceServer freespace_server_;
};

}  // namespace hydra
