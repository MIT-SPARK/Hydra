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
#include "hydra_ros/backend/ros_backend_publisher.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/context.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <hydra/common/common_types.h>
#include <hydra/common/global_info.h>
#include <ianvs/node_handle.h>
#include <kimera_pgmo_ros/visualization_functions.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "hydra_ros/utils/dsg_streaming_interface.h"

namespace hydra {

using kimera_pgmo::DeformationGraph;
using kimera_pgmo::KimeraPgmoConfig;
using kimera_pgmo_msgs::msg::Mesh;
using pose_graph_tools::PoseGraphTypeAdapter;
using visualization_msgs::msg::Marker;

namespace {

inline RosBackendPublisher::Config get_config() {
  const auto map_frame = GlobalInfo::instance().getFrames().map;
  auto config = config::fromContext<RosBackendPublisher::Config>("backend");
  config.dsg_sender = config.dsg_sender.with_name("backend").with_frame(map_frame);
  return config;
}

}  // namespace

void declare_config(RosBackendPublisher::Config& config) {
  using namespace config;
  name("RosBackendPublisher::Config");
  field(config.dsg_sender, "");
  field(config.publish_backend_tf, "publish_backend_tf");
  field(config.tf_pub_robot_frame, "tf_pub_robot_frame");
  field(config.tf_pub_map_frame, "tf_pub_map_frame");
  field(config.tf_pub_odom_frame, "tf_pub_odom_frame");
}

RosBackendPublisher::RosBackendPublisher(ianvs::NodeHandle nh)
    : config(config::checkValid(get_config())), nh_(nh), tf_br_(nh_.node()) {
  mesh_mesh_edges_pub_ = nh.create_publisher<Marker>("deformation_graph_mesh_mesh", 10);
  pose_mesh_edges_pub_ = nh.create_publisher<Marker>("deformation_graph_pose_mesh", 10);
  pose_graph_pub_ = nh.create_publisher<PoseGraphTypeAdapter>("pose_graph", 10);
  mesh_graph_pub_ = nh.create_publisher<PoseGraphTypeAdapter>("mesh_graph", 10);
  dsg_sender_ = std::make_unique<DsgSender>(config.dsg_sender, nh);
}

void RosBackendPublisher::call(uint64_t timestamp_ns,
                               const DynamicSceneGraph& graph,
                               const DeformationGraph& dgraph) const {
  const rclcpp::Time stamp(timestamp_ns);
  dsg_sender_->sendGraph(graph, stamp);

  if (pose_graph_pub_->get_subscription_count()) {
    publishPoseGraph(graph, dgraph, timestamp_ns);
  }

  if (mesh_graph_pub_->get_subscription_count()) {
    publishMeshGraph(graph, dgraph, timestamp_ns);
  }

  if (mesh_mesh_edges_pub_->get_subscription_count() ||
      pose_mesh_edges_pub_->get_subscription_count()) {
    publishDeformationGraphViz(dgraph, timestamp_ns);
  }

  if (config.publish_backend_tf) {
    publishTf(graph, dgraph);
  }
}

std::string RosBackendPublisher::printInfo() const { return config::toString(config); }

void RosBackendPublisher::publishTf(const DynamicSceneGraph& graph,
                                    const DeformationGraph& dgraph) const {
  const auto& prefix = GlobalInfo::instance().getRobotPrefix();

  const auto& frames = GlobalInfo::instance().getFrames();
  std::string map_frame = config.tf_pub_map_frame;
  if (map_frame == "") {
    map_frame = frames.map;
  }

  std::string odom_frame = config.tf_pub_odom_frame;
  if (odom_frame == "") {
    odom_frame = frames.odom;
  }

  const auto layer_id = graph.getLayerKey(DsgLayers::AGENTS).value().layer;
  const auto agents = graph.findLayer(layer_id, prefix.key);

  gtsam::Pose3 map_T_odom;
  gtsam::Pose3 map_T_body;

  if (agents && agents->numNodes()) {
    NodeSymbol pgmo_key(prefix.key, agents->numNodes() - 1);
    const auto& attrs = graph.getNode(pgmo_key).attributes<AgentNodeAttributes>();

    const auto& odom_T_body = dgraph.getInitialPose(prefix.key, pgmo_key.categoryId());
    map_T_body = gtsam::Pose3(gtsam::Rot3(attrs.world_R_body), attrs.position);
    map_T_odom = map_T_body * odom_T_body.inverse();
  }

  const auto curr_time = nh_.now();
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  // Build map->odom transform message
  auto& tf = transforms.emplace_back();
  tf.header.frame_id = map_frame;
  tf.header.stamp = curr_time;
  tf.child_frame_id = odom_frame;
  tf2::convert(map_T_odom.rotation().toQuaternion(), tf.transform.rotation);
  tf2::toMsg(map_T_odom.translation(), tf.transform.translation);

  if (config.tf_pub_robot_frame != "") {
    // Build map->robot transform message
    auto& tf_robot = transforms.emplace_back();
    tf_robot.header.stamp = curr_time;
    tf_robot.header.frame_id = map_frame;
    tf_robot.child_frame_id = config.tf_pub_robot_frame;
    tf2::convert(map_T_body.rotation().toQuaternion(), tf_robot.transform.rotation);
    tf2::toMsg(map_T_body.translation(), tf_robot.transform.translation);
  }

  tf_br_.sendTransform(transforms);
}

void RosBackendPublisher::publishPoseGraph(const DynamicSceneGraph& graph,
                                           const DeformationGraph& dgraph,
                                           const uint64_t& stamp) const {
  const auto& prefix = GlobalInfo::instance().getRobotPrefix();
  const auto layer_id = graph.getLayerKey(DsgLayers::AGENTS).value().layer;
  const auto agent = graph.findLayer(layer_id, prefix.key);
  if (!agent) {
    return;
  }

  std::map<size_t, std::vector<size_t>> id_timestamps;
  id_timestamps[prefix.id] = std::vector<size_t>();
  auto& times = id_timestamps[prefix.id];
  for (const auto& [node_id, node] : agent->nodes()) {
    times.push_back(node->attributes<AgentNodeAttributes>().timestamp.count());
  }

  auto pose_graph = *dgraph.getPoseGraph(id_timestamps, false, true);
  pose_graph.stamp_ns = stamp;
  pose_graph.frame_id = GlobalInfo::instance().getFrames().map;
  pose_graph_pub_->publish(pose_graph);
}

void RosBackendPublisher::publishMeshGraph(const DynamicSceneGraph&,
                                           const DeformationGraph& dgraph,
                                           const uint64_t& stamp) const {
  std::map<size_t, std::vector<size_t>> id_timestamps_temp;
  auto mesh_graph = *dgraph.getPoseGraph(id_timestamps_temp, true, false);
  mesh_graph.stamp_ns = stamp;
  mesh_graph.frame_id = GlobalInfo::instance().getFrames().map;
  mesh_graph_pub_->publish(mesh_graph);
}

void RosBackendPublisher::publishDeformationGraphViz(const DeformationGraph& dgraph,
                                                     size_t timestamp_ns) const {
  const rclcpp::Time stamp(timestamp_ns);

  Marker mm_edges_msg;
  Marker pm_edges_msg;
  kimera_pgmo::fillDeformationGraphMarkers(dgraph,
                                           stamp,
                                           mm_edges_msg,
                                           pm_edges_msg,
                                           GlobalInfo::instance().getFrames().map);

  if (!mm_edges_msg.points.empty()) {
    mesh_mesh_edges_pub_->publish(mm_edges_msg);
  }
  if (!pm_edges_msg.points.empty()) {
    pose_mesh_edges_pub_->publish(pm_edges_msg);
  }
}

}  // namespace hydra
