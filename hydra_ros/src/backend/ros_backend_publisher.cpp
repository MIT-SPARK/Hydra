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
#include <hydra/common/global_info.h>
#include <ianvs/node_handle.h>
#include <kimera_pgmo_ros/visualization_functions.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "hydra_ros/utils/dsg_streaming_interface.h"

namespace hydra {

using namespace spark_dsg;
using kimera_pgmo::DeformationGraph;
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
  field(config.tf_robot_frame, "tf_robot_frame");
  field(config.tf_map_frame, "tf_map_frame");
  field(config.tf_odom_frame, "tf_odom_frame");
}

RosBackendPublisher::RosBackendPublisher(ianvs::NodeHandle nh)
    : config(config::checkValid(get_config())),
      nh_(nh),
      dsg_sender_(config.dsg_sender, nh),
      pose_graph_pub_(nh.create_publisher<PoseGraphTypeAdapter>("pose_graph", 10)),
      mesh_graph_pub_(nh.create_publisher<PoseGraphTypeAdapter>("mesh_graph", 10)),
      mesh_mesh_viz_(nh.create_publisher<Marker>("mesh_mesh_viz", 10)),
      pose_mesh_viz_(nh.create_publisher<Marker>("pose_mesh_viz", 10)),
      tf_broadcaster_(nh_.node()) {}

void RosBackendPublisher::call(uint64_t timestamp_ns,
                               const SceneGraph& graph,
                               const DeformationGraph& dgraph) const {
  dsg_sender_.sendGraph(graph, rclcpp::Time(timestamp_ns));
  publishPoseGraph(timestamp_ns, dgraph);
  publishMeshGraph(timestamp_ns, dgraph);
  visualizeDeformation(timestamp_ns, dgraph);
  if (config.publish_backend_tf) {
    publishTf(graph, dgraph);
  }
}

std::string RosBackendPublisher::printInfo() const { return config::toString(config); }

void RosBackendPublisher::publishPoseGraph(const uint64_t stamp,
                                           const DeformationGraph& dgraph) const {
  if (!pose_graph_pub_->get_subscription_count()) {
    return;
  }

  auto pose_graph = *dgraph.getPoseGraph(false, true);
  pose_graph.stamp_ns = stamp;
  pose_graph.frame_id = GlobalInfo::instance().getFrames().map;
  pose_graph_pub_->publish(pose_graph);
}

void RosBackendPublisher::publishMeshGraph(const uint64_t stamp,
                                           const DeformationGraph& dgraph) const {
  if (!mesh_graph_pub_->get_subscription_count()) {
    return;
  }

  auto mesh_graph = *dgraph.getPoseGraph(true, false);
  mesh_graph.stamp_ns = stamp;
  mesh_graph.frame_id = GlobalInfo::instance().getFrames().map;
  mesh_graph_pub_->publish(mesh_graph);
}

void RosBackendPublisher::visualizeDeformation(const uint64_t stamp,
                                               const DeformationGraph& dgraph) const {
  if (!mesh_mesh_viz_->get_subscription_count() &&
      !pose_mesh_viz_->get_subscription_count()) {
    return;
  }

  Marker mm_edges_msg;
  Marker pm_edges_msg;
  kimera_pgmo::fillDeformationGraphMarkers(dgraph,
                                           rclcpp::Time(stamp),
                                           mm_edges_msg,
                                           pm_edges_msg,
                                           GlobalInfo::instance().getFrames().map);

  if (!mm_edges_msg.points.empty()) {
    mesh_mesh_viz_->publish(mm_edges_msg);
  }

  if (!pm_edges_msg.points.empty()) {
    pose_mesh_viz_->publish(pm_edges_msg);
  }
}

void RosBackendPublisher::publishTf(const SceneGraph& graph,
                                    const DeformationGraph& dgraph) const {
  const auto& frames = GlobalInfo::instance().getFrames();
  const auto map = config.tf_map_frame.empty() ? frames.map : config.tf_map_frame;
  const auto odom = config.tf_odom_frame.empty() ? frames.odom : config.tf_odom_frame;

  const auto prefix = GlobalInfo::instance().getRobotPrefix();
  const auto layer_id = graph.getLayerKey(DsgLayers::AGENTS).value().layer;
  const auto agents = graph.findLayer(layer_id, prefix.key);

  gtsam::Pose3 map_T_body;
  gtsam::Pose3 map_T_odom;
  if (agents && agents->numNodes()) {
    NodeSymbol pgmo_key(prefix.key, agents->numNodes() - 1);
    const auto& attrs = graph.getNode(pgmo_key).attributes<AgentNodeAttributes>();
    const auto odom_T_body = dgraph.getInitialPose(prefix.key, pgmo_key.categoryId());
    map_T_body = gtsam::Pose3(gtsam::Rot3(attrs.world_R_body), attrs.position);
    map_T_odom = map_T_body * odom_T_body.inverse();
  }

  const auto curr_time = nh_.now();
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  // Build map->odom transform message
  auto& tf = transforms.emplace_back();
  tf.header.frame_id = map;
  tf.header.stamp = curr_time;
  tf.child_frame_id = odom;
  tf2::convert(map_T_odom.rotation().toQuaternion(), tf.transform.rotation);
  tf2::toMsg(map_T_odom.translation(), tf.transform.translation);

  if (!config.tf_robot_frame.empty()) {
    auto& tf_robot = transforms.emplace_back();
    tf_robot.header.stamp = curr_time;
    tf_robot.header.frame_id = map;
    tf_robot.child_frame_id = config.tf_robot_frame;
    tf2::convert(map_T_body.rotation().toQuaternion(), tf_robot.transform.rotation);
    tf2::toMsg(map_T_body.translation(), tf_robot.transform.translation);
  }

  tf_broadcaster_.sendTransform(transforms);
}

}  // namespace hydra
