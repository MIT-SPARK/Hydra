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
#include "hydra/odometry/pose_graph_from_odom.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/validation.h>

#include "hydra/common/global_info.h"

namespace hydra {
namespace {

static const auto registration =
    config::RegistrationWithConfig<PoseGraphTracker,
                                   PoseGraphFromOdom,
                                   PoseGraphFromOdom::Config>("PoseGraphFromOdom");

}

using pose_graph_tools::PoseGraph;

void addNode(PoseGraph& graph, const StampedPose& stamped_pose, size_t index) {
  auto& node = graph.nodes.emplace_back();
  node.stamp_ns = stamped_pose.stamp.count();
  node.robot_id = GlobalInfo::instance().getRobotPrefix().id;
  node.key = index;
  node.pose = stamped_pose.pose;
}

void addEdge(PoseGraph& graph, const Eigen::Isometry3d& prev_T_curr) {
  auto& edge = graph.edges.emplace_back();
  edge.stamp_ns = graph.nodes.back().stamp_ns;

  const auto& prev_node = graph.nodes.front();
  const auto& curr_node = graph.nodes.back();
  edge.key_from = prev_node.key;
  edge.key_to = curr_node.key;
  edge.robot_from = prev_node.robot_id;
  edge.robot_to = curr_node.robot_id;
  edge.type = pose_graph_tools::PoseGraphEdge::ODOM;
  // convention is always from_T_to in gtsam
  edge.pose = prev_T_curr;
}

PoseGraph makePoseGraph(const StampedPose& curr_pose,
                        const StampedPose& prev_pose,
                        size_t prev_index) {
  PoseGraph graph;
  graph.stamp_ns = curr_pose.stamp.count();

  addNode(graph, prev_pose, prev_index);
  addNode(graph, curr_pose, prev_index + 1);

  Eigen::Isometry3d prev_T_curr = prev_pose.pose.inverse() * curr_pose.pose;
  addEdge(graph, prev_T_curr);
  return graph;
}

PoseGraphFromOdom::PoseGraphFromOdom(const PoseGraphFromOdom::Config& config)
    : config(config::checkValid(config)), num_poses_received_(0) {}

PoseGraphPacket PoseGraphFromOdom::update(uint64_t timestamp_ns,
                                          const Eigen::Isometry3d& world_T_body) {
  PoseGraphPacket packet;
  const StampedPose curr_pose{std::chrono::nanoseconds(timestamp_ns), world_T_body};

  if (num_poses_received_ > 0) {
    const auto diff_ns = curr_pose.stamp - prev_pose_.stamp;
    const auto diff_s =
        std::chrono::duration_cast<std::chrono::duration<double>>(diff_ns);
    if (config.min_time_separation_s && diff_s.count() < config.min_time_separation_s) {
      VLOG(2) << "[PoseGraphFromOdom] Dropped pose @ " << timestamp_ns
              << "[ns] with time separation " << diff_s.count() << " < "
              << config.min_time_separation_s << " [s]";
      return packet;
    }

    const Eigen::Isometry3d pose_diff = curr_pose.pose.inverse() * prev_pose_.pose;
    const auto translation_diff = pose_diff.translation().norm();
    const auto rotation_diff = pose_diff.rotation().norm();
    const auto total_diff =
        translation_diff + config.rotation_separation_weight * rotation_diff;
    if (config.min_pose_separation && total_diff < config.min_pose_separation) {
      VLOG(2) << "[PoseGraphFromOdom] Dropped pose @ " << timestamp_ns
              << "[ns] with pose separation " << total_diff << " < "
              << config.min_pose_separation;
      return packet;
    }

    const auto index = num_poses_received_ - 1;
    packet.pose_graphs.push_back(makePoseGraph(curr_pose, prev_pose_, index));
  }

  prev_pose_ = curr_pose;
  ++num_poses_received_;
  return packet;
}

// TODO(nathan) this may get expanded with covariances...
void declare_config(PoseGraphFromOdom::Config& config) {
  using namespace config;
  name("PoseGraphFromOdom::Config");
  field(config.min_pose_separation, "min_pose_separation");
  field(config.rotation_separation_weight, "rotation_separation_weight");
  field(config.min_time_separation_s, "min_time_separation_s");
  check(config.min_pose_separation, GE, 0.0, "rotation_separation_weight");
  check(config.rotation_separation_weight, GE, 0.0, "rotation_separation_weight");
  check(config.min_time_separation_s, GE, 0.0, "rotation_separation_weight");
}

}  // namespace hydra
