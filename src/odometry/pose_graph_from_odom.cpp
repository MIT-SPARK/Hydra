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
#include <config_utilities/validation.h>

#include "hydra/common/global_info.h"

namespace hydra {

using pose_graph_tools::PoseGraph;

void addNode(PoseGraph& graph, const StampedPose& stamped_pose, size_t index) {
  auto& node = graph.nodes.emplace_back();
  node.stamp_ns = stamped_pose.stamp;
  node.robot_id = GlobalInfo::instance().getRobotPrefix().id;
  node.key = index;
  node.pose = stamped_pose.pose;
}

void addEdge(PoseGraph& graph, const Eigen::Isometry3d& body_i_T_body_j) {
  auto& edge = graph.edges.emplace_back();
  edge.stamp_ns = graph.nodes.back().stamp_ns;

  const auto& prev_node = graph.nodes.front();
  const auto& curr_node = graph.nodes.back();
  edge.key_from = prev_node.key;
  edge.key_to = curr_node.key;
  edge.robot_from = prev_node.robot_id;
  edge.robot_to = curr_node.robot_id;
  edge.type = pose_graph_tools::PoseGraphEdge::ODOM;
  edge.pose = body_i_T_body_j;
}

PoseGraph::Ptr makePoseGraph(const StampedPose& curr_pose,
                             const StampedPose& prev_pose,
                             size_t prev_index) {
  auto graph = std::make_shared<PoseGraph>();
  graph->stamp_ns = curr_pose.stamp;

  addNode(*graph, prev_pose, prev_index);
  addNode(*graph, curr_pose, prev_index + 1);

  Eigen::Isometry3d body_i_T_body_j = curr_pose.pose.inverse() * prev_pose.pose;
  addEdge(*graph, body_i_T_body_j);
  return graph;
}

PoseGraphFromOdom::PoseGraphFromOdom(const PoseGraphFromOdom::Config& config)
    : config(config::checkValid(config)), num_poses_received_(0) {}

PoseGraphPacket PoseGraphFromOdom::update(uint64_t timestamp_ns,
                                          const Eigen::Isometry3d& world_T_body) {
  PoseGraphPacket packet;
  const StampedPose curr_pose{timestamp_ns, world_T_body};

  if (num_poses_received_ > 0) {
    const auto index = num_poses_received_ - 1;
    packet.pose_graphs.push_back(makePoseGraph(curr_pose, prev_pose_, index));
  }

  prev_pose_ = curr_pose;
  ++num_poses_received_;
  return packet;
}

// TODO(nathan) this may get expanded with covariances...
void declare_config(PoseGraphFromOdom::Config&) {
  config::name("PoseGraphFromOdom::Config");
}

}  // namespace hydra
