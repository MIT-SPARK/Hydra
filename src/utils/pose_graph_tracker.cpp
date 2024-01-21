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
#include "hydra/utils/pose_graph_tracker.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <pose_graph_tools_ros/conversions.h>
#include <tf2_eigen/tf2_eigen.h>

#include "hydra/common/common.h"
#include "hydra/common/dsg_types.h"
#include "hydra/common/hydra_config.h"
#include "hydra/common/shared_module_state.h"
#include "hydra/reconstruction/reconstruction_output.h"

namespace hydra {

using pose_graph_tools_msgs::PoseGraph;

struct StampedPose {
  StampedPose(uint64_t stamp, const Eigen::Isometry3d& pose)
      : stamp(stamp), pose(pose) {}

  explicit StampedPose(const ReconstructionOutput& msg) : stamp(msg.timestamp_ns) {
    pose = Eigen::Isometry3d(Eigen::Translation3d(msg.world_t_body) * msg.world_R_body);
  }

  uint64_t stamp;
  Eigen::Isometry3d pose;
};

void addNode(PoseGraph& graph, const StampedPose& stamped_pose, size_t index) {
  auto& node = graph.nodes.emplace_back();
  node.header.stamp.fromNSec(stamped_pose.stamp);
  node.header.frame_id = graph.header.frame_id;
  node.robot_id = HydraConfig::instance().getRobotPrefix().id;
  node.key = index;
  tf2::convert(stamped_pose.pose, node.pose);
}

void addEdge(PoseGraph& graph, const Eigen::Isometry3d& body_i_T_body_j) {
  auto& edge = graph.edges.emplace_back();
  edge.header.stamp = graph.nodes.back().header.stamp;
  edge.header.frame_id = graph.header.frame_id;

  const auto num_nodes = graph.nodes.size();
  // this will fail if num_nodes is 0 or 1
  const auto& prev_node = graph.nodes.at(num_nodes - 2);
  const auto& curr_node = graph.nodes.at(num_nodes - 1);
  edge.key_from = prev_node.key;
  edge.key_to = curr_node.key;
  edge.robot_from = prev_node.robot_id;
  edge.robot_to = curr_node.robot_id;
  edge.type = pose_graph_tools_msgs::PoseGraphEdge::ODOM;
  tf2::convert(body_i_T_body_j, edge.pose);
}

PoseGraphTracker::PoseGraphTracker(const PoseGraphTracker::Config& config)
    : config_(config::checkValid(config)), num_poses_received_(0) {}

std::optional<size_t> PoseGraphTracker::update(const ReconstructionOutput& msg) {
  const StampedPose curr_pose(msg);
  if (num_poses_received_ > 0) {
    const auto diff_m = (curr_pose.pose.translation() - prev_pose_.translation()).norm();
    if (diff_m < config_.min_separation_m) {
      return std::nullopt;
    }
  }

  const auto frame_id = HydraConfig::instance().getFrames().odom;
  graph_.header.frame_id = frame_id;
  graph_.header.stamp.fromNSec(msg.timestamp_ns);
  addNode(graph_, curr_pose, num_poses_received_);

  if (num_poses_received_ > 0) {
    Eigen::Isometry3d body_i_T_body_j = curr_pose.pose.inverse() * prev_pose_;
    addEdge(graph_, body_i_T_body_j);
  }

  prev_pose_ = curr_pose.pose;
  prev_time_ = msg.timestamp_ns;
  const auto curr_idx = num_poses_received_;
  ++num_poses_received_;
  return curr_idx;
}

void PoseGraphTracker::fillPoseGraphs(BackendInput& msg,
                                      const std::vector<NodeId>& new_nodes) const {
  VLOG(VLEVEL_DEBUG) << "[PoseGraph Tracker] pose graph size: " << graph_.nodes.size();

  for (const auto node_id : new_nodes) {
    const NodeSymbol node_key(node_id);
    const auto idx = node_key.categoryId();
    if (idx == 0) {
      // no edge exists before pose 0
      continue;
    }

    const auto prev_idx = idx - 1;
    pose_graph_tools_msgs::PoseGraph::Ptr new_graph;
    new_graph.reset(new pose_graph_tools_msgs::PoseGraph());
    new_graph->nodes.push_back(graph_.nodes.at(prev_idx));
    new_graph->nodes.push_back(graph_.nodes.at(idx));
    // can treat edges as 0-index to node 1-index
    new_graph->edges.push_back(graph_.edges.at(prev_idx));
    msg.pose_graphs.push_back(new_graph);
  }
}

// TODO(nathan) this may get expanded with covariances...
void declare_config(PoseGraphTracker::Config& conf) {
  config::name("PoseGraphTracker::Config");
  config::field(conf.make_pose_graph, "make_pose_graph");
  config::field(conf.min_separation_m, "min_separation_m");
}

std::vector<NodeId> PoseGraphTracker::addAgentNodes(
    DynamicSceneGraph& graph, const RobotPrefixConfig& prefix) const {
  const auto& agents = graph.getLayer(DsgLayers::AGENTS, prefix.key);
  const auto curr_num_nodes = agents.numNodes();

  std::vector<NodeId> new_nodes;
  for (size_t i = curr_num_nodes; i < graph_.nodes.size(); ++i) {
    const auto& node = graph_.nodes[i];
    CHECK_EQ(i, node.key);

    Eigen::Vector3d position;
    tf2::convert(node.pose.position, position);
    Eigen::Quaterniond rotation;
    tf2::convert(node.pose.orientation, rotation);

    NodeSymbol pgmo_key(prefix.key, node.key);

    const std::chrono::nanoseconds stamp(node.header.stamp.toNSec());
    auto attrs = std::make_unique<AgentNodeAttributes>(rotation, position, pgmo_key);
    if (!graph.emplaceNode(agents.id, agents.prefix, stamp, std::move(attrs))) {
      VLOG(VLEVEL_TRACE) << "[Hydra Frontend] repeated timestamp " << stamp.count()
                         << "[ns] found";
      continue;
    }

    new_nodes.push_back(pgmo_key);
  }

  return new_nodes;
}

}  // namespace hydra
