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

#include "hydra/common/global_info.h"
#include "hydra/input/input_packet.h"
#include "hydra/reconstruction/reconstruction_output.h"

namespace hydra {

using pose_graph_tools_msgs::PoseGraph;

struct StampedPose {
  uint64_t stamp;
  Eigen::Isometry3d pose;
};

void addNode(PoseGraph& graph, const StampedPose& stamped_pose, size_t index) {
  auto& node = graph.nodes.emplace_back();
  node.header.stamp.fromNSec(stamped_pose.stamp);
  node.header.frame_id = graph.header.frame_id;
  node.robot_id = GlobalInfo::instance().getRobotPrefix().id;
  node.key = index;
  tf2::convert(stamped_pose.pose, node.pose);
}

void addEdge(PoseGraph& graph, const Eigen::Isometry3d& body_i_T_body_j) {
  auto& edge = graph.edges.emplace_back();
  edge.header.stamp = graph.nodes.back().header.stamp;
  edge.header.frame_id = graph.header.frame_id;

  const auto& prev_node = graph.nodes.front();
  const auto& curr_node = graph.nodes.back();
  edge.key_from = prev_node.key;
  edge.key_to = curr_node.key;
  edge.robot_from = prev_node.robot_id;
  edge.robot_to = curr_node.robot_id;
  edge.type = pose_graph_tools_msgs::PoseGraphEdge::ODOM;
  tf2::convert(body_i_T_body_j, edge.pose);
}

PoseGraph::Ptr makePoseGraph(const StampedPose& curr_pose,
                             const StampedPose& prev_pose,
                             const std::string& frame_id,
                             size_t prev_index) {
  PoseGraph::Ptr graph(new PoseGraph());
  graph->header.stamp.fromNSec(curr_pose.stamp);
  graph->header.frame_id = frame_id;

  addNode(*graph, prev_pose, prev_index);
  addNode(*graph, curr_pose, prev_index + 1);

  Eigen::Isometry3d body_i_T_body_j = curr_pose.pose.inverse() * prev_pose.pose;
  addEdge(*graph, body_i_T_body_j);
  return graph;
}

PoseGraphTracker::PoseGraphTracker(const PoseGraphTracker::Config& config)
    : config_(config::checkValid(config)), num_poses_received_(0) {}

void PoseGraphTracker::update(const InputPacket& msg) {
  const Eigen::Isometry3d curr_pose(Eigen::Translation3d(msg.world_t_body) *
                                    msg.world_R_body);

  if (config_.make_pose_graph && num_poses_received_ > 0) {
    StampedPose curr_pose_stamped{msg.timestamp_ns, curr_pose};
    StampedPose prev_pose_stamped{prev_time_, prev_pose_};
    graphs_.push_back(makePoseGraph(curr_pose_stamped,
                                    prev_pose_stamped,
                                    GlobalInfo::instance().getFrames().odom,
                                    num_poses_received_ - 1));
  } else {
    graphs_.insert(graphs_.end(), msg.pose_graphs.begin(), msg.pose_graphs.end());
  }

  prev_pose_ = curr_pose;
  prev_time_ = msg.timestamp_ns;
  ++num_poses_received_;
}

void PoseGraphTracker::fillPoseGraphs(ReconstructionOutput& msg) {
  VLOG(10) << "[PoseGraph Tracker] queued pose graphs: " << graphs_.size();
  for (const auto& graph : graphs_) {
    msg.pose_graphs.emplace_back(graph);
  }

  graphs_.clear();
}

// TODO(nathan) this may get expanded with covariances...
void declare_config(PoseGraphTracker::Config& conf) {
  config::name("PoseGraphTracker::Config");
  config::field(conf.make_pose_graph, "make_pose_graph");
}

}  // namespace hydra
