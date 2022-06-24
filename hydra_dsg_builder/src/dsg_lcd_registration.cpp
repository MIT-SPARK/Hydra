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
#include "hydra_dsg_builder/dsg_lcd_registration.h"

#include <hydra_utils/timing_utilities.h>
#include <kimera_pgmo/utils/CommonFunctions.h>
#include <pose_graph_tools/LcdFrameRegistration.h>
#include <ros/service.h>
#include <tf2_eigen/tf2_eigen.h>
#include <fstream>

namespace hydra {
namespace lcd {

inline std::ostream& operator<<(std::ostream& out, const gtsam::Quaternion& q) {
  out << "{w: " << q.w() << ", x: " << q.x() << ", y: " << q.y() << ", z: " << q.z()
      << "}";
  return out;
}

using incremental::SharedDsgInfo;
using DsgNode = DynamicSceneGraphNode;
using hydra::timing::ScopedTimer;

struct AgentNodePose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool valid = false;
  gtsam::Pose3 world_T_body;
  NodeId id;
};

AgentNodePose getAgentPose(const DynamicSceneGraph& graph, NodeId root_id) {
  if (!graph.hasNode(root_id)) {
    return {};
  }

  const SceneGraphNode& root_node = graph.getNode(root_id).value();

  std::optional<NodeId> child_id = std::nullopt;
  for (const auto& root_child : root_node.children()) {
    if (graph.isDynamic(root_child)) {
      child_id = root_child;
      break;
    }
  }

  if (!child_id) {
    return {};
  }

  CHECK(child_id);

  const auto& attrs =
      graph.getNode(*child_id).value().get().attributes<AgentNodeAttributes>();
  return {
      true, gtsam::Pose3(gtsam::Rot3(attrs.world_R_body), attrs.position), *child_id};
}

AgentNodePose getQueryPose(const DynamicSceneGraph& graph, NodeId query_agent_id) {
  const auto& attrs =
      graph.getNode(query_agent_id).value().get().attributes<AgentNodeAttributes>();
  return {true,
          gtsam::Pose3(gtsam::Rot3(attrs.world_R_body), attrs.position),
          query_agent_id};
}

DsgRegistrationSolution getFullSolutionFromLayer(
    const DynamicSceneGraph& graph,
    const LayerRegistrationSolution& solution,
    NodeId query_agent_id,
    NodeId match_root) {
  if (!solution.valid || solution.inliers.empty()) {
    return {};
  }

  auto from_pose_info = getQueryPose(graph, query_agent_id);
  auto to_pose_info = getAgentPose(graph, match_root);

  if (!from_pose_info.valid || !to_pose_info.valid) {
    return {};
  }

  VLOG(3) << "=================================================";
  VLOG(3) << "world_T_from:";
  VLOG(3) << from_pose_info.world_T_body;
  VLOG(3) << "";
  VLOG(3) << "world_T_to:";
  VLOG(3) << to_pose_info.world_T_body;
  VLOG(3) << "";
  VLOG(3) << "dest_T_src:";
  VLOG(3) << solution.dest_T_src;
  VLOG(3) << "";

  gtsam::Pose3 to_T_from = to_pose_info.world_T_body.inverse() * solution.dest_T_src *
                           from_pose_info.world_T_body;
  VLOG(3) << "to_T_from:";
  VLOG(3) << to_T_from;
  VLOG(3) << "";

  return {true, from_pose_info.id, to_pose_info.id, to_T_from, -1};
}

void logRegistrationProblem(const std::string& path_prefix,
                            const DynamicSceneGraph& dsg,
                            const LayerRegistrationSolution& solution,
                            const DsgRegistrationInput& match,
                            NodeId query_agent_id) {
  static size_t registration_index = 0;
  std::stringstream ss;
  ss << path_prefix << registration_index << ".json";
  ++registration_index;

  std::ofstream outfile(ss.str());
  outfile << "query_id: " << query_agent_id << std::endl;
  outfile << "query_set: " << displayNodeSymbolContainer(match.query_nodes)
          << std::endl;
  outfile << "match_set: " << displayNodeSymbolContainer(match.match_nodes)
          << std::endl;

  auto match_pose_info = getAgentPose(dsg, match.match_root);
  outfile << "match_id: " << match_pose_info.id << std::endl;
  outfile << "world_q_match: " << match_pose_info.world_T_body.rotation().toQuaternion()
          << std::endl;
  outfile << "world_t_match: " << match_pose_info.world_T_body.translation()
          << std::endl;
  outfile << "match_valid: " << match_pose_info.valid << std::endl;

  outfile << "solution_valid: " << solution.valid << std::endl;
  outfile << "dest_q_src: " << solution.dest_T_src.rotation().toQuaternion()
          << std::endl;
  outfile << "dest_t_src: " << solution.dest_T_src.translation() << std::endl;

  // TODO(nathan) output position data
}

DsgTeaserSolver::DsgTeaserSolver(LayerId layer_id,
                                 const LayerRegistrationConfig& config,
                                 const TeaserParams& params)
    : layer_id(layer_id), config(config), solver(params) {
  const std::string layer_str = DsgLayers::LayerIdToString(layer_id);
  timer_prefix = "lcd/" + layer_str + "_registration";
  log_prefix = config.registration_output_path + "/" + layer_str + "_registration_";
}

DsgRegistrationSolution DsgTeaserSolver::solve(const DynamicSceneGraph& dsg,
                                               const DsgRegistrationInput& match,
                                               NodeId query_agent_id) const {
  // TODO(nathan) helper function in dsg
  const uint64_t timestamp =
      dsg.getDynamicNode(query_agent_id).value().get().timestamp.count();
  ScopedTimer timer(timer_prefix, timestamp, true, 2, false);

  if (match.query_nodes.size() <= 3 || match.match_nodes.size() <= 3) {
    return {};
  }

  LayerRegistrationProblem<std::set<NodeId>> problem;
  problem.src_nodes = match.query_nodes;
  problem.dest_nodes = match.match_nodes;

  const auto& layer = dsg.getLayer(layer_id);
  LayerRegistrationSolution solution;
  if (config.use_pairwise_registration) {
    solution = registerDsgLayerPairwise(config, solver, problem, layer);
  } else {
    solution = registerDsgLayerSemantic(config, solver, problem, layer);
  }

  if (config.log_registration_problem) {
    logRegistrationProblem(log_prefix, dsg, solution, match, query_agent_id);
  }

  return getFullSolutionFromLayer(dsg, solution, query_agent_id, match.match_root);
}

inline size_t getFrameIdFromNode(const DynamicSceneGraph& graph, NodeId node_id) {
  const auto& attrs =
      graph.getNode(node_id).value().get().attributes<AgentNodeAttributes>();
  return NodeSymbol(attrs.external_key).categoryId();
}

inline size_t getRobotIdFromNode(const DynamicSceneGraph& graph, NodeId node_id) {
  const auto& attrs =
      graph.getNode(node_id).value().get().attributes<AgentNodeAttributes>();
  // TODO(yun) cleaner way to track robot prefix to id?
  return kimera_pgmo::robot_prefix_to_id.at(NodeSymbol(attrs.external_key).category());
}

inline size_t getTimestampFromNode(const DynamicSceneGraph& graph, NodeId node_id) {
  const auto& attrs =
      graph.getNode(node_id).value().get().attributes<AgentNodeAttributes>();
  return NodeSymbol(attrs.external_key).categoryId();
}

DsgRegistrationSolution DsgAgentSolver::solve(const DynamicSceneGraph& dsg,
                                              const DsgRegistrationInput& match,
                                              NodeId) const {
  if (match.query_nodes.empty() || match.match_nodes.empty()) {
    return {};
  }

  if (!ros::service::exists("frame_registration", true)) {
    LOG(ERROR) << "Frame registration service missing!";
    return {};
  }

  // at the agent level, match sets are one node each
  const NodeId query_id = *match.query_nodes.begin();
  const NodeId match_id = *match.match_nodes.begin();

  uint64_t timestamp;
  pose_graph_tools::LcdFrameRegistration msg;
  msg.request.query_robot = getRobotIdFromNode(dsg, query_id);
  msg.request.match_robot = getRobotIdFromNode(dsg, match_id);
  msg.request.query = getFrameIdFromNode(dsg, query_id);
  msg.request.match = getFrameIdFromNode(dsg, match_id);
  timestamp = dsg.getDynamicNode(query_id).value().get().timestamp.count();

  ScopedTimer timer("lcd/register_agent", timestamp, true, 2, false);

  if (!ros::service::call("frame_registration", msg)) {
    LOG(ERROR) << "Frame registration service failed!";
    return {};
  }

  VLOG(3) << "Visual Registration Request: " << msg.request;
  VLOG(3) << "Visual Registration Response: " << msg.response;

  if (!msg.response.valid) {
    LOG(INFO) << "registration failed: " << NodeSymbol(query_id).getLabel() << " -> "
              << NodeSymbol(match_id).getLabel();
    return {};
  }

  Eigen::Quaterniond match_q_query;
  Eigen::Vector3d match_t_query;
  tf2::convert(msg.response.match_T_query.orientation, match_q_query);
  tf2::convert(msg.response.match_T_query.position, match_t_query);
  LOG(INFO) << "registration worked " << NodeSymbol(query_id).getLabel() << " -> "
            << NodeSymbol(match_id).getLabel();
  return {true,
          query_id,
          match_id,
          gtsam::Pose3(gtsam::Rot3(match_q_query), match_t_query),
          -1};
}

}  // namespace lcd
}  // namespace hydra
