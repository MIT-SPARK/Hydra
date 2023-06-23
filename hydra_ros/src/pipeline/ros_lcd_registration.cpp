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
#include "hydra_ros/pipeline/ros_lcd_registration.h"

#include <hydra/utils/timing_utilities.h>
#include <kimera_pgmo/utils/CommonFunctions.h>
#include <pose_graph_tools/LcdFrameRegistration.h>
#include <ros/service.h>
#include <tf2_eigen/tf2_eigen.h>

#include <iomanip>

namespace hydra {

using hydra::timing::ScopedTimer;
using lcd::DsgRegistrationInput;
using lcd::DsgRegistrationSolution;

inline size_t getRobotIdFromNode(const DynamicSceneGraph& graph, NodeId node_id) {
  const auto& attrs =
      graph.getNode(node_id).value().get().attributes<AgentNodeAttributes>();
  // TODO(yun) cleaner way to track robot prefix to id?
  return kimera_pgmo::robot_prefix_to_id.at(NodeSymbol(attrs.external_key).category());
}

inline size_t getFrameIdFromNode(const DynamicSceneGraph& graph, NodeId node_id) {
  const auto& attrs =
      graph.getNode(node_id).value().get().attributes<AgentNodeAttributes>();
  return NodeSymbol(attrs.external_key).categoryId();
}

std::string getPoseRepr(const Eigen::Quaterniond& q, const Eigen::Vector3d& v) {
  const Eigen::IOFormat format(3, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");

  std::stringstream ss;
  ss << std::fixed << std::setprecision(3) << "R={w: " << q.w() << ", x: " << q.x()
     << ", y: " << q.y() << ", z: " << q.z() << "}, t=" << v.format(format);
  return ss.str();
}

DsgRegistrationSolution DsgAgentSolver::solve(const DynamicSceneGraph& dsg,
                                              const DsgRegistrationInput& match,
                                              NodeId) const {
  if (match.query_nodes.empty() || match.match_nodes.empty()) {
    return {};
  }

  if (!ros::service::exists("frame_registration", true)) {
    LOG(ERROR) << "[Hydra LCD] Frame registration service missing!";
    return {};
  }

  // at the agent level, match sets are one node each
  const NodeId query_id = *match.query_nodes.begin();
  const NodeId match_id = *match.match_nodes.begin();

  if (!dsg.hasNode(query_id) || !dsg.hasNode(match_id)) {
    LOG(ERROR) << "Query or match node does not exist in graph!";
    return {};
  }

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

  VLOG(3) << "Visual registration request: query={robot: " << msg.request.query_robot
          << ", frame: " << msg.request.query
          << "}, match={robot: " << msg.request.match_robot
          << ", frame: " << msg.request.match;

  if (!msg.response.valid) {
    VLOG(1) << "Visual registration failed: " << NodeSymbol(query_id).getLabel()
            << " -> " << NodeSymbol(match_id).getLabel();
    return {};
  }

  Eigen::Quaterniond match_q_query;
  Eigen::Vector3d match_t_query;
  tf2::convert(msg.response.match_T_query.orientation, match_q_query);
  tf2::convert(msg.response.match_T_query.position, match_t_query);
  const Eigen::IOFormat format(3, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");
  VLOG(3) << "Visual registration succeded: "
          << getPoseRepr(match_q_query, match_t_query);
  return {true,
          query_id,
          match_id,
          gtsam::Pose3(gtsam::Rot3(match_q_query), match_t_query),
          -1};
}

}  // namespace hydra
