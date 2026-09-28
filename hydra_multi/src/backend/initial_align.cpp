#include "hydra_multi/backend/initial_align.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <kimera_pgmo/utils/common_functions.h>

namespace hydra_multi {

void declare_config(InitialAlignModule::Config& conf) {
  using namespace config;
  name("InitialAlignModule::Config");
  field(conf.dgraph_consistency_tol, "dgraph_consistency_tol");
  field(conf.reference_robot_id, "reference_robot_id");

  check(conf.dgraph_consistency_tol, GE, 0.0f, "dgraph_consistency_tol");
}

gtsam::Pose3 InitialAlignModule::Info::getPrevRobotPose(size_t robot_id) const {
  // TODO(nathan) think about warning if this isn't set?
  auto iter = prev_W_T_robot.find(robot_id);
  return iter == prev_W_T_robot.end() ? gtsam::Pose3() : iter->second;
}

void InitialAlignModule::update(InitialAlignModule::Info::Ptr info) {
  if (!checkAlignConsistent(info)) {
    LOG(WARNING)
        << "Initial alignment inconsistent with deformation graph. Resetting...";
    W_T_robot_.clear();
  }
}

gtsam::Values InitialAlignModule::computeInitialGuess(
    InitialAlignModule::Info::Ptr info,
    const gtsam::Values& input_values,
    const NodeIdRobotMap& node_id_robot_map) {
  gtsam::Values aligned;
  for (const auto& key : input_values.keys()) {
    gtsam::Symbol node(key);
    RobotId robot_id;
    if (node.chr() == 'p' || node.chr() == 'O') {
      // Places or object nodes
      if (!node_id_robot_map.count(key)) {
        LOG(ERROR) << "Unrecognized node in loopClosureInitialization";
        continue;
      }
      robot_id = node_id_robot_map.at(key);
    } else if (kimera_pgmo::robot_prefix_to_id.count(node.chr())) {
      // Agent node
      robot_id = kimera_pgmo::robot_prefix_to_id.at(node.chr());
    } else if (kimera_pgmo::vertex_prefix_to_id.count(node.chr())) {
      // Deformation graph vertex
      robot_id = kimera_pgmo::vertex_prefix_to_id.at(node.chr());
    } else {
      LOG(ERROR) << "Unrecognized prefix in loopClosureInitialization.";
      continue;
    }

    if (!W_T_robot_.count(robot_id)) {
      LOG_EVERY_N(ERROR, 50) << "Missing Robot ID (" << robot_id
                             << ") when computing initial guess.";
      continue;
    }

    if (node.chr() == 'p' || node.chr() == 'O') {
      // Places are from temporary values (not stored in pgmo dgraph)
      gtsam::Pose3 Wglobal_T_Wrobot = W_T_robot_.at(robot_id);
      gtsam::Pose3 Wrobot_T_pose =
          info->getPrevRobotPose(robot_id).between(input_values.at<gtsam::Pose3>(key));
      gtsam::Pose3 Wglobal_T_pose = Wglobal_T_Wrobot.compose(Wrobot_T_pose);
      aligned.insert(key, Wglobal_T_pose);
      continue;
    }

    gtsam::Pose3 Wrobot_T_pose;
    if (kimera_pgmo::robot_prefix_to_id.count(node.chr())) {
      Wrobot_T_pose = info->getPrevRobotPose(robot_id).between(
          info->dgraph->getInitialPose(node.chr(), node.index()));
    } else {
      Wrobot_T_pose = info->getPrevRobotPose(robot_id).between(gtsam::Pose3(
          gtsam::Rot3(),
          info->dgraph->getInitialPositionVertex(node.chr(), node.index())));
    }
    gtsam::Pose3 Wglobal_T_Wrobot = W_T_robot_.at(robot_id);
    gtsam::Pose3 Wglobal_T_pose = Wglobal_T_Wrobot.compose(Wrobot_T_pose);
    aligned.insert(key, Wglobal_T_pose);
  }
  return aligned;
}

bool InitialAlignModule::checkAlignConsistent(InitialAlignModule::Info::Ptr info) {
  const auto optimized_values = info->dgraph->getValues();
  for (const auto& [robot_id, transform] : W_T_robot_) {
    CHECK(kimera_pgmo::robot_id_to_prefix.count(robot_id));

    const char robot_prefix = kimera_pgmo::robot_id_to_prefix.at(robot_id);

    const gtsam::Pose3& Wrobot_T_pose = info->getPrevRobotPose(robot_id).between(
        info->dgraph->getInitialPose(robot_prefix, 0));
    const gtsam::Pose3& Wglobal_T_pose =
        optimized_values->at<gtsam::Pose3>(gtsam::Symbol(robot_prefix, 0));
    const gtsam::Pose3& Wglobal_T_Wrobot =
        Wglobal_T_pose.compose(Wrobot_T_pose.inverse());
    if (!transform.equals(Wglobal_T_Wrobot, config.dgraph_consistency_tol)) {
      LOG(ERROR) << "Robot " << robot_id << " has initial alignment: " << transform
                 << " which is inconsistent with actual: " << Wglobal_T_Wrobot;
      return false;
    }
  }
  return true;
}

}  // namespace hydra_multi
