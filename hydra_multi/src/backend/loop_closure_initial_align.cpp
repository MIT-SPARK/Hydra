#include "hydra_multi/backend/loop_closure_initial_align.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <kimera_pgmo/utils/common_functions.h>

namespace hydra_multi {

void declare_config(LoopClosureInitialAlignModule::Config& conf) {
  using namespace config;
  name("LoopClosureInitialAlignModule::Config");
  base<InitialAlignModule::Config>(conf);

  field(conf.alignment_gnc_alpha, "alignment_gnc_alpha");
  field(conf.rot_sigma, "rot_sigma");
  field(conf.trans_sigma, "trans_sigma");

  check(conf.alignment_gnc_alpha, GE, 0.f, "alignment_gnc_alpha");
  check(conf.alignment_gnc_alpha, LE, 1.0f, "alignment_gnc_alpha");
  check(conf.rot_sigma, GT, 0.f, "rot_sigma");
  check(conf.trans_sigma, GT, 0.f, "trans_sigma");
}

LoopClosureInitialAlignModule::LoopClosureInitialAlignModule(const Config& config)
    : config(config::checkValid(config)) {}

void LoopClosureInitialAlignModule::update(InitialAlignModule::Info::Ptr info) {
  InitialAlignModule::update(info);
  for (const auto& lc : info->loop_closures) {
    addLoopClosure(lc.from, lc.to, lc.from_T_to);
  }

  updatePairwiseTransforms(info);
  computeInitialTransforms();
}

bool LoopClosureInitialAlignModule::addLoopClosure(const gtsam::Key& from,
                                                   const gtsam::Key& to,
                                                   const gtsam::Pose3& from_T_to) {
  gtsam::Symbol from_symb(from);
  gtsam::Symbol to_symb(to);
  if (from_symb.chr() == to_symb.chr()) {
    VLOG(3) << "Attempting to add single-robot loop closure to "
               "LoopClosureInitialAlignModule.";
    return false;
  }

  if (!kimera_pgmo::robot_prefix_to_id.count(from_symb.chr())) {
    VLOG(3) << "Prefix: " << from_symb.chr() << " not a known robot prefix.";
    return false;
  }

  if (!kimera_pgmo::robot_prefix_to_id.count(to_symb.chr())) {
    VLOG(3) << "Prefix: " << to_symb.chr() << " not a known robot prefix.";
    return false;
  }

  RobotId from_robot_id = kimera_pgmo::robot_prefix_to_id.at(from_symb.chr());
  RobotId to_robot_id = kimera_pgmo::robot_prefix_to_id.at(to_symb.chr());

  all_robots_.insert(from_robot_id);
  all_robots_.insert(to_robot_id);

  if (!interrobot_lcs_.count(from_robot_id)) {
    interrobot_lcs_[from_robot_id] = std::map<RobotId, LoopClosures>();
    pairwise_transforms_[from_robot_id] = std::map<RobotId, Transforms>();
  }
  if (!interrobot_lcs_[from_robot_id].count(to_robot_id)) {
    interrobot_lcs_[from_robot_id][to_robot_id] = LoopClosures();
    pairwise_transforms_[from_robot_id][to_robot_id] = Transforms();
  }
  interrobot_lcs_[from_robot_id][to_robot_id].push_back({from, to, from_T_to});
  return true;
}

void LoopClosureInitialAlignModule::updatePairwiseTransforms(
    LoopClosureInitialAlignModule::Info::Ptr info) {
  // TODO note that we are not actually chaining edges but using the initial value
  // Should mirror interrobot_lcs_
  for (const auto& [from, connections] : interrobot_lcs_) {
    for (const auto& [to, loop_closures] : connections) {
      size_t computed_size = pairwise_transforms_[from][to].size();
      if (loop_closures.size() == computed_size) {
        // All pairwise transforms already computed
        continue;
      }

      for (size_t i = computed_size; i < loop_closures.size(); ++i) {
        const auto& lc = loop_closures[i];
        gtsam::Pose3 from_T_to(lc.from_T_to);
        gtsam::Symbol from_node(lc.from);
        gtsam::Symbol to_node(lc.to);

        // This is necessary for now due to sometimes key not yet added to dgraph
        try {
          gtsam::Pose3 Wto_T_to = info->getPrevRobotPose(to).between(
              info->dgraph->getInitialPose(to_node.chr(), to_node.index()));
          gtsam::Pose3 Wfrom_T_from = info->getPrevRobotPose(from).between(
              info->dgraph->getInitialPose(from_node.chr(), from_node.index()));
          gtsam::Pose3 Wfrom_T_Wto =
              Wfrom_T_from.compose(from_T_to).compose(Wto_T_to.inverse());
          pairwise_transforms_[from][to].push_back(Wfrom_T_Wto);
        } catch (const std::out_of_range& e) {
          LOG(ERROR)
              << "[Loop Closure Initial Align]: index not in deformation graph for to="
              << to_node.string() << " -> from=" << from_node.string()
              << " (error: " << e.what() << ")";
          break;
        }
      }
    }
  }
}

gtsam::Pose3 LoopClosureInitialAlignModule::robustInitialGuess(
    const RobotId& robot_id) {
  // Determine reference robot: use config if set, otherwise use lowest ID
  const RobotId reference_robot =
      (config.reference_robot_id >= 0 &&
       all_robots_.count(static_cast<RobotId>(config.reference_robot_id)))
          ? static_cast<RobotId>(config.reference_robot_id)
          : *all_robots_.begin();

  // Do Robust pose averaging (pairwise) for initial guess
  if (robot_id == reference_robot) {
    // If it's the reference robot, fix at identity
    return gtsam::Pose3();
  }

  const RobotId robot_0 = reference_robot;

  gtsam::Values initial;
  initial.insert(0, gtsam::Pose3());
  gtsam::NonlinearFactorGraph factors;

  // Set up noise
  gtsam::Vector sigmas;
  sigmas.resize(6);
  sigmas.head(3).setConstant(config.rot_sigma);
  sigmas.tail(3).setConstant(config.trans_sigma);
  const gtsam::noiseModel::Diagonal::shared_ptr noise =
      gtsam::noiseModel::Diagonal::Sigmas(sigmas);

  // Get all ransfroms from robot i to robot_id
  std::vector<std::pair<RobotId, gtsam::Pose3>> transforms_to;
  // Get all transforms from robot 0 to robot i
  std::vector<std::pair<RobotId, gtsam::Pose3>> transforms_from_0;
  for (const auto& from_to : pairwise_transforms_) {
    for (const auto& to_transforms : from_to.second) {
      for (const auto& transform : to_transforms.second) {
        if (from_to.first == robot_0 && to_transforms.first == robot_id) {
          factors.add(gtsam::PriorFactor<gtsam::Pose3>(0, transform, noise));
          continue;
        }

        if (from_to.first == robot_id && to_transforms.first == robot_0) {
          factors.add(gtsam::PriorFactor<gtsam::Pose3>(0, transform.inverse(), noise));
          continue;
        }

        if (from_to.first == robot_id) {
          transforms_to.push_back({robot_id, transform.inverse()});
          continue;
        }

        if (to_transforms.first == robot_id) {
          transforms_to.push_back({robot_id, transform});
          continue;
        }

        if (from_to.first == robot_0) {
          transforms_from_0.push_back({robot_id, transform});
        }

        if (to_transforms.first == robot_id) {
          transforms_from_0.push_back({robot_id, transform.inverse()});
        }
      }
    }
  }

  for (const auto& interm1_transform : transforms_to) {
    for (const auto& interm2_transform : transforms_from_0) {
      if (interm1_transform.first == interm2_transform.first) {
        auto o_T_id = interm2_transform.second.compose(interm1_transform.second);
        factors.add(gtsam::PriorFactor<gtsam::Pose3>(0, o_T_id, noise));
      }
    }
  }

  // Solve with GNC
  gtsam::GncParams<gtsam::LevenbergMarquardtParams> gnc_params;
  auto gnc = gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>(
      factors, initial, gnc_params);
  gnc.setInlierCostThresholdsAtProbability(config.alignment_gnc_alpha);
  gtsam::Values estimate = gnc.optimize();
  return estimate.at<gtsam::Pose3>(0);
}

void LoopClosureInitialAlignModule::computeInitialTransforms() {
  if (pairwise_transforms_.size() == 0) {
    VLOG(10) << "No pair-wsie transforms computed yet.";
    return;
  }

  // Set up initial guess
  gtsam::Values initial;
  for (const RobotId& robot : all_robots_) {
    if (W_T_robot_.count(robot)) {
      initial.insert(robot, W_T_robot_.at(robot));
    } else {
      gtsam::Pose3 initial_guess = robustInitialGuess(robot);
      initial.insert(robot, initial_guess);
    }
  }

  // Set up noise
  gtsam::Vector sigmas;
  sigmas.resize(6);
  sigmas.head(3).setConstant(config.rot_sigma);
  sigmas.tail(3).setConstant(config.trans_sigma);
  const gtsam::noiseModel::Diagonal::shared_ptr noise =
      gtsam::noiseModel::Diagonal::Sigmas(sigmas);

  // Determine reference robot: use config if set, otherwise use lowest ID
  const RobotId reference_robot =
      (config.reference_robot_id >= 0 &&
       all_robots_.count(static_cast<RobotId>(config.reference_robot_id)))
          ? static_cast<RobotId>(config.reference_robot_id)
          : *all_robots_.begin();

  // Add factors (from pairwise transforms)
  gtsam::NonlinearFactorGraph factors;
  for (const auto& from_to : pairwise_transforms_) {
    for (const auto& to_transforms : from_to.second) {
      for (const auto& transform : to_transforms.second) {
        factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
            from_to.first, to_transforms.first, transform, noise));
      }
    }
  }
  // Fix reference robot as identity
  factors.add(gtsam::PriorFactor<gtsam::Pose3>(reference_robot, gtsam::Pose3(), noise));

  // TODO(Yun) Consider adding back set weights logic

  // Solve with GNC
  align_num_measurements_ = factors.size() - 1;  // Don't count the prior factor
  gtsam::GncParams<gtsam::LevenbergMarquardtParams> gnc_params;
  auto gnc = gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>(
      factors, initial, gnc_params);
  gnc.setInlierCostThresholdsAtProbability(config.alignment_gnc_alpha);
  gtsam::Values estimate = gnc.optimize();
  align_num_inliers_ = gnc.getWeights().sum();

  // Update W_T_robot_
  for (const auto& key : estimate.keys()) {
    const RobotId robot_id(key);
    W_T_robot_[robot_id] = estimate.at<gtsam::Pose3>(key);
    VLOG(5) << "Update robot " << robot_id << " W_T_robot: " << W_T_robot_[robot_id];
  }
}

}  // namespace hydra_multi
