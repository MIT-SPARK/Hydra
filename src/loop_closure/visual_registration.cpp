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
#include "hydra/loop_closure/visual_registration.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>

#include <iomanip>

#include "hydra/utils/teaser_params.h"

namespace hydra::lcd {

using Input = SensorRegistrationInput;
using TeaserSolver = teaser::RobustRegistrationSolver;
using Correspondences = std::vector<std::pair<size_t, size_t>>;

struct TeaserSolution {
  bool valid = false;
  Eigen::Matrix3d dest_R_src;
  Eigen::Vector3d dest_t_src;
  std::vector<int> inliers;

  inline operator bool() const { return valid; }
};

TeaserSolution registerViews(TeaserSolver& solver,
                             const Input& input,
                             const Correspondences& correspondences,
                             size_t min_inliers) {
  Eigen::Matrix<double, 3, Eigen::Dynamic> src_points(3, correspondences.size());
  Eigen::Matrix<double, 3, Eigen::Dynamic> dest_points(3, correspondences.size());
  const auto& src = input.query_features->features;
  const auto& dest = input.match_features->features;
  for (size_t i = 0; i < correspondences.size(); ++i) {
    const auto correspondence = correspondences[i];
    src_points.col(i) = src.row(correspondence.first);
    dest_points.col(i) = dest.row(correspondence.second);
  }

  const Eigen::IOFormat fmt(4, 0, ", ", "\n", "[", "]");
  VLOG(VLEVEL_ALL) << "=======================================================";
  VLOG(VLEVEL_ALL) << std::setfill(' ') << "Source: " << std::endl
                   << src_points.format(fmt);
  VLOG(VLEVEL_ALL) << std::setfill(' ') << "Dest: " << std::endl
                   << dest_points.format(fmt);

  VLOG(VLEVEL_INFO) << "[Hydra LCD] Registering two-view problem with "
                    << correspondences.size() << " correspondences out of "
                    << src.rows() << " query and " << dest.rows() << " match features";

  auto params = solver.getParams();
  params.estimate_scaling = false;
  solver.reset(params);

  auto result = solver.solve(src_points, dest_points);
  if (!result.valid) {
    return {};
  }

  auto inliers = solver.getInlierMaxClique();
  if (inliers.size() < min_inliers) {
    VLOG(VLEVEL_INFO) << "[Hydra LCD] Not enough inliers after registration: "
                      << inliers.size() << " / " << min_inliers;
    return {};
  }

  return {true, result.rotation, result.translation, inliers};
}

RegistrationSolution getFullSolution(const Input& input,
                                     const TeaserSolution& solution) {
  Eigen::Quaterniond q(solution.dest_R_src);
  const Eigen::Isometry3d dest_T_src = Eigen::Translation3d(solution.dest_t_src) *
                                       Eigen::Quaterniond(solution.dest_R_src);
  const auto to_T_from = input.match_features->body_T_sensor * dest_T_src *
                         input.query_features->body_T_sensor.inverse();
  return {true,
          input.query,
          input.match,
          to_T_from.translation(),
          Eigen::Quaterniond(to_T_from.rotation()),
          -1};
}

TwoViewTeaserSolver::TwoViewTeaserSolver(const Config& config)
    : config(config::checkValid(config)), solver_(config.teaser) {
  if (config.timer_prefix.empty()) {
    timer_prefix = "lcd/agent_registration";
  } else {
    timer_prefix = config.timer_prefix;
  }

  matcher_ = cv::DescriptorMatcher::create(config.matcher_type);
}

RegistrationSolution TwoViewTeaserSolver::solve(const DynamicSceneGraph&,
                                                const Input& input) const {
  if (!input.query_features || !input.match_features) {
    LOG(ERROR) << "Sensor features required for two-view registration";
    return {};
  }

  std::vector<std::vector<cv::DMatch>> matches;
  matcher_->knnMatch(
      input.query_features->descriptors, input.match_features->descriptors, matches, 2);

  Correspondences correspondences;
  correspondences.reserve(matches.size());
  for (const auto& match : matches) {
    if (match.empty()) {
      continue;
    }

    if (match.size() == 1) {
      correspondences.push_back({match.front().queryIdx, match.front().trainIdx});
      continue;
    }

    if (match[0].distance >= config.lowe_ratio * match[1].distance) {
      continue;
    }

    correspondences.push_back({match.front().queryIdx, match.front().trainIdx});
  }

  if (correspondences.size() < config.min_correspondences) {
    VLOG(VLEVEL_INFO)
        << "[Hydra LCD] Not enough correspondences for two-view registration: "
        << correspondences.size() << " / " << config.min_correspondences;
    return {};
  }

  auto teaser_result =
      registerViews(solver_, input, correspondences, config.min_inliers);
  if (!teaser_result) {
    return {};
  }

  return getFullSolution(input, teaser_result);
}

void declare_config(TwoViewTeaserSolver::Config& config) {
  using namespace config;
  name("TwoViewTeaserSolver::Config");
  field(config.teaser, "teaser");
  field(config.timer_prefix, "timer_prefix");
  field(config.matcher_type, "matcher_type");
  field(config.min_correspondences, "min_correspondences");
  field(config.lowe_ratio, "lowe_ratio");
  field(config.min_inliers, "min_inliers");

  std::vector<std::string> valid_types{"BruteForce",
                                       "BruteForce-L1",
                                       "BruteForce-Hamming",
                                       "BruteForce-Hamming(2)",
                                       "FlannBased"};
  checkIsOneOf(config.matcher_type, valid_types, "matcher_type");
  check(config.lowe_ratio, GT, 0.0, "lowe_ratio");
}

}  // namespace hydra::lcd
