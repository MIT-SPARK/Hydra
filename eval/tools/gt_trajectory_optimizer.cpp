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
#include <KimeraRPGO/RobustSolver.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/BoundingConstraint.h>
#include <gtsam/slam/dataset.h>
#include <kimera_pgmo/deformation_graph.h>
#include <spark_dsg/dynamic_scene_graph.h>
#include <yaml-cpp/yaml.h>

DEFINE_string(result_dir, "", "directory to read from");
DEFINE_string(g2o_file, "pgmo/result.g2o", "file to read");
DEFINE_string(pgmo_file, "pgmo/deformation_graph.dgrf", "deformation graph");
DEFINE_string(dsg_file, "backend/dsg.json", "file to read");
DEFINE_string(config_file, "gt_sidpac_f34.yaml", "file to read");
DEFINE_string(agent_prefix, "a", "agent prefix");
DEFINE_bool(use_g2o, false, "use g2o file");

using namespace KimeraRPGO;
using namespace spark_dsg;
using PoseFactor = gtsam::BetweenFactor<gtsam::Pose3>;
using PriorFactor = gtsam::PriorFactor<gtsam::Pose3>;
using ManualLCFactor = gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3, double>;

#if GTSAM_VERSION_MAJOR <= 4 && GTSAM_VERSION_MINOR < 3
using GtsamJacobianType = boost::optional<gtsam::Matrix&>;
#define JACOBIAN_DEFAULT \
  {}
#else
using GtsamJacobianType = gtsam::OptionalMatrixType;
#define JACOBIAN_DEFAULT nullptr
#endif

class Pose3Bounds : public gtsam::BoundingConstraint1<gtsam::Pose3> {
 public:
  using shared_ptr = gtsam::NonlinearFactor::shared_ptr;

  Pose3Bounds(
      gtsam::Key key, size_t idx, double threshold, bool is_greater, double mu = 1000.0)
      : gtsam::BoundingConstraint1<gtsam::Pose3>(key, threshold, is_greater, mu),
        idx_(idx),
        threshold_(threshold),
        is_greater_(is_greater),
        mu_(mu) {}

  double value(const gtsam::Pose3& x,
               GtsamJacobianType H = JACOBIAN_DEFAULT) const override {
    if (H) {
      gtsam::Matrix D =
          gtsam::Matrix::Zero(1, gtsam::traits<gtsam::Pose3>::GetDimension(x));
      D(0, idx_ + 3) = 1.0;
      *H = D;
    }
    return x.translation()(idx_);
  }

  shared_ptr clone() const override {
    return shared_ptr(new Pose3Bounds(front(), idx_, threshold_, is_greater_, mu_));
  }

 private:
  size_t idx_;
  double threshold_;
  bool is_greater_;
  double mu_;
};

void add_height_bounds(gtsam::NonlinearFactorGraph& factors,
                       const gtsam::Values& values,
                       size_t start,
                       size_t end,
                       size_t skip,
                       double lower,
                       double upper,
                       double mu = 1000.0) {
  for (size_t i = start; i < end; i += skip) {
    const auto key = gtsam::Symbol(FLAGS_agent_prefix[0], i);
    if (!values.exists(key)) {
      return;
    }

    factors.emplace_shared<Pose3Bounds>(key, 2, lower, false, mu);
    factors.emplace_shared<Pose3Bounds>(key, 2, upper, true, mu);
  }
}

void add_height(gtsam::NonlinearFactorGraph& factors,
                const gtsam::Values& values,
                size_t start,
                size_t end,
                size_t skip,
                double height,
                double variance = 1e-1,
                double other_variance = 100.0) {
  for (size_t i = start; i < end; i += skip) {
    const auto key = gtsam::Symbol(FLAGS_agent_prefix[0], i);
    if (!values.exists(key)) {
      return;
    }

    gtsam::Pose3 orig_pose = values.at<gtsam::Pose3>(key);
    gtsam::Rot3 prior_rot = orig_pose.rotation();
    gtsam::Point3 prior_trans(
        orig_pose.translation().x(), orig_pose.translation().y(), height);

    Eigen::VectorXd vars(6);
    vars << other_variance, other_variance, other_variance, other_variance,
        other_variance, variance;
    auto noise = gtsam::noiseModel::Diagonal::Variances(vars);

    factors.add(PriorFactor(key, gtsam::Pose3(prior_rot, prior_trans), noise));
  }
}

std::vector<size_t> read_timestamps(const std::string& filepath) {
  DynamicSceneGraph graph;
  graph.load(filepath);

  const auto& agents = graph.getLayer(DsgLayers::AGENTS, 'a');

  std::vector<size_t> times_ns;
  for (const auto& node : agents.nodes()) {
    times_ns.push_back(node->timestamp.value().count());
  }

  return times_ns;
}

void load_factors_g2o(gtsam::NonlinearFactorGraph& factors, gtsam::Values& values) {
  const std::string g2o_file = FLAGS_result_dir + "/" + FLAGS_g2o_file;
  auto g2o_info = gtsam::readG2o(g2o_file, true);
  if (!g2o_info.first || g2o_info.first->size() == 0) {
    LOG(FATAL) << "empty g2o file!";
    return;
  }

  factors = *g2o_info.first;
  values = *g2o_info.second;
  const char agent_prefix = FLAGS_agent_prefix[0];

  std::vector<gtsam::Key> to_erase;
  for (const auto& key : values.keys()) {
    if (gtsam::Symbol(key).chr() != agent_prefix) {
      to_erase.push_back(key);
    }
  }

  for (const auto& key : to_erase) {
    values.erase(key);
  }
}

void add_priors_from_config(const YAML::Node& config,
                            gtsam::Values& values,
                            gtsam::NonlinearFactorGraph& factors) {
  for (const auto& constraint : config["heights"]) {
    size_t start = constraint["start"].as<size_t>();
    size_t stop = constraint["stop"].as<size_t>();
    size_t inc = constraint["inc"].as<size_t>();
    double height = constraint["height"].as<double>();
    double variance = constraint["variance"].as<double>();
    double default_variance = constraint["default_variance"].as<double>();
    LOG(INFO) << constraint;
    LOG(INFO) << "Adding height prior: range(" << start << ", " << stop << ", " << inc
              << ") @ " << height << ", var=" << variance
              << " (default=" << default_variance << ")";
    add_height(factors, values, start, stop, inc, height, variance, default_variance);
  }
}

void add_bounds_from_config(const YAML::Node& config,
                            gtsam::Values& values,
                            gtsam::NonlinearFactorGraph& factors) {
  for (const auto& constraint : config["height_bounds"]) {
    size_t start = constraint["start"].as<size_t>();
    size_t stop = constraint["stop"].as<size_t>();
    size_t inc = constraint["inc"].as<size_t>();
    double lower = constraint["lower"].as<double>();
    double upper = constraint["upper"].as<double>();
    double mu = constraint["mu"].as<double>();
    LOG(INFO) << "Adding height constraint: range(" << start << ", " << stop << ", "
              << inc << ") -> [" << lower << ", " << upper << "], mu=" << mu;
    add_height_bounds(factors, values, start, stop, inc, lower, upper, mu);
  }
}

void output(const gtsam::NonlinearFactorGraph& factors, gtsam::Values& values) {
  const char agent_prefix = FLAGS_agent_prefix[0];

  std::vector<gtsam::Key> to_erase;
  for (const auto& key : values.keys()) {
    if (gtsam::Symbol(key).chr() != agent_prefix) {
      to_erase.push_back(key);
    }
  }

  for (const auto& key : to_erase) {
    values.erase(key);
  }

  gtsam::writeG2o(factors, values, "opt_trajectory.g2o");

  std::string dsg_file = FLAGS_result_dir + "/" + FLAGS_dsg_file;
  auto timestamps = read_timestamps(dsg_file);

  std::ofstream fout("opt_trajectory.csv");
  fout << "#timestamp_kf,x,y,z,qw,qx,qy,qz" << std::endl;

  for (const auto& kv_pair : values) {
    gtsam::Symbol symb(kv_pair.key);
    if (symb.chr() != agent_prefix) {
      continue;
    }

    if (symb.index() >= timestamps.size()) {
      LOG(FATAL) << "timestamps and values disagree: " << symb.index()
                 << " >= " << timestamps.size();
    }

    fout << timestamps[symb.index()] << ",";

    auto pose = kv_pair.value.cast<gtsam::Pose3>();
    Eigen::Vector3d t = pose.translation();
    auto q = pose.rotation().toQuaternion();

    fout << t.x() << "," << t.y() << "," << t.z() << "," << q.w() << "," << q.x() << ","
         << q.y() << "," << q.z() << std::endl;
  }
}

void load_factors_pgmo(const RobustSolverParams& params,
                       gtsam::NonlinearFactorGraph& factors,
                       gtsam::Values& values) {
  const std::string pgmo_file = FLAGS_result_dir + "/" + FLAGS_pgmo_file;

  kimera_pgmo::DeformationGraph graph;
  graph.initialize(params);
  graph.load(pgmo_file);

  factors = graph.getGtsamFactors();
  values = graph.getGtsamValues();
}

void add_manual_loop_clousres(const YAML::Node& config,
                              const gtsam::Values& values,
                              gtsam::NonlinearFactorGraph& factors) {
  const double var = config["manual_lc_var"].as<double>();

  Eigen::VectorXd lc_vars(6);
  lc_vars << var, var, var, var, var, var;
  auto lc_noise = gtsam::noiseModel::Diagonal::Variances(lc_vars);

  const char agent_prefix = FLAGS_agent_prefix[0];
  for (const auto& factor_info : config["manual_loop_closures"]) {
    gtsam::Symbol key1(agent_prefix, factor_info["index1"].as<size_t>());
    if (!values.exists(key1)) {
      LOG(WARNING) << "Invalid key for loop-closure: " << key1 << " (" << values.size()
                   << " keys)";
      continue;
    }

    gtsam::Symbol key2(agent_prefix, factor_info["index2"].as<size_t>());
    if (!values.exists(key2)) {
      LOG(WARNING) << "Invalid key for loop-closure: " << key2 << " (" << values.size()
                   << " keys)";
      continue;
    }

    LOG(INFO) << "Adding manual loop closure: " << key1 << " -> " << key2;
    factors.emplace_shared<PoseFactor>(key1, key2, gtsam::Pose3(), lc_noise);
  }
}

void remap_covariances(const YAML::Node& config,
                       const gtsam::NonlinearFactorGraph& old_factors,
                       gtsam::NonlinearFactorGraph& factors) {
  const double odom_rot_var = config["odom_rot"].as<double>();
  const double odom_trans_var = config["odom_trans"].as<double>();
  const double lc_rot_var = config["loop_close_rot"].as<double>();
  const double lc_trans_var = config["loop_close_trans"].as<double>();

  Eigen::VectorXd odom_vars(6);
  odom_vars << odom_rot_var, odom_rot_var, odom_rot_var, odom_trans_var, odom_trans_var,
      odom_trans_var;
  auto odom_noise = gtsam::noiseModel::Diagonal::Variances(odom_vars);

  Eigen::VectorXd lc_vars(6);
  lc_vars << lc_rot_var, lc_rot_var, lc_rot_var, lc_trans_var, lc_trans_var,
      lc_trans_var;
  auto lc_noise = gtsam::noiseModel::Diagonal::Variances(lc_vars);

  const char agent_prefix = FLAGS_agent_prefix[0];
  for (const auto& base_factor : old_factors) {
    if (base_factor->keys().size() != 1) {
      factors.push_back(base_factor);
      continue;
    }

    const gtsam::Symbol symb1(base_factor->keys()[0]);
    const gtsam::Symbol symb2(base_factor->keys()[1]);
    if (symb1.chr() != agent_prefix || symb2.chr() != agent_prefix) {
      factors.push_back(base_factor);
      continue;
    }

    const auto& factor = dynamic_cast<const PoseFactor&>(*base_factor);

    if (std::abs(static_cast<int64_t>(symb1.index()) -
                 static_cast<int64_t>(symb2.index())) > 1) {
      LOG(INFO) << "Loop closure: " << gtsam::Symbol(factor.keys()[0]) << " -> "
                << gtsam::Symbol(factor.keys()[1]);
      factors.emplace_shared<PoseFactor>(
          factor.keys()[0], factor.keys()[1], factor.measured(), lc_noise);
    } else {
      factors.emplace_shared<PoseFactor>(
          factor.keys()[0], factor.keys()[1], factor.measured(), odom_noise);
    }
  }
}

int main(int argc, char* argv[]) {
  FLAGS_minloglevel = 0;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::SetUsageMessage("utility for optimizing a trajectory again");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  if (FLAGS_result_dir == "") {
    LOG(FATAL) << "result directory argument required!";
    return 1;
  }

  std::string config_path(FLAGS_config_file);
  YAML::Node config = YAML::LoadFile(config_path);

  RobustSolverParams params;
  params.setPcmSimple3DParams(config["odom_trans_threshold"].as<double>(),
                              config["odom_rot_threshold"].as<double>(),
                              config["pcm_trans_threshold"].as<double>(),
                              config["pcm_rot_threshold"].as<double>(),
                              KimeraRPGO::Verbosity::UPDATE);
  params.setGncInlierCostThresholdsAtProbability(
      config["gnc_alpha"].as<double>(),
      config["gnc_max_iterations"].as<size_t>(),
      config["gnc_mu_step"].as<double>(),
      config["gnc_cost_tolerance"].as<double>(),
      config["gnc_weight_tolerance"].as<double>(),
      config["gnc_fix_prev_inliers"].as<bool>());

  gtsam::Values values;
  gtsam::NonlinearFactorGraph factors, old_factors;
  if (FLAGS_use_g2o) {
    load_factors_g2o(old_factors, values);
  } else {
    load_factors_pgmo(params, factors, values);
  }

  remap_covariances(config, old_factors, factors);
  add_priors_from_config(config, values, factors);
  add_bounds_from_config(config, values, factors);
  add_manual_loop_clousres(config, values, factors);

  RobustSolver solver(params);
  solver.forceUpdate(factors, values);

  auto opt_values = solver.calculateEstimate();
  std::cout << "Finished!" << '\a' << std::endl;

  output(factors, opt_values);

  return 0;
}
