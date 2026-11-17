#pragma once
#include <config_utilities/factory.h>
#include <gtsam/geometry/Pose3.h>
#include <kimera_pgmo/deformation_graph.h>
#include <kimera_pgmo/optimizer/kimera_rpgo_optimizer.h>

#include <map>

#include "hydra_multi/backend/initial_align.h"
#include "hydra_multi/common/types.h"

namespace hydra_multi {

class LoopClosureInitialAlignModule : public InitialAlignModule {
 public:
  // Types.
  using Ptr = std::unique_ptr<LoopClosureInitialAlignModule>;
  struct Config : InitialAlignModule::Config {
    Config();
    double rot_sigma = 0.1;
    double trans_sigma = 1.0;
    kimera_pgmo::KimeraRpgoOptimizer::Config optimizer;
  } config;

  using LoopClosures = std::vector<LoopClosure>;
  using MultiLoopClosures = std::map<RobotId, std::map<RobotId, LoopClosures>>;

  explicit LoopClosureInitialAlignModule(const Config& config);

  void update(InitialAlignModule::Info::Ptr info) override;

  inline size_t getNumMeasurements() const { return align_num_measurements_; }

  inline size_t getNumInliers() const { return align_num_inliers_; }

  inline MultiLoopClosures getLoopClosures() const { return interrobot_lcs_; }

  bool addLoopClosure(const gtsam::Key& from,
                      const gtsam::Key& to,
                      const gtsam::Pose3& from_T_to);

  void updatePairwiseTransforms(Info::Ptr info);

 private:
  gtsam::Pose3 robustInitialGuess(const RobotId& robot_id);

  void computeInitialTransforms();

  bool checkLoopClosureInitialAlignConsistent(
      const kimera_pgmo::DeformationGraph& dgraph);

 private:
  kimera_pgmo::KimeraRpgoOptimizer optimizer_;
  MultiLoopClosures interrobot_lcs_;
  size_t align_num_measurements_ = 0;
  size_t align_num_inliers_ = 0;
};

void declare_config(LoopClosureInitialAlignModule::Config&);

}  // namespace hydra_multi
