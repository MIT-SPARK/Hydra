#pragma once
#include <gtsam/geometry/Pose3.h>
#include <kimera_pgmo/deformation_graph.h>

#include <map>
#include <memory>
#include <set>

#include "hydra_multi/common/types.h"

namespace hydra_multi {

class InitialAlignModule {
 public:
  // Types.
  using Ptr = std::unique_ptr<InitialAlignModule>;

  struct LoopClosure {
    gtsam::Key from;
    gtsam::Key to;
    gtsam::Pose3 from_T_to;
  };

  struct Info {
    using Ptr = std::shared_ptr<InitialAlignModule::Info>;
    virtual ~Info() = default;
    std::shared_ptr<const kimera_pgmo::DeformationGraph> dgraph;
    std::vector<LoopClosure> loop_closures;
    // The W_T_robot_ used to transform the graphs
    std::map<size_t, gtsam::Pose3> prev_W_T_robot;

    gtsam::Pose3 getPrevRobotPose(size_t robot_id) const;
  };

  struct Config {
    double dgraph_consistency_tol = 5.0;  // Recompute if PGO result defers by a lot
    int reference_robot_id =
        -1;  // Robot to use as reference frame (-1 = use lowest ID)
  } config;

  InitialAlignModule() = default;

  virtual ~InitialAlignModule() = default;

  virtual void update(InitialAlignModule::Info::Ptr info);

  inline std::map<size_t, gtsam::Pose3> getFrames() const { return W_T_robot_; }

  inline MultiTransforms getPairwiseTransforms() const { return pairwise_transforms_; }

  gtsam::Values computeInitialGuess(InitialAlignModule::Info::Ptr info,
                                    const gtsam::Values& input_values,
                                    const NodeIdRobotMap& node_id_robot_map);

  bool checkAlignConsistent(InitialAlignModule::Info::Ptr info);

 protected:
  MultiTransforms pairwise_transforms_;
  std::set<RobotId> all_robots_;
  std::map<RobotId, gtsam::Pose3> W_T_robot_;
};
void declare_config(InitialAlignModule::Config&);

}  // namespace hydra_multi
