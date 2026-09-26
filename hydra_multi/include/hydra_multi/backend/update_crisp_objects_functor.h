#pragma once
#include <config_utilities/factory.h>
#include <hydra/backend/association_strategies.h>
#include <hydra/backend/merge_tracker.h>
#include <hydra/backend/update_functions.h>
#include <hydra/utils/active_window_tracker.h>

#include "hydra_multi/common/types.h"
namespace hydra_multi {

using hydra::MergeList;
using hydra::SharedDsgInfo;
using hydra::UpdateInfo;

struct UpdateCrispObjectsFunctor : public hydra::UpdateFunctor {
  struct Config {
    //! Allow feature for each object to be merged
    bool allow_feature_merging = true;
    //! Require merges to have same semantic label
    bool merge_require_same_label = true;
    //! Require a check on the shape vectors
    bool check_feature_similarity = true;
    //! min feature similarity
    float min_feature_similarity = 0.5;
    //! Association strategy for finding matches to active nodes
    hydra::MergeProposer::Config merge_proposer = {
        config::VirtualConfig<hydra::AssociationStrategy>{
            hydra::association::SemanticNearestNode::Config{}}};
  } const config;

  explicit UpdateCrispObjectsFunctor(const Config& config);
  Hooks hooks() const override;
  void call(const DynamicSceneGraph& unmerged,
            SharedDsgInfo& dsg,
            const UpdateInfo::ConstPtr& info) const override;

  MergeList findMerges(const DynamicSceneGraph& graph,
                       const UpdateInfo::ConstPtr& info) const;

  const hydra::MergeProposer merge_proposer;

  mutable hydra::ActiveWindowTracker active_tracker;
  mutable std::unordered_map<NodeId, Eigen::Vector3d> cached_pos_;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<UpdateFunctor, UpdateCrispObjectsFunctor, Config>(
          "UpdateCrispObjectsFunctor");
};

void declare_config(UpdateCrispObjectsFunctor::Config& config);

}  // namespace hydra_multi
