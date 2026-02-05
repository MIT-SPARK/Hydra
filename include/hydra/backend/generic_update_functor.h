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
#pragma once

#include <config_utilities/virtual_config.h>

#include "hydra/backend/association_strategies.h"
#include "hydra/backend/deformation_interpolator.h"
#include "hydra/backend/merge_tracker.h"
#include "hydra/backend/update_functions.h"
#include "hydra/common/node_matchers.h"
#include "hydra/utils/active_window_tracker.h"
#include "hydra/utils/logging.h"

namespace hydra {

struct GenericUpdateFunctor : public UpdateFunctor {
  struct Config : VerbosityConfig {
    //! Layer to update
    std::string layer;
    //! Enable merging for this update functor
    bool enable_merging = true;
    //! Settings for deformation of the places from the deformation graph
    DeformationInterpolator::Config deformation_interpolator = {};
    //! Validator of association between two nodes
    config::VirtualConfig<NodeMatcher> matcher{DistanceNodeMatcher::Config{}};
    //! Association strategy for finding matches to active nodes
    MergeProposer::Config merge_proposer = {
        config::VirtualConfig<AssociationStrategy>{association::NearestNode::Config{}}};
  } const config;

  explicit GenericUpdateFunctor(const Config& config);
  Hooks hooks() const override;
  void call(const DynamicSceneGraph& unmerged,
            SharedDsgInfo& dsg,
            const UpdateInfo::ConstPtr& info) const override;

  MergeList findMerges(const DynamicSceneGraph& graph,
                       const UpdateInfo::ConstPtr& info) const;

  std::optional<NodeId> proposeMerge(const SceneGraphLayer& layer,
                                     const SceneGraphNode& node) const;

  mutable ActiveWindowTracker active_tracker;
  const std::unique_ptr<NodeMatcher> node_matcher;
  const MergeProposer merge_proposer;
  const DeformationInterpolator deformation_interpolator;
};

void declare_config(GenericUpdateFunctor::Config& config);

}  // namespace hydra
