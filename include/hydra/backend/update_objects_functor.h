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
#include <config_utilities/factory.h>

#include "hydra/backend/association_strategies.h"
#include "hydra/backend/merge_tracker.h"
#include "hydra/backend/update_functions.h"
#include "hydra/utils/active_window_tracker.h"

namespace hydra {

struct UpdateObjectsFunctor : public UpdateFunctor {
  struct Config {
    //! Allow mesh vertices for each object to be merged
    bool allow_connection_merging = true;
    //! Association strategy for finding matches to active nodes
    MergeProposer::Config merge_proposer = {config::VirtualConfig<AssociationStrategy>{
        association::SemanticNearestNode::Config{}}};
    // If set (>0), use cognition labels for merging. Scores have to be large than this
    // value to merge.
    double cognition_similarity_threshold = 0.0;
  } const config;

  explicit UpdateObjectsFunctor(const Config& config);
  Hooks hooks() const override;
  void call(const DynamicSceneGraph& unmerged,
            SharedDsgInfo& dsg,
            const UpdateInfo::ConstPtr& info) const override;
  MergeList findMerges(const DynamicSceneGraph& graph,
                       const UpdateInfo::ConstPtr& info) const;

  void mergeAttributes(const DynamicSceneGraph& layer, NodeId from, NodeId to) const;

  mutable ActiveWindowTracker active_tracker;
  const MergeProposer merge_proposer;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<UpdateFunctor, UpdateObjectsFunctor, Config>(
          "UpdateObjectsFunctor");
};

void declare_config(UpdateObjectsFunctor::Config& config);

}  // namespace hydra
