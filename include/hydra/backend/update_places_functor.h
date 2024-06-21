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
#include "hydra/backend/merge_tracker.h"
#include "hydra/backend/update_functions.h"
#include "hydra/utils/active_window_tracker.h"
#include "hydra/utils/nearest_neighbor_utilities.h"

namespace hydra {

struct UpdatePlacesFunctor : public UpdateFunctor {
  UpdatePlacesFunctor(double pos_threshold, double distance_tolerance);
  MergeList call(const DynamicSceneGraph& unmerged,
                 SharedDsgInfo& dsg,
                 const UpdateInfo::ConstPtr& info) const override;

  void updatePlace(const gtsam::Values& values,
                   NodeId node,
                   NodeAttributes& attrs) const;

  std::optional<NodeId> proposeMerge(const SceneGraphLayer& layer,
                                     const SceneGraphNode& node) const;

  void filterMissing(DynamicSceneGraph& graph,
                     const std::list<NodeId> missing_nodes) const;

  size_t num_merges_to_consider = 1;
  double pos_threshold_m;
  double distance_tolerance_m;

  mutable ActiveWindowTracker active_tracker;
  mutable std::unique_ptr<NearestNodeFinder> node_finder;
};

}  // namespace hydra
