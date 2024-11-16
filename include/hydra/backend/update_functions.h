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
#include <gtsam/nonlinear/Values.h>

#include "hydra/backend/merge_proposer.h"
#include "hydra/common/dsg_types.h"
#include "hydra/common/shared_dsg_info.h"

namespace kimera_pgmo {
class DeformationGraph;
}

namespace hydra {

struct UpdateInfo {
  using Ptr = std::shared_ptr<UpdateInfo>;
  using ConstPtr = std::shared_ptr<const UpdateInfo>;
  using LayerMerges = std::map<LayerId, MergeList>;

  const gtsam::Values* places_values = nullptr;
  const gtsam::Values* pgmo_values = nullptr;
  bool loop_closure_detected = false;
  uint64_t timestamp_ns = 0;
  //! External merges (e.g., from GNC)
  LayerMerges given_merges;
  const gtsam::Values* complete_agent_values = nullptr;
  // TODO(nathan) flip to const when we have mutable state
  kimera_pgmo::DeformationGraph* deformation_graph = nullptr;
  const std::unordered_map<NodeId, size_t>* node_to_robot_id = nullptr;
};

using LayerCleanupFunc =
    std::function<void(const UpdateInfo::ConstPtr&, SharedDsgInfo*)>;
using MergeFunc = std::function<NodeAttributes::Ptr(const DynamicSceneGraph&,
                                                    const std::vector<NodeId>&)>;

struct UpdateFunctor {
  using Ptr = std::shared_ptr<UpdateFunctor>;

  struct Hooks {
    LayerCleanupFunc cleanup;
    MergeFunc merge;
  };

  virtual ~UpdateFunctor() = default;
  virtual Hooks hooks() const;
  virtual void call(const DynamicSceneGraph& unmerged,
                    SharedDsgInfo& dsg,
                    const UpdateInfo::ConstPtr& info) const = 0;
  virtual MergeList findMerges(const DynamicSceneGraph& graph,
                               const UpdateInfo::ConstPtr& info) const;
};

}  // namespace hydra
