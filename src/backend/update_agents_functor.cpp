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
#include "hydra/backend/update_agents_functor.h"

#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>

#include "hydra/utils/timing_utilities.h"

namespace hydra {

using timing::ScopedTimer;

MergeList UpdateAgentsFunctor::call(const DynamicSceneGraph&,
                                    SharedDsgInfo& dsg,
                                    const UpdateInfo::ConstPtr& info) const {
  if (!info->complete_agent_values || info->complete_agent_values->size() == 0) {
    return {};
  }

  ScopedTimer timer("backend/agent_update", info->timestamp_ns, true, 1, false);
  auto& graph = *dsg.graph;
  const auto desired_layer = DsgLayers::AGENTS;
  for (const auto& prefix_layer_pair : graph.dynamicLayersOfType(desired_layer)) {
    std::set<NodeId> missing_nodes;
    for (const auto& node : prefix_layer_pair.second->nodes()) {
      auto& attrs = node->attributes<AgentNodeAttributes>();
      if (!info->complete_agent_values->exists(attrs.external_key)) {
        missing_nodes.insert(node->id);
        continue;
      }

      auto pose = info->complete_agent_values->at<gtsam::Pose3>(attrs.external_key);
      attrs.position = pose.translation();
      attrs.world_R_body = Eigen::Quaterniond(pose.rotation().matrix());
    }

    if (!missing_nodes.empty()) {
      LOG(WARNING) << "Layer " << DsgLayers::AGENTS << "(" << prefix_layer_pair.first
                   << "): could not update "
                   << displayNodeSymbolContainer(missing_nodes);
    }
  }

  return {};
}

}  // namespace hydra
