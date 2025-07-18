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

#include "hydra/backend/merge_proposer.h"

#include <config_utilities/config.h>
#include <glog/logging.h>

namespace hydra {

namespace {

inline NodeId getRemappedNode(const std::map<NodeId, NodeId>& remapping, NodeId node) {
  auto iter = remapping.find(node);
  return iter == remapping.end() ? node : iter->second;
}

}  // namespace

Merge Merge::remap(const std::map<NodeId, NodeId>& remapping) const {
  return {getRemappedNode(remapping, from), getRemappedNode(remapping, to)};
}

std::ostream& operator<<(std::ostream& out, const Merge& merge) {
  out << NodeSymbol(merge.from).str() << "  -> " << NodeSymbol(merge.to).str();
  return out;
}

void declare_config(MergeProposer::Config& config) {
  using namespace config;
  name("MergeProposer::Config");
  config.strategy.setOptional();
  field(config.strategy, "strategy");
}

void MergeProposer::findMerges(const SceneGraphLayer& layer,
                               const LayerView& view,
                               const MergeCheck& should_merge,
                               MergeList& nodes_to_merge) const {
  const auto associator = config.strategy.create<const SceneGraphLayer&>(layer);
  if (!associator) {
    LOG(WARNING) << "Merges enabled, but factory not specified!";
    return;
  }

  for (const auto& node : view) {
    const auto candidates = associator->candidates(layer, node);
    if (!candidates) {
      continue;
    }

    for (const auto& other : candidates) {
      if (layer.hasEdge(node.id, other.id)) {
        continue;  // avoid merging siblings
      }

      if (should_merge(node, other)) {
        nodes_to_merge.push_back({node.id, other.id});
      }
    }
  }
}

}  // namespace hydra
