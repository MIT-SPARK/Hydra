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
#include "hydra/backend/merge_tracker.h"

#include <glog/logging.h>

namespace hydra {

void MergeTracker::applyMerges(const DynamicSceneGraph& unmerged,
                               const MergeList& proposals,
                               SharedDsgInfo& dsg,
                               const MergeFunc& merge_attrs) {
  auto& prior_merges = dsg.merges;

  auto& graph = *dsg.graph;
  std::set<NodeId> to_update;
  for (const auto& orig_merge : proposals) {
    const auto merge = orig_merge.remap(prior_merges);
    if (merge.from == merge.to) {
      VLOG(10) << "Found present merge: " << orig_merge;
      to_update.insert(merge.to);
      continue;
    }

    if (!graph.mergeNodes(merge.from, merge.to)) {
      LOG(WARNING) << "Failed to apply merge: " << merge << " (original: " << orig_merge
                   << ", from: " << std::boolalpha << graph.hasNode(merge.from)
                   << ", to: " << graph.hasNode(merge.to) << ")";
      continue;
    }

    VLOG(5) << "Applied merge: " << merge << " (original: " << orig_merge << ")";

    to_update.insert(merge.to);
    updateParents(prior_merges, merge);
  }

  if (!merge_attrs) {
    return;
  }

  for (const auto& node : to_update) {
    auto iter = merge_sets_.find(node);
    if (iter == merge_sets_.end()) {
      continue;
    }

    auto child_iter = iter->second.begin();
    while (child_iter != iter->second.end()) {
      if (!unmerged.hasNode(*child_iter)) {
        child_iter = iter->second.erase(child_iter);
      } else {
        ++child_iter;
      }
    }

    std::vector<NodeId> nodes{node};
    nodes.insert(nodes.end(), iter->second.begin(), iter->second.end());
    graph.setNodeAttributes(node, merge_attrs(unmerged, nodes));
  }
}

void MergeTracker::clear() { merge_sets_.clear(); }

void MergeTracker::updateParents(std::map<NodeId, NodeId>& prior_merges,
                                 const Merge& merge) {
  // look up (and maybe initialize new parent)
  auto to_iter = merge_sets_.find(merge.to);
  if (to_iter == merge_sets_.end()) {
    to_iter = merge_sets_.emplace(merge.to, std::set<NodeId>()).first;
  }

  // record merge parent
  to_iter->second.insert(merge.from);
  prior_merges[merge.from] = merge.to;

  // find prior merges
  auto from_iter = merge_sets_.find(merge.from);
  if (from_iter == merge_sets_.end()) {
    return;  // no previous merges
  }

  // rewire old merges
  for (const auto& child : from_iter->second) {
    prior_merges[child] = merge.to;
    to_iter->second.insert(child);
  }

  merge_sets_.erase(from_iter);
}

}  // namespace hydra
