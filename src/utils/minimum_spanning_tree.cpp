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
#include "hydra/utils/minimum_spanning_tree.h"

#include <glog/logging.h>

#include "hydra/utils/disjoint_set.h"

namespace hydra {

MinimumSpanningTreeInfo getMinimumSpanningEdges(const SceneGraphLayer& layer) {
  EdgeFilter filter;
  return getMinimumSpanningEdges(layer, filter);
}

// implementation mainly from: https://en.wikipedia.org/wiki/Kruskal%27s_algorithm
MinimumSpanningTreeInfo getMinimumSpanningEdges(const SceneGraphLayer& layer,
                                                const EdgeFilter& filter) {
  std::vector<MinimalEdge> sorted_edges;
  sorted_edges.reserve(layer.edges().size());
  for (const auto& id_edge_pair : layer.edges()) {
    const auto& edge = id_edge_pair.second;
    if (filter && !filter(layer, edge)) {
      continue;
    }

    sorted_edges.emplace_back(
        edge.source,
        edge.target,
        (layer.getPosition(edge.source) - layer.getPosition(edge.target)).norm());
  }
  std::make_heap(sorted_edges.begin(), sorted_edges.end(), std::greater<>{});

  MinimumSpanningTreeInfo info;
  info.edges.reserve(sorted_edges.size());
  for (const auto& id_node_pair : layer.nodes()) {
    info.counts[id_node_pair.first] = 0;
  }

  DisjointSet subtrees(layer);
  while (!sorted_edges.empty()) {
    std::pop_heap(sorted_edges.begin(), sorted_edges.end(), std::greater<>{});
    auto min_edge = sorted_edges.back();
    sorted_edges.pop_back();

    NodeId lhs_set = subtrees.findSet(min_edge.source);
    NodeId rhs_set = subtrees.findSet(min_edge.target);
    if (lhs_set != rhs_set) {
      subtrees.doUnion(lhs_set, rhs_set);
      info.edges.push_back(min_edge);
      info.counts[min_edge.source]++;
      info.counts[min_edge.target]++;
    }
  }

  for (const auto& id_count_pair : info.counts) {
    if (id_count_pair.second == 1) {
      info.leaves.insert(id_count_pair.first);
    }
  }

  return info;
}

}  // namespace hydra
