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
#include "hydra/utils/disjoint_set.h"

#include <glog/logging.h>

namespace hydra {

DisjointSet::DisjointSet() {}

// implementation mainly from: https://en.wikipedia.org/wiki/Disjoint-set_data_structure
DisjointSet::DisjointSet(const SceneGraphLayer& layer) {
  for (const auto& id_node_pair : layer.nodes()) {
    addSet(id_node_pair.first);
  }
}

bool DisjointSet::addSet(NodeId node) {
  if (parents.count(node)) {
    return false;
  }

  roots.insert(node);
  parents[node] = node;
  sizes[node] = 1;
  return true;
}

NodeId DisjointSet::findSet(NodeId node) const {
  NodeId parent = node;

  NodeId curr_node;
  do {
    curr_node = parent;
    parent = parents.at(curr_node);
  } while (parent != curr_node);

  return parent;
}

bool DisjointSet::hasSet(NodeId node) const { return parents.count(node); }

std::optional<NodeId> DisjointSet::doUnion(NodeId lhs, NodeId rhs, bool rhs_better) {
  NodeId lhs_set = findSet(lhs);
  NodeId rhs_set = findSet(rhs);

  if (lhs_set == rhs_set) {
    return std::nullopt;
  }

  const auto lhs_size = sizes.at(lhs_set);
  const auto rhs_size = sizes.at(rhs_set);
  if (lhs_size < rhs_size) {
    std::swap(lhs_set, rhs_set);
  } else if (lhs_size == rhs_size && rhs_better) {
    // allow external stable ordering information
    std::swap(lhs_set, rhs_set);
  }

  parents[rhs_set] = lhs_set;
  sizes[lhs_set] = sizes[lhs_set] + sizes[rhs_set];
  sizes.erase(rhs_set);  // |sizes| = number of components
  roots.erase(rhs_set);
  return rhs_set;
}

}  // namespace hydra
