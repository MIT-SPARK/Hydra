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
#include <spark_dsg/node_symbol.h>

#include <deque>

#include "hydra/common/partial_graph.h"

namespace hydra {

template <typename AttrT>
auto PartialGraph<AttrT>::add(NodeId node_id, NodeAttrPtr&& attrs) -> NodeAttr& {
  deleted_nodes_.erase(node_id);
  auto& node = allocate(node_id);
  if (attrs) {
    node.attrs_ = std::move(attrs);
  }

  return *node.attrs_;
}

template <typename AttrT>
auto PartialGraph<AttrT>::add(NodeId source, NodeId target, EdgeAttrPtr&& attrs)
    -> EdgeAttr& {
  allocate(source).neighbors.insert(target);
  allocate(target).neighbors.insert(source);

  const spark_dsg::EdgeKey key{source, target};
  deleted_edges_.erase(key);
  auto iter = edges_.find(key);
  if (iter == edges_.end()) {
    iter = edges_.emplace(key, std::make_unique<EdgeAttr>()).first;
  }

  if (attrs) {
    iter->second = std::move(attrs);
  }

  return *iter->second;
}

template <typename AttrT>
void PartialGraph<AttrT>::remove(NodeId node_id, bool ignore_archive) {
  erase(nodes_.find(node_id), ignore_archive);
}

template <typename AttrT>
void PartialGraph<AttrT>::remove(NodeId source, NodeId target, bool ignore_archive) {
  erase(edges_.find({source, target}), ignore_archive);
}

template <typename AttrT>
auto PartialGraph<AttrT>::erase(const typename Nodes::iterator& iter,
                                bool ignore_archive) -> typename Nodes::iterator {
  if (iter == nodes_.end()) {
    return iter;
  }

  const auto active = iter->second.attributes().is_active;
  for (const auto& neighbor : iter->second.neighbors) {
    const spark_dsg::EdgeKey key{iter->first, neighbor};
    edges_.erase(key);
    nodes_.at(neighbor).neighbors.erase(iter->first);
    if (active || ignore_archive) {
      deleted_edges_.insert(key);
    }
  }

  if (active || ignore_archive) {
    deleted_nodes_.insert(iter->first);
  }

  return nodes_.erase(iter);
}

template <typename AttrT>
auto PartialGraph<AttrT>::erase(const typename Edges::iterator& iter,
                                bool ignore_archive) -> typename Edges::iterator {
  if (iter == edges_.end()) {
    return iter;
  }

  const auto [source, target] = iter->first;

  bool active = false;
  auto source_node = nodes_.find(source);
  if (source_node != nodes_.end()) {
    source_node->second.neighbors.erase(target);
    active |= source_node->second.attributes().is_active;
  }

  auto target_node = nodes_.find(target);
  if (target_node != nodes_.end()) {
    target_node->second.neighbors.erase(source);
    active |= target_node->second.attributes().is_active;
  }

  if (active || ignore_archive) {
    deleted_edges_.insert(iter->first);
  }

  return edges_.erase(iter);
}

template <typename AttrT>
bool PartialGraph<AttrT>::has(NodeId node_id) const {
  return nodes_.count(node_id);
}

template <typename AttrT>
bool PartialGraph<AttrT>::has(NodeId source, NodeId target) const {
  return edges_.count(spark_dsg::EdgeKey{source, target});
}

template <typename AttrT>
auto PartialGraph<AttrT>::find(NodeId node) -> NodeAttr* {
  auto iter = nodes_.find(node);
  return iter == nodes_.end() ? nullptr : iter->second.attrs_.get();
}

template <typename AttrT>
auto PartialGraph<AttrT>::find(NodeId node) const -> const NodeAttr* {
  return const_cast<PartialGraph<AttrT>*>(this)->find(node);
}

template <typename AttrT>
auto PartialGraph<AttrT>::find(NodeId source, NodeId target) -> EdgeAttr* {
  auto iter = edges_.find(spark_dsg::EdgeKey{source, target});
  return iter == edges_.end() ? nullptr : iter->second.get();
}

template <typename AttrT>
auto PartialGraph<AttrT>::find(NodeId source, NodeId target) const -> const EdgeAttr* {
  return const_cast<PartialGraph<AttrT>*>(this)->find(source, target);
}

template <typename AttrT>
auto PartialGraph<AttrT>::at(NodeId node) -> NodeAttr& {
  auto attrs = find(node);
  if (!attrs) {
    throw std::out_of_range("Missing node '" + spark_dsg::NodeSymbol(node).str() + "'");
  }

  return *attrs;
}

template <typename AttrT>
auto PartialGraph<AttrT>::at(NodeId node) const -> const NodeAttr& {
  return const_cast<PartialGraph<AttrT>*>(this)->at(node);
}

template <typename AttrT>
auto PartialGraph<AttrT>::at(NodeId source, NodeId target) -> EdgeAttr& {
  auto attrs = find(source, target);
  if (!attrs) {
    throw std::out_of_range("Missing edge '" + spark_dsg::NodeSymbol(source).str() +
                            "' -> '" + spark_dsg::NodeSymbol(target).str() + "'");
  }

  return *attrs;
}

template <typename AttrT>
auto PartialGraph<AttrT>::at(NodeId source, NodeId target) const -> const EdgeAttr& {
  return const_cast<PartialGraph<AttrT>*>(this)->at(source, target);
}

template <typename AttrT>
void PartialGraph<AttrT>::archive(NodeId node) {
  at(node).is_active = false;
}

template <typename AttrT>
std::set<spark_dsg::NodeId> PartialGraph<AttrT>::neighbors(NodeId node) const {
  auto iter = nodes_.find(node);
  return iter == nodes_.end() ? std::set<NodeId>{} : iter->second.neighbors;
}

template <typename AttrT>
void PartialGraph<AttrT>::contract(NodeId from, NodeId to) {
  auto iter = nodes_.find(from);
  if (iter == nodes_.end()) {
    return;
  }

  if (!nodes_.count(to)) {
    return;
  }

  for (const auto& neighbor : iter->second.neighbors) {
    if (neighbor == to) {
      continue;
    }

    if (!edges_.count(spark_dsg::EdgeKey{to, neighbor})) {
      add(to, neighbor, std::move(edges_.at({from, neighbor})));
    }
  }

  erase(iter);
}

template <typename AttrT>
std::vector<uint64_t> PartialGraph<AttrT>::prune() {
  std::vector<uint64_t> pruned;
  auto iter = nodes_.begin();
  while (iter != nodes_.end()) {
    if (!iter->second.archived()) {
      ++iter;
      continue;  // skip active nodes
    }

    bool can_prune = true;
    for (const auto& neighbor : iter->second.neighbors) {
      if (!nodes_.at(neighbor).archived()) {
        can_prune = false;
        break;
      }
    }

    if (can_prune) {
      pruned.push_back(iter->first);
      iter = erase(iter);
    } else {
      ++iter;
    }
  }

  deleted_nodes_.clear();
  deleted_edges_.clear();
  return pruned;
}

template <typename AttrT>
auto PartialGraph<AttrT>::connected_components(bool sort_components) const
    -> Components {
  Components components;
  std::unordered_set<NodeId> visited;
  for (const auto& [seed, _] : nodes_) {
    if (visited.count(seed)) {
      continue;
    }

    visited.insert(seed);
    std::deque<NodeId> frontier{seed};
    auto& component = components.emplace_back();
    while (!frontier.empty()) {
      const auto curr_id = frontier.front();
      frontier.pop_front();
      component.push_back(curr_id);
      for (const auto neighbor : nodes_.at(curr_id).neighbors) {
        if (visited.count(neighbor)) {
          continue;
        }

        frontier.push_back(neighbor);
        visited.insert(neighbor);
      }
    }
  }

  if (sort_components) {
    std::sort(components.begin(),
              components.end(),
              [](const auto& lhs, const auto& rhs) { return lhs.size() > rhs.size(); });
  }

  return components;
}

template <typename AttrT>
auto PartialGraph<AttrT>::allocate(NodeId node) -> typename PartialGraph::Node& {
  auto iter = nodes_.find(node);
  if (iter == nodes_.end()) {
    iter = nodes_.emplace(node, Node()).first;
    deleted_nodes_.erase(node);
  }

  return iter->second;
}

}  // namespace hydra
