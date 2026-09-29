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

#include <algorithm>
#include <deque>
#include <stdexcept>
#include <unordered_set>

#include "hydra/common/partial_graph.h"

namespace hydra {

template <typename AttrT>
auto PartialGraph<AttrT>::add(NodeId node_id, NodeAttrPtr&& attrs) -> NodeAttr& {
  deleted_nodes_.erase(node_id);
  finalized_nodes_.erase(node_id);
  auto& node = nodes_.try_emplace(node_id).first->second;
  node.archived_ = false;
  if (attrs) {
    node.attrs_ = std::move(attrs);
  }

  return node.attributes();
}

template <typename AttrT>
auto PartialGraph<AttrT>::add(NodeId source, NodeId target, EdgeAttrPtr&& attrs)
    -> EdgeAttr& {
  if (source == target) {
    throw std::invalid_argument("PartialGraph edges must have distinct endpoints");
  }

  // Validate both endpoints before modifying either neighbor set.
  auto& source_node = nodes_.at(source);
  auto& target_node = nodes_.at(target);
  source_node.neighbors.insert(target);
  target_node.neighbors.insert(source);

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
void PartialGraph<AttrT>::remove(NodeId node_id) {
  auto iter = nodes_.find(node_id);
  if (iter == nodes_.end()) {
    return;
  }

  for (const auto neighbor : iter->second.neighbors) {
    deleted_edges_.insert({node_id, neighbor});
  }

  deleted_nodes_.insert(node_id);
  finalized_nodes_.erase(node_id);
  eraseNode(iter);
}

template <typename AttrT>
void PartialGraph<AttrT>::remove(NodeId source, NodeId target) {
  auto iter = edges_.find({source, target});
  if (iter == edges_.end()) {
    return;
  }

  deleted_edges_.insert(iter->first);
  eraseEdge(iter);
}

template <typename AttrT>
void PartialGraph<AttrT>::finalize(NodeId node_id) {
  auto iter = nodes_.find(node_id);
  if (iter == nodes_.end()) {
    return;
  }

  if (!canFinalize(node_id)) {
    throw std::logic_error("Cannot finalize a node with active support or neighbors");
  }

  finalized_nodes_.insert(node_id);
  eraseNode(iter);
}

template <typename AttrT>
void PartialGraph<AttrT>::acknowledgeChanges() {
  deleted_nodes_.clear();
  deleted_edges_.clear();
  finalized_nodes_.clear();
}

template <typename AttrT>
void PartialGraph<AttrT>::eraseNode(typename Nodes::iterator iter) {
  while (!iter->second.neighbors.empty()) {
    eraseEdge(edges_.find({iter->first, *iter->second.neighbors.begin()}));
  }

  nodes_.erase(iter);
}

template <typename AttrT>
void PartialGraph<AttrT>::eraseEdge(typename Edges::iterator iter) {
  const auto [source, target] = iter->first;
  nodes_.at(source).neighbors.erase(target);
  nodes_.at(target).neighbors.erase(source);
  edges_.erase(iter);
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
  return iter == nodes_.end() ? nullptr : &iter->second.attributes();
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
  nodes_.at(node).archived_ = true;
}

template <typename AttrT>
bool PartialGraph<AttrT>::archived(NodeId node) const {
  return nodes_.at(node).archived();
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

  if (from == to || !nodes_.count(to) || archived(from) || archived(to)) {
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

  remove(from);
}

template <typename AttrT>
bool PartialGraph<AttrT>::canFinalize(NodeId node) const {
  const auto& entry = nodes_.at(node);
  if (!entry.archived()) {
    return false;
  }

  return std::all_of(entry.neighbors.begin(),
                     entry.neighbors.end(),
                     [this](NodeId neighbor) { return archived(neighbor); });
}

template <typename AttrT>
std::vector<uint64_t> PartialGraph<AttrT>::prune(
    const std::set<NodeId>& retained_nodes) {
  std::vector<uint64_t> pruned;
  for (const auto& [node_id, node] : nodes_) {
    if (!retained_nodes.count(node_id) && canFinalize(node_id)) {
      pruned.push_back(node_id);
    }
  }

  for (const auto node_id : pruned) {
    finalize(node_id);
  }

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

}  // namespace hydra
