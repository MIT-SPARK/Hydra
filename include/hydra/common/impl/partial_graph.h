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
#include "hydra/common/partial_graph.h"

namespace hydra {

template <typename AttrT>
bool PartialGraph<AttrT>::add(NodeId node_id, NodeAttr&& attrs) {
  deleted_nodes_.erase(node_id);
  return nodes_.emplace(node_id, Node{std::move(attrs), {}}).second;
}

template <typename AttrT>
bool PartialGraph<AttrT>::add(NodeId source, NodeId target, EdgeAttr&& attrs) {
  allocate(source).neighbors.insert(target);
  allocate(target).neighbors.insert(source);

  const spark_dsg::EdgeKey key{source, target};
  deleted_edges_.erase(key);
  return edges_.emplace(key, std::move(attrs)).second;
}

template <typename AttrT>
void PartialGraph<AttrT>::update(NodeId node_id, NodeAttr&& attrs) {
  auto iter = nodes_.find(node_id);
  if (iter == nodes_.end()) {
    add(node_id, std::move(attrs));
    return;
  }

  iter->second.attrs = std::move(attrs);
}

template <typename AttrT>
void PartialGraph<AttrT>::update(NodeId source, NodeId target, EdgeAttr&& attrs) {
  auto iter = edges_.find(spark_dsg::EdgeKey{source, target});
  if (iter == edges_.end()) {
    add(source, target, std::move(attrs));
  }

  iter->second = std::move(attrs);
}

template <typename AttrT>
void PartialGraph<AttrT>::archive(NodeId node_id) {
  auto node = find(node_id);
  if (node) {
    node->attrs.reset();  // archived nodes point to nothing
  }
}

template <typename AttrT>
void PartialGraph<AttrT>::remove(NodeId node_id) {
  erase(nodes_.find(node_id));
}

template <typename AttrT>
void PartialGraph<AttrT>::remove(NodeId source, NodeId target) {
  const spark_dsg::EdgeKey key{source, target};
  edges_.erase(key);
  deleted_edges_.insert(key);

  auto source_node = find(source);
  if (source_node) {
    source_node->neighbors.erase(target);
  }

  auto target_node = find(target);
  if (target_node) {
    target_node->neighbors.erase(source);
  }
}

template <typename AttrT>
auto PartialGraph<AttrT>::erase(const typename Nodes::iterator& iter) ->
    typename Nodes::iterator {
  if (iter == nodes_.end()) {
    return iter;
  }

  for (const auto& neighbor : iter->second.neighbors) {
    const spark_dsg::EdgeKey key{iter->first, neighbor};
    edges_.erase(key);
    nodes_.at(neighbor).neighbors.erase(iter->first);
    deleted_edges_.insert(key);
  }

  deleted_nodes_.insert(iter->first);
  return nodes_.erase(iter);
}

template <typename AttrT>
void PartialGraph<AttrT>::contract(NodeId from, NodeId to) {
  auto iter = nodes_.find(from);
  if (iter == nodes_.end()) {
    return;
  }

  for (const auto& neighbor : iter->second.neighbors) {
    if (!edges_.count(spark_dsg::EdgeKey{to, neighbor})) {
      add(to, neighbor, std::move(edges_.find(from, neighbor)->info));
    }
  }

  erase(iter);
}

template <typename AttrT>
void PartialGraph<AttrT>::prune() {
  auto iter = nodes_.begin();
  while (iter != nodes_.end()) {
    if (iter->second.attrs) {
      ++iter;
      continue;  // skip active nodes
    }

    bool can_prune = true;
    for (const auto& neighbor : iter->second.neighbors) {
      if (nodes_.at(neighbor).attrs) {
        can_prune = false;
        break;
      }
    }

    if (can_prune) {
      iter = erase(iter);
    } else {
      ++iter;
    }
  }

  deleted_nodes_.clear();
  deleted_edges_.clear();
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
std::set<spark_dsg::NodeId> PartialGraph<AttrT>::neighbors(NodeId node) const {
  auto iter = nodes_.find(node);
  return iter == nodes_.end() ? std::set<NodeId>{} : iter->second.neighbors;
}

template <typename AttrT>
AttrT* PartialGraph<AttrT>::at(NodeId node) const {
  auto iter = nodes_.find(node);
  return iter == nodes_.end() ? nullptr : iter->second.attrs.get();
}

template <typename AttrT>
spark_dsg::EdgeAttributes* PartialGraph<AttrT>::at(NodeId source, NodeId target) const {
  auto iter = edges_.find(spark_dsg::EdgeKey{source, target});
  return iter == edges_.end() ? nullptr : iter->second.get();
}

template <typename AttrT>
auto PartialGraph<AttrT>::find(NodeId node) -> typename PartialGraph::Node* {
  auto iter = nodes_.find(node);
  return iter == nodes_.end() ? nullptr : &iter->second;
}

template <typename AttrT>
auto PartialGraph<AttrT>::allocate(NodeId node) -> typename PartialGraph::Node& {
  auto iter = nodes_.find(node);
  if (iter == nodes_.end()) {
    iter = nodes_.emplace(node, Node{nullptr, {}}).first;
    deleted_nodes_.erase(node);
  }

  return iter->second;
}

}  // namespace hydra
