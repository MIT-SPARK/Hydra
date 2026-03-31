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
#include <spark_dsg/edge_attributes.h>
#include <spark_dsg/edge_container.h>
#include <spark_dsg/node_attributes.h>

#include <set>

namespace hydra {

template <typename AttrT>
class PartialGraph {
 public:
  using NodeId = spark_dsg::NodeId;
  using NodeAttr = std::unique_ptr<AttrT>;
  using EdgeAttr = spark_dsg::EdgeAttributes::Ptr;
  struct Node {
    NodeAttr attrs;
    std::set<NodeId> neighbors;
  };

  void add(NodeId node_id, NodeAttr&& attrs = nullptr);

  void add(NodeId source, NodeId target, EdgeAttr&& attrs = nullptr);

  void update(NodeId node_id, NodeAttr&& attrs = nullptr);

  void update(NodeId source, NodeId target, EdgeAttr&& attrs = nullptr);

  void remove(NodeId node_id);

  void remove(NodeId source, NodeId target);

  bool has(NodeId node) const;

  bool has(NodeId source, NodeId target) const;

  std::set<spark_dsg::NodeId> neighbors(NodeId node) const;

  AttrT* at(NodeId node) const;

  void contract(NodeId from, NodeId to);

  const std::map<NodeId, Node>& nodes() const { return nodes_; }

 private:
  Node* find(NodeId node);

  Node& allocate(NodeId node);

  std::map<NodeId, Node> nodes_;
  spark_dsg::EdgeContainer edges_;
};

template <typename AttrT>
void PartialGraph<AttrT>::add(NodeId node_id, NodeAttr&& attrs) {
  nodes_.emplace(node_id, Node{std::move(attrs), {}});
}

template <typename AttrT>
void PartialGraph<AttrT>::add(NodeId source, NodeId target, EdgeAttr&& attrs) {
  edges_.insert(source, target, std::move(attrs));

  auto& source_node = allocate(source);
  source_node.neighbors.insert(target);

  auto& target_node = allocate(target);
  target_node.neighbors.insert(source);
}

template <typename AttrT>
void PartialGraph<AttrT>::update(NodeId node_id, NodeAttr&& attrs) {
  auto iter = nodes_.find(node_id);
  if (iter == nodes_.end()) {
    add(node_id, std::move(attrs));
  }

  iter->second.attrs = std::move(attrs);
}

template <typename AttrT>
void PartialGraph<AttrT>::update(NodeId source, NodeId target, EdgeAttr&& attrs) {
  auto edge = edges_.find(source, target);
  if (!edge) {
    add(source, target, std::move(attrs));
  }

  edge->info = std::move(attrs);
}

template <typename AttrT>
void PartialGraph<AttrT>::remove(NodeId node_id) {
  nodes_.remove(node_id);
}

template <typename AttrT>
void PartialGraph<AttrT>::remove(NodeId source, NodeId target) {
  edges_.remove(source, target);
  auto source_node = find(source);
  if (source_node) {
    source_node->neighbors.remove(source);
  }

  auto target_node = find(target);
  if (target_node) {
    target_node->neighbors.remove(target_node);
  }
}

template <typename AttrT>
bool PartialGraph<AttrT>::has(NodeId node_id) const {
  return nodes_.count(node_id);
}

template <typename AttrT>
bool PartialGraph<AttrT>::has(NodeId source, NodeId target) const {
  return edges_.contains(source, target);
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
void PartialGraph<AttrT>::contract(NodeId from, NodeId to) {
  auto iter = nodes_.find(from);
  if (iter == nodes_.end()) {
    return;
  }

  for (const auto& sibling : iter->second.neighbors) {
    if (!edges_.contains(to, sibling)) {
      add(to, sibling, std::move(edges_.find(from, sibling)->info));
    }
  }

  nodes_.erase(iter);
}

template <typename AttrT>
auto PartialGraph<AttrT>::allocate(NodeId node) -> typename PartialGraph::Node& {
  auto iter = nodes_.find(node);
  if (iter == nodes_.end()) {
    iter = nodes_.emplace(node, Node{nullptr, {}}).first;
  }

  return iter->second;
}

template <typename AttrT>
auto PartialGraph<AttrT>::find(NodeId node) -> typename PartialGraph::Node* {
  auto iter = nodes_.find(node);
  return iter == nodes_.end() ? nullptr : &iter->second;
}

}  // namespace hydra
