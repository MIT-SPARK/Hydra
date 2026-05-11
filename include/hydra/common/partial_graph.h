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
  using Nodes = std::map<NodeId, Node>;

  /**
   * @brief Add a node to the graph
   * @param node_id Node to add
   * @param attrs Attributes to add with node
   * @return Whether or not the add was successful
   */
  bool add(NodeId node_id, NodeAttr&& attrs = nullptr);

  /**
   * @brief Add a undirected edge to the graph
   * @param source One endpoint of edge to add
   * @param target Other endpoint of edge to add
   * @param attrs Attributes to add with edge
   * @return Whether or not the add was successful
   */
  bool add(NodeId source, NodeId target, EdgeAttr&& attrs = nullptr);

  /**
   * @brief Update the attributes of a node
   * @param node_id Node to update
   * @param attrs New attributes to use for node
   *
   * Allocates the node if it doesn't exist already
   */
  void update(NodeId node_id, NodeAttr&& attrs);

  /**
   * @brief Update the attributes of an edge
   * @param source One endpoint of edge to add
   * @param target Other endpoint of edge to add
   * @param attrs New attributes to use for edge
   *
   * Allocates the edge if it doesn't exist already
   */
  void update(NodeId source, NodeId target, EdgeAttr&& attrs);

  void archive(NodeId node_id);

  void remove(NodeId node_id);

  void remove(NodeId source, NodeId target);

  typename Nodes::iterator erase(const typename Nodes::iterator& iter);

  void contract(NodeId from, NodeId to);

  //! Clear archived nodes and deleted tracking information
  void prune();

  bool has(NodeId node) const;

  bool has(NodeId source, NodeId target) const;

  std::set<spark_dsg::NodeId> neighbors(NodeId node) const;

  AttrT* at(NodeId node) const;

  spark_dsg::EdgeAttributes* at(NodeId source, NodeId target) const;

  const std::map<NodeId, Node>& nodes() const { return nodes_; }

  const std::map<spark_dsg::EdgeKey, EdgeAttr>& edges() const { return edges_; }

  const std::set<NodeId>& deleted_nodes() const { return deleted_nodes_; }

  const std::set<spark_dsg::EdgeKey>& deleted_edges() const { return deleted_edges_; }

  size_t num_nodes() const { return nodes_.size(); }

  size_t num_edges() const { return edges_.size(); }

  typename Nodes::iterator begin() { return nodes_.begin(); }
  typename Nodes::const_iterator begin() const { return nodes_.begin(); }

  typename Nodes::iterator end() { return nodes_.end(); }
  typename Nodes::const_iterator end() const { return nodes_.end(); }

 private:
  Node* find(NodeId node);

  Node& allocate(NodeId node);

  std::map<NodeId, Node> nodes_;
  std::map<spark_dsg::EdgeKey, EdgeAttr> edges_;

  std::set<NodeId> deleted_nodes_;
  std::set<spark_dsg::EdgeKey> deleted_edges_;
};

}  // namespace hydra

#include "hydra/common/impl/partial_graph.h"  // IWYU pragma: keep
