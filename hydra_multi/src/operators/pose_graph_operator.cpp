#include "hydra_multi/operators/pose_graph_operator.h"

#include <glog/logging.h>

#include <Eigen/Dense>
#include <unordered_set>

namespace hydra_multi {

using PoseGraph = pose_graph_tools::PoseGraph;
using PoseGraphEdge = pose_graph_tools::PoseGraphEdge;
using PoseGraphNode = pose_graph_tools::PoseGraphNode;

void updateConnections(const PoseGraphEdge& edge,
                       PoseGraphOperator::Connections& connections) {
  static const std::unordered_set<int> ignored_edge_types = {
      pose_graph_tools::PoseGraphEdge::LOOPCLOSE,
      pose_graph_tools::PoseGraphEdge::POSE_MESH,
      pose_graph_tools::PoseGraphEdge::MESH_POSE};

  if (connections.count(edge.key_to) || edge.key_to <= edge.key_from ||
      ignored_edge_types.count(edge.type)) {
    return;
  }

  connections[edge.key_to] = {edge.key_from, edge.pose};
}

bool PoseGraphOperator::incrementalAppend(const PoseGraph& incremental_source) {
  // Appending new edges and nodes from source to data_
  for (const auto& edge : incremental_source.edges) {
    updateConnections(edge, connections_);
    data_->edges.push_back(edge);
  }

  for (const auto& node : incremental_source.nodes) {
    if (!data_->nodes.size()) {
      data_->nodes.push_back(node);
      tracked_nodes_[node.key] = node;
      continue;
    }
    if (!connections_.count(node.key)) {
      LOG(ERROR) << "Disconnected node " << node.key << "in append.";
      return false;
    }
    auto tracked = connections_[node.key].first;
    if (!tracked_nodes_.count(tracked)) {
      return false;
    }
    auto tracked_T_new = connections_[node.key].second;
    auto new_node = PoseGraphNode(node);
    new_node.pose = tracked_nodes_[tracked].pose * tracked_T_new;

    data_->nodes.push_back(new_node);
    tracked_nodes_[new_node.key] = new_node;
  }

  return true;
}

bool PoseGraphOperator::update(const PoseGraph& source) {
  // Updating the node attributes in data_ according to source
  // Returns an error if data_ node does not exist in source

  HashedNodes source_nodes;
  for (const auto& node : source.nodes) {
    source_nodes[node.key] = node;
  }

  for (auto& node : data_->nodes) {
    if (!source_nodes.count(node.key)) {
      LOG(ERROR) << "Missing source node in update.";
      // NOTE(Yun): but some nodes potentially already updated
      return false;
    }
    // Currently only updates the pose
    node.pose = source_nodes[node.key].pose;
  }
  return true;
}

bool PoseGraphOperator::rebase(const PoseGraph& source) {
  // Rebase by first finding overlapping parts, updating to source, and then appending
  // the non-overlapping data_ nodes Note that we don't add nodes or edges in source
  // not in data_ to data_
  HashedNodes source_nodes;
  for (const auto& node : source.nodes) {
    source_nodes[node.key] = node;
  }

  // Update the data_ nodes that is in source and pick out the ones not in source
  tracked_nodes_ = HashedNodes();
  PoseGraph data_to_append;
  for (auto node_it = data_->nodes.begin(); node_it != data_->nodes.end();) {
    if (!source_nodes.count(node_it->key)) {
      data_to_append.nodes.push_back(*node_it);
      node_it = data_->nodes.erase(node_it);
      continue;
    }

    // Update pose
    node_it->pose = source_nodes[node_it->key].pose;
    tracked_nodes_[node_it->key] = *node_it;
    node_it++;
  }

  // Implicitly assume that edges attributes don't need to be updated
  for (const auto& node : data_to_append.nodes) {
    if (!connections_.count(node.key)) {
      LOG(ERROR) << "Disconnected node in rebase.";
      return false;
    }
    auto tracked = connections_[node.key].first;
    auto tracked_T_new = connections_[node.key].second;
    auto new_node = PoseGraphNode(node);
    new_node.pose = tracked_nodes_[tracked].pose * tracked_T_new;

    data_->nodes.push_back(new_node);
    tracked_nodes_[new_node.key] = new_node;
  }
  return true;
}

bool PoseGraphOperator::merge(const PoseGraph& source) {
  // To merge find the new parts of source and append
  // Implicitly assume that we can directly add the edges
  data_->edges = source.edges;
  connections_ = Connections();
  for (const auto& edge : data_->edges) {
    updateConnections(edge, connections_);
  }

  for (const auto& node : source.nodes) {
    // Check if source is new
    if (tracked_nodes_.count(node.key)) {
      continue;
    }

    if (!data_->nodes.size()) {
      data_->nodes.push_back(node);
      tracked_nodes_[node.key] = node;
      continue;
    }

    if (!connections_.count(node.key)) {
      LOG(ERROR) << "Disconnected node in merge.";
      // This shows up a lot fo mesh graphs. TODO(Yun) fix this or profile.
      data_->nodes.push_back(node);
      tracked_nodes_[node.key] = node;
      continue;
    }
    const auto& tracked = connections_[node.key].first;
    const auto& tracked_T_new = connections_[node.key].second;
    auto new_node = PoseGraphNode(node);
    new_node.pose = tracked_nodes_.at(tracked).pose * tracked_T_new;

    data_->nodes.push_back(new_node);
    tracked_nodes_[new_node.key] = new_node;
  }
  return true;
}

}  // namespace hydra_multi
