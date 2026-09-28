#include "hydra_multi/operators/deformation_graph_operator.h"

#include <glog/logging.h>
#include <gtsam/inference/Symbol.h>

#include <Eigen/Dense>

namespace hydra_multi {

using PoseGraph = pose_graph_tools::PoseGraph;
using PoseGraphEdge = pose_graph_tools::PoseGraphEdge;
using PoseGraphNode = pose_graph_tools::PoseGraphNode;

namespace {

inline void updateAdjacency(std::map<uint64_t, std::set<uint64_t>>& adj_map,
                            uint64_t source,
                            uint64_t target) {
  auto iter = adj_map.find(source);
  if (iter == adj_map.end()) {
    iter = adj_map.emplace(source, std::set<uint64_t>()).first;
  }

  iter->second.insert(target);
}

}  // namespace

void DeformationGraphOperator::processEdge(const PoseGraphEdge& edge) {
  const std::pair<size_t, size_t> key{edge.key_to, edge.key_from};
  if (connections_.count(key) || edge.type != pose_graph_tools::PoseGraphEdge::MESH) {
    return;
  }

  connections_[key] = edge.pose;
  updateAdjacency(adjacency_, key.first, key.second);
  updateAdjacency(adjacency_, key.second, key.first);
}

bool DeformationGraphOperator::nodeDisconnected(uint64_t id) const {
  auto iter = adjacency_.find(id);
  return iter == adjacency_.end() ? true : iter->second.empty();
}

bool DeformationGraphOperator::incrementalAppend(const PoseGraph& incremental_source) {
  // Appending new edges and nodes from source to data_
  for (const auto& edge : incremental_source.edges) {
    processEdge(edge);
    data_->edges.push_back(edge);
  }

  for (const auto& node : incremental_source.nodes) {
    if (!data_->nodes.size()) {
      data_->nodes.push_back(node);
      tracked_nodes_[node.key] = node;
      continue;
    }

    if (nodeDisconnected(node.key)) {
      LOG(ERROR) << "Disconnected node " << node.key << "in append.";
      return false;
    }

    // TODO(nathan) think through how this works
    data_->nodes.push_back(node);
    tracked_nodes_[node.key] = node;
  }

  return true;
}

bool DeformationGraphOperator::update(const PoseGraph& source) {
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

bool DeformationGraphOperator::rebase(const PoseGraph& source) {
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
    if (nodeDisconnected(node.key)) {
      LOG(ERROR) << "Disconnected node in rebase.";
      return false;
    }

    data_->nodes.push_back(node);
    tracked_nodes_[node.key] = node;
  }

  return true;
}

bool DeformationGraphOperator::merge(const PoseGraph& source) {
  // To merge find the new parts of source and append
  // Implicitly assume that we can directly add the edges
  data_->edges = source.edges;
  connections_ = Connections();
  adjacency_.clear();
  for (const auto& edge : data_->edges) {
    processEdge(edge);
  }

  for (const auto& node : source.nodes) {
    // Check if source is new
    if (tracked_nodes_.count(node.key)) {
      continue;
    }

    if (data_->nodes.empty()) {
      data_->nodes.push_back(node);
      tracked_nodes_[node.key] = node;
      continue;
    }

    if (nodeDisconnected(node.key)) {
      LOG(ERROR) << "Disconnected node in merge: " << node.robot_id << ":" << node.key;
      continue;
    }

    VLOG(2) << "New node in merge: " << node.robot_id << ":" << node.key;
    auto new_node = PoseGraphNode(node);
    const auto& neighbors = adjacency_.at(node.key);
    for (const auto& n : neighbors) {
      auto iter = tracked_nodes_.find(n);
      if (iter == tracked_nodes_.end()) {
        continue;
      }

      // TODO(nathan) think about averaging all estimates?
      const std::pair<gtsam::Key, gtsam::Key> edge_key{node.key, n};
      const auto& tracked_T_new = connections_.at(edge_key);
      new_node.pose = tracked_nodes_.at(n).pose * tracked_T_new;
      break;
    }

    data_->nodes.push_back(new_node);
    tracked_nodes_[new_node.key] = new_node;
  }

  return true;
}

}  // namespace hydra_multi
