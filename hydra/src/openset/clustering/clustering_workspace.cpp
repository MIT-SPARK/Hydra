#include "clio/clustering_workspace.h"

#include <glog/logging.h>
#include <hydra/utils/display_utilities.h>
#include <spark_dsg/node_attributes.h>

#include <numeric>

namespace clio {

using namespace spark_dsg;
using EmbeddingMap = std::map<NodeId, Eigen::VectorXf>;

bool keysIntersect(EdgeKey key1, EdgeKey key2) {
  return key1.k1 == key2.k1 || key1.k1 == key2.k2 || key1.k2 == key2.k1 ||
         key1.k2 == key2.k2;
}

EmbeddingMap getEmbeddingMap(const std::map<NodeId, SceneGraphNode::Ptr>& nodes) {
  EmbeddingMap features;
  for (const auto& id_node : nodes) {
    const auto& attrs = id_node.second->attributes<SemanticNodeAttributes>();
    // TODO(nathan) consider other pooling operations
    features[id_node.first] = attrs.semantic_feature.rowwise().mean();
  }

  return features;
}

EmbeddingMap getEmbeddingMap(const SceneGraphLayer& layer,
                             const std::vector<NodeId>& nodes) {
  EmbeddingMap features;
  for (const auto node : nodes) {
    const auto& attrs = layer.getNode(node).attributes<SemanticNodeAttributes>();
    // TODO(nathan) consider other pooling operations
    features[node] = attrs.semantic_feature.rowwise().mean();
  }

  return features;
}

ClusteringWorkspace::ClusteringWorkspace(const SceneGraphLayer& layer)
    : ClusteringWorkspace(layer, getEmbeddingMap(layer.nodes())) {}

ClusteringWorkspace::ClusteringWorkspace(const SceneGraphLayer& layer,
                                         const std::vector<NodeId>& nodes)
    : ClusteringWorkspace(layer, getEmbeddingMap(layer, nodes)) {}

ClusteringWorkspace::ClusteringWorkspace(const SceneGraphLayer& layer,
                                         const EmbeddingMap& node_embeddings) {
  size_t index = 0;
  for (auto&& [node_id, feature] : node_embeddings) {
    features[index] = feature;
    node_lookup[index] = node_id;
    order[node_id] = index;
    ++index;
  }

  for (auto&& [node_id, index] : order) {
    const auto& node = layer.getNode(node_id);
    for (const auto& sibling : node.siblings()) {
      auto iter = order.find(sibling);
      if (iter == order.end()) {
        continue;
      }

      edges.emplace(EdgeKey(index, iter->second), 0.0);
    }
  }

  assignments.resize(order.size());
  std::iota(assignments.begin(), assignments.end(), 0);
}

size_t ClusteringWorkspace::size() const { return order.size(); }

size_t ClusteringWorkspace::featureDim() const {
  if (features.empty()) {
    return 0;
  }

  return features.begin()->second.rows();
}

std::list<EdgeKey> ClusteringWorkspace::addMerge(EdgeKey key) {
  // TODO(nathan) there are more efficient ways to maintain this, but modfiying
  // disjoint set right now is not the best decision
  for (auto& parent : assignments) {
    if (parent == key.k2) {
      parent = key.k1;
    }
  }

  // clear all affected edge weights
  std::set<EdgeKey> seen;
  auto iter = edges.begin();
  while (iter != edges.end()) {
    if (keysIntersect(iter->first, key)) {
      seen.insert(iter->first);
      iter = edges.erase(iter);
    } else {
      ++iter;
    }
  }

  std::set<EdgeKey> new_keys;
  std::list<EdgeKey> to_update;
  for (const auto& prev : seen) {
    const auto new_k1 = prev.k1 == key.k2 ? key.k1 : prev.k1;
    const auto new_k2 = prev.k2 == key.k2 ? key.k1 : prev.k2;
    if (new_k1 == new_k2) {
      continue;
    }

    const EdgeKey new_key(new_k1, new_k2);
    if (new_keys.count(new_key)) {
      continue;  // we got a duplicate key from a merge
    }

    edges.emplace(new_key, 0.0);
    to_update.push_back(new_key);
  }

  return to_update;
}

std::vector<std::vector<NodeId>> ClusteringWorkspace::getClusters() const {
  const std::set<size_t> cluster_ids(assignments.begin(), assignments.end());

  size_t index = 0;
  std::map<size_t, size_t> cluster_lookup;
  for (const auto cluster_id : cluster_ids) {
    cluster_lookup[cluster_id] = index;
    ++index;
  }

  std::vector<std::vector<NodeId>> to_return(cluster_ids.size());
  for (size_t i = 0; i < assignments.size(); ++i) {
    auto& cluster = to_return.at(cluster_lookup[assignments[i]]);
    cluster.push_back(node_lookup.at(i));
  }

  return to_return;
}

}  // namespace clio
