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
#include "hydra/rooms/graph_clustering.h"

#include <queue>

namespace hydra {

void ClusterResults::clear() {
  valid = false;
  total_iters = 0;
  labels.clear();
  clusters.clear();
}

void ClusterResults::fillFromInitialClusters(const InitialClusters& initial_clusters) {
  valid = true;
  total_iters = 0;

  size_t i = 0;
  for (const auto& cluster : initial_clusters) {
    clusters[i] = std::unordered_set<NodeId>(cluster.begin(), cluster.end());
    for (const auto& node_id : cluster) {
      labels[node_id] = i;
    }

    ++i;
  }
}

ClusterResults clusterGraphByModularity(const SceneGraphLayer& layer,
                                        const InitialClusters& initial_clusters,
                                        size_t max_iters,
                                        double gamma) {
  return clusterGraphByModularity(
      layer,
      initial_clusters,
      [](const SceneGraphLayer& G, NodeId n1, const NodeId n2) {
        return G.getEdge(n1, n2).info->weight;
      },
      max_iters,
      gamma);
}

ClusterResults clusterGraphByModularity(const SceneGraphLayer& layer,
                                        const InitialClusters& initial_clusters,
                                        const EdgeWeightFunc& edge_weight_func,
                                        size_t max_iters,
                                        double gamma) {
  std::map<NodeId, double> degrees;
  std::map<NodeId, std::map<NodeId, double>> neighbors;
  for (const auto& id_node_pair : layer.nodes()) {
    double degree = 0.0;
    neighbors[id_node_pair.first] = std::map<NodeId, double>();
    for (const auto& sibling : id_node_pair.second->siblings()) {
      double edge_weight = edge_weight_func(layer, id_node_pair.first, sibling);
      // we should probably assert that this isn't happening, but it should be pretty
      // feasbile to not return negative weights
      edge_weight = edge_weight < 0.0 ? 0.0 : edge_weight;
      degree += edge_weight;
      neighbors[id_node_pair.first][sibling] = edge_weight;
    }
    degrees[id_node_pair.first] = degree;
  }

  const double m = layer.numEdges();

  std::map<size_t, double> community_degrees;
  std::map<NodeId, size_t> labels;
  for (size_t i = 0; i < initial_clusters.size(); ++i) {
    const auto& component = initial_clusters[i];
    double total_degree = 0.0;
    for (const auto& node_id : component) {
      labels[node_id] = i;
      total_degree += degrees.at(node_id);
    }
    community_degrees[i] = total_degree;
  }

  std::set<NodeId> unlabeled_nodes;
  for (const auto& id_node_pair : layer.nodes()) {
    if (labels.count(id_node_pair.first)) {
      continue;
    }

    unlabeled_nodes.insert(id_node_pair.first);
  }

  size_t iter;
  for (iter = 0; iter < max_iters; ++iter) {
    size_t num_changes = 0;
    for (const auto& node : unlabeled_nodes) {
      std::map<size_t, double> community_weights;
      for (const auto id_weight_pair : neighbors.at(node)) {
        if (!labels.count(id_weight_pair.first)) {
          continue;
        }

        const size_t community = labels.at(id_weight_pair.first);
        if (!community_weights.count(community)) {
          community_weights[community] = 0.0;
        }

        community_weights[community] += id_weight_pair.second;
      }

      const double node_degree = degrees.at(node);
      if (labels.count(node)) {
        community_degrees[labels.at(node)] -= node_degree;
      }

      double best_gain = 0.0;
      size_t best_community = initial_clusters.size();
      for (const auto& id_weight_pair : community_weights) {
        const double gain =
            2 * id_weight_pair.second -
            gamma * (community_degrees.at(id_weight_pair.first) * node_degree) / m;
        if (gain > best_gain) {
          best_gain = gain;
          best_community = id_weight_pair.first;
        }
      }

      if (best_community == initial_clusters.size()) {
        continue;  // we couldn't pick a community
      }

      community_degrees[best_community] += node_degree;

      if (labels.count(node) && labels.at(node) == best_community) {
        continue;
      }

      ++num_changes;
      labels[node] = best_community;
    }

    if (!num_changes) {
      break;
    }
  }

  std::map<size_t, std::unordered_set<NodeId>> clusters;
  for (const auto& node_cluster_pair : labels) {
    if (!clusters.count(node_cluster_pair.second)) {
      clusters[node_cluster_pair.second] = std::unordered_set<NodeId>();
    }

    clusters[node_cluster_pair.second].insert(node_cluster_pair.first);
  }

  return {clusters, labels, iter, true};
}

struct EdgeInfo {
  NodeId id;
  size_t label;
  double distance;

  bool operator<(const EdgeInfo& other) const { return distance < other.distance; }
};

ClusterResults clusterGraphByNeighbors(const SceneGraphLayer& layer,
                                       const InitialClusters& initial_clusters) {
  std::map<NodeId, size_t> labels;
  for (size_t i = 0; i < initial_clusters.size(); ++i) {
    const auto& component = initial_clusters[i];
    for (const auto& node_id : component) {
      labels[node_id] = i;
    }
  }

  // populate frontier from all room boundaries
  std::priority_queue<EdgeInfo> frontier;
  for (auto&& [id, node] : layer.nodes()) {
    if (labels.count(id)) {
      continue;
    }

    for (const auto sibling : node->siblings()) {
      auto iter = labels.find(sibling);
      if (iter == labels.end()) {
        continue;
      }

      frontier.push({id, iter->second, layer.getEdge(id, sibling).info->weight});
    }
  }

  while (!frontier.empty()) {
    const auto candidate = frontier.top();
    frontier.pop();

    if (labels.count(candidate.id)) {
      continue;
    }

    labels[candidate.id] = candidate.label;
    const auto& node = layer.getNode(candidate.id);
    for (const auto sibling : node.siblings()) {
      if (labels.count(sibling)) {
        continue;
      }

      frontier.push({sibling,
                     candidate.label,
                     layer.getEdge(candidate.id, sibling).info->weight});
    }
  }

  std::map<size_t, std::unordered_set<NodeId>> clusters;
  for (const auto& node_cluster_pair : labels) {
    if (!clusters.count(node_cluster_pair.second)) {
      clusters[node_cluster_pair.second] = std::unordered_set<NodeId>();
    }

    clusters[node_cluster_pair.second].insert(node_cluster_pair.first);
  }

  return {clusters, labels, 0, true};
}

}  // namespace hydra
