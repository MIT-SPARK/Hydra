#pragma once
#include "hydra_dsg_builder/dsg_types.h"

namespace hydra {

struct MinimalEdge {
  NodeId source;
  NodeId target;
  double distance;

  MinimalEdge() = default;

  MinimalEdge(NodeId source, NodeId target, double distance)
      : source(source), target(target), distance(distance) {}

  inline bool operator>(const MinimalEdge& other) const {
    return distance > other.distance;
  }
};

struct DisjointSet {
  explicit DisjointSet(const SceneGraphLayer& layer);

  NodeId findSet(NodeId node) const;

  bool doUnion(NodeId lhs, NodeId rhs);

  std::unordered_map<NodeId, NodeId> parents;
  std::unordered_map<NodeId, size_t> sizes;
};

struct MinimumSpanningTreeInfo {
  std::vector<MinimalEdge> edges;
  std::unordered_set<NodeId> leaves;
  std::unordered_map<NodeId, size_t> counts;
};

MinimumSpanningTreeInfo getMinimumSpanningEdges(const SceneGraphLayer& layer);

}  // namespace hydra
