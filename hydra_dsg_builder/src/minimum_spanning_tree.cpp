#include "hydra_dsg_builder/minimum_spanning_tree.h"
#include <glog/logging.h>

namespace hydra {

// implementation mainly from: https://en.wikipedia.org/wiki/Disjoint-set_data_structure
DisjointSet::DisjointSet(const SceneGraphLayer& layer) {
  for (const auto& id_node_pair : layer.nodes()) {
    parents[id_node_pair.first] = id_node_pair.first;
    sizes[id_node_pair.first] = 1;
  }
}

NodeId DisjointSet::findSet(NodeId node) const {
  NodeId parent = node;

  NodeId curr_node;
  do {
    curr_node = parent;
    parent = parents.at(curr_node);
  } while (parent != curr_node);

  return parent;
}

bool DisjointSet::doUnion(NodeId lhs, NodeId rhs) {
  NodeId lhs_set = findSet(lhs);
  NodeId rhs_set = findSet(rhs);

  // technically don't need this
  if (lhs_set == rhs_set) {
    return false;
  }

  if (sizes.at(lhs_set) < sizes.at(rhs_set)) {
    std::swap(lhs_set, rhs_set);
  }

  parents[rhs_set] = lhs_set;
  sizes[lhs_set] = sizes[lhs_set] + sizes[rhs_set];
  return true;
}

// implementation mainly from: https://en.wikipedia.org/wiki/Kruskal%27s_algorithm
MinimumSpanningTreeInfo getMinimumSpanningEdges(const SceneGraphLayer& layer) {
  std::vector<MinimalEdge> sorted_edges;
  sorted_edges.reserve(layer.edges().size());
  for (const auto& id_edge_pair : layer.edges()) {
    const auto& edge = id_edge_pair.second;
    sorted_edges.emplace_back(
        edge.source,
        edge.target,
        (layer.getPosition(edge.source) - layer.getPosition(edge.target)).norm());
  }
  std::make_heap(sorted_edges.begin(), sorted_edges.end(), std::greater<>{});

  MinimumSpanningTreeInfo info;
  info.edges.reserve(sorted_edges.size());
  for (const auto& id_node_pair : layer.nodes()) {
    info.counts[id_node_pair.first] = 0;
  }

  DisjointSet subtrees(layer);
  while (!sorted_edges.empty()) {
    std::pop_heap(sorted_edges.begin(), sorted_edges.end(), std::greater<>{});
    auto min_edge = sorted_edges.back();
    sorted_edges.pop_back();

    NodeId lhs_set = subtrees.findSet(min_edge.source);
    NodeId rhs_set = subtrees.findSet(min_edge.target);
    if (lhs_set != rhs_set) {
      subtrees.doUnion(lhs_set, rhs_set);
      info.edges.push_back(min_edge);
      info.counts[min_edge.source]++;
      info.counts[min_edge.target]++;
    }
  }

  for (const auto& id_count_pair : info.counts) {
    if (id_count_pair.second == 1) {
      info.leaves.insert(id_count_pair.first);
    }
  }

  return info;
}

}  // namespace hydra
