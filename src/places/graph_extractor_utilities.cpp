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
#include "hydra/places/graph_extractor_utilities.h"

#include <spatial_hash/neighbor_utils.h>

#include "hydra/places/nearest_voxel_utilities.h"
#include "hydra/utils/nearest_neighbor_utilities.h"

namespace hydra::places {

using Components = std::vector<std::vector<NodeId>>;

namespace {

inline double getNodeGvdDistance(const SceneGraphLayer& graph, NodeId node) {
  return graph.getNode(node).attributes<PlaceNodeAttributes>().distance;
}

void sortComponents(const SceneGraphLayer& graph, Components& to_sort) {
  for (auto& c : to_sort) {
    std::sort(c.begin(), c.end(), [&](const NodeId& lhs, const NodeId& rhs) {
      return getNodeGvdDistance(graph, lhs) > getNodeGvdDistance(graph, rhs);
    });
  }

  std::sort(to_sort.begin(), to_sort.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.size() > rhs.size();
  });
}

}  // namespace

// implementation loosely based on: https://gist.github.com/yamamushi/5823518
GlobalIndices makeBresenhamLine(const GlobalIndex& start, const GlobalIndex& end) {
  GlobalIndex diff = end - start;
  GlobalIndex inc(diff(0) < 0 ? -1 : 1, diff(1) < 0 ? -1 : 1, diff(2) < 0 ? -1 : 1);

  diff = diff.array().abs();
  const GlobalIndex diff_twice = 2 * diff;

  int max_idx;
  int min_idx_1;
  int min_idx_2;
  if (diff(0) >= diff(1) && diff(0) >= diff(2)) {
    max_idx = 0;
    min_idx_1 = 1;
    min_idx_2 = 2;
  } else if (diff(1) >= diff(0) && diff(1) >= diff(2)) {
    max_idx = 1;
    min_idx_1 = 0;
    min_idx_2 = 2;
  } else {
    max_idx = 2;
    min_idx_1 = 0;
    min_idx_2 = 1;
  }

  if (diff(max_idx) <= 1) {
    return GlobalIndices();
  }

  GlobalIndices line_points(diff(max_idx) - 1);
  GlobalIndex point = start;

  int64_t err_1 = diff_twice(min_idx_1) - diff(max_idx);
  int64_t err_2 = diff_twice(min_idx_2) - diff(max_idx);
  for (int64_t i = 0; i < diff(max_idx); ++i) {
    if (i > 0) {
      line_points[i - 1] = point;
    }

    if (err_1 > 0) {
      point(min_idx_1) += inc(min_idx_1);
      err_1 -= diff_twice(max_idx);
    }
    if (err_2 > 0) {
      point(min_idx_2) += inc(min_idx_2);
      err_2 -= diff_twice(max_idx);
    }
    err_1 += diff_twice(min_idx_1);
    err_2 += diff_twice(min_idx_2);
    point[max_idx] += inc(max_idx);
  }

  return line_points;
}

EdgeAttributes::Ptr getOverlapEdgeInfo(const SceneGraphLayer& graph,
                                       NodeId node,
                                       NodeId neighbor,
                                       double min_clearance) {
  if (node == neighbor) {
    return nullptr;
  }

  const double r1 = getNodeGvdDistance(graph, node);
  const double r2 = getNodeGvdDistance(graph, neighbor);
  const double d = (graph.getPosition(node) - graph.getPosition(neighbor)).norm();

  if (d >= r1 + r2) {
    return nullptr;
  }

  if (d <= r1 || d <= r2) {
    const double clearance = std::min(r1, r2);
    if (clearance < min_clearance) {
      // mostly for debugging
      return nullptr;
    }

    // intersection is inside one node's sphere
    return std::make_unique<EdgeAttributes>(clearance);
  }

  // see https://mathworld.wolfram.com/Sphere-SphereIntersection.html
  const double clearance =
      std::sqrt(4 * std::pow(d, 2) * std::pow(r1, 2) -
                std::pow(std::pow(d, 2) - std::pow(r2, 2) + std::pow(r1, 2), 2)) /
      (2 * d);
  if (clearance < min_clearance) {
    return nullptr;
  }

  return std::make_unique<EdgeAttributes>(clearance);
}

EdgeAttributes::Ptr getFreespaceEdgeInfo(const SceneGraphLayer& graph,
                                         const GvdLayer& gvd,
                                         const NodeIndexMap& node_index_map,
                                         NodeId node,
                                         NodeId other,
                                         double min_clearance_m) {
  const GlobalIndex source = node_index_map.at(node);
  const GlobalIndex target = node_index_map.at(other);
  const auto path = makeBresenhamLine(source, target);
  if (path.empty()) {
    return nullptr;
  }

  const double source_dist =
      graph.getNode(node).attributes<PlaceNodeAttributes>().distance;
  const double target_dist =
      graph.getNode(other).attributes<PlaceNodeAttributes>().distance;
  double min_weight = std::min(source_dist, target_dist);

  for (const auto& index : path) {
    const GvdVoxel* voxel = gvd.getVoxelPtr(index);
    if (!voxel || !voxel->observed || voxel->distance <= min_clearance_m) {
      return nullptr;
    }

    if (voxel->distance < min_weight) {
      min_weight = voxel->distance;
    }
  }

  return std::make_unique<EdgeAttributes>(min_weight);
}

void findOverlapEdges(const OverlapEdgeConfig& config,
                      const SceneGraphLayer& graph,
                      const std::unordered_set<NodeId> active_nodes,
                      EdgeInfoMap& proposed_edges) {
  NearestNodeFinder node_finder(graph, active_nodes);
  for (const auto node : active_nodes) {
    // TODO(nathan) consider deleting edges
    node_finder.find(graph.getPosition(node),
                     config.num_neighbors_to_check,
                     true,
                     [&](NodeId other, size_t, double) {
                       if (graph.hasEdge(node, other)) {
                         return;
                       }

                       auto info = getOverlapEdgeInfo(
                           graph, node, other, config.min_clearance_m);
                       if (info) {
                         proposed_edges.emplace(EdgeKey(node, other), std::move(info));
                       }
                     });
  }
}

void findFreespaceEdges(const FreespaceEdgeConfig& config,
                        const SceneGraphLayer& graph,
                        const GvdLayer& gvd,
                        const std::unordered_set<NodeId>& nodes,
                        const NodeIndexMap& indices,
                        EdgeInfoMap& proposed_edges) {
  auto components = graph_utilities::getConnectedComponents(graph, nodes, true);
  if (components.size() <= 1) {
    return;  // nothing to do
  }

  sortComponents(graph, components);
  std::vector<NodeId> first_component = components.front();

  for (size_t i = 1; i < components.size(); ++i) {
    const auto& component = components[i];
    NearestNodeFinder node_finder(graph, first_component);

    bool inserted_edge = false;
    for (size_t j = 0; j < config.num_nodes_to_check; ++j) {
      if (j >= component.size()) {
        break;
      }

      const NodeId node = component[j];
      node_finder.find(graph.getPosition(node),
                       config.num_neighbors_to_find,
                       false,
                       [&](NodeId other, size_t, double distance) {
                         if (distance > config.max_length_m) {
                           return;
                         }

                         if (graph.hasEdge(node, other)) {
                           return;
                         }

                         auto info = getFreespaceEdgeInfo(
                             graph, gvd, indices, node, other, config.min_clearance_m);
                         inserted_edge |= info != nullptr;

                         if (info) {
                           proposed_edges.emplace(EdgeKey(node, other),
                                                  std::move(info));
                         }
                       });
    }

    if (inserted_edge) {
      // merge components if an edge was inserted
      first_component.insert(first_component.end(), component.begin(), component.end());
    }
  }
}

}  // namespace hydra::places
