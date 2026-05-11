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
#include "hydra/places/gvd_places/graph_extractor_utilities.h"

#include <config_utilities/config.h>
#include <spatial_hash/neighbor_utils.h>

namespace hydra::places {

using spark_dsg::EdgeAttributes;
using spark_dsg::EdgeKey;
using spark_dsg::NodeId;

using Components = std::vector<std::vector<NodeId>>;

namespace {

std::optional<NodeId> getBestNode(const PlaceGraph& graph,
                                  NodeId source,
                                  const std::vector<NodeId>& candidates,
                                  double max_distance_m) {
  const auto source_attrs = graph.at(source);
  if (!source_attrs) {
    return std::nullopt;  // no best node for archived node
  }

  double best_dist = 0.0;
  std::optional<NodeId> best_node;
  const auto pos = source_attrs->position;
  for (const auto& target : candidates) {
    const auto target_attrs = graph.at(target);
    if (!target_attrs) {
      continue;  // archived node
    }

    const auto dist = (pos - target_attrs->position).norm();
    if (!best_node || dist < best_dist) {
      best_node = target;
      best_dist = dist;
    }
  }

  if (best_node && best_dist > max_distance_m) {
    best_node = std::nullopt;
  }

  return best_node;
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

EdgeAttributes::Ptr getOverlapEdgeInfo(const PlaceGraph& graph,
                                       NodeId node,
                                       NodeId neighbor,
                                       double min_clearance) {
  if (node == neighbor) {
    return nullptr;
  }

  const auto r1 = graph.at(node)->distance;
  const auto r2 = graph.at(neighbor)->distance;
  const auto d = (graph.at(node)->position - graph.at(neighbor)->position).norm();
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

EdgeAttributes::Ptr getFreespaceEdgeInfo(const PlaceGraph& graph,
                                         const GvdLayer& gvd,
                                         const NodeIndexMap& node_index_map,
                                         NodeId node,
                                         NodeId other,
                                         double min_clearance_m,
                                         bool optimistic) {
  const auto source = node_index_map.at(node);
  const auto target = node_index_map.at(other);
  const auto path = makeBresenhamLine(source, target);
  if (path.empty()) {
    return nullptr;
  }

  const auto source_dist = graph.at(node)->distance;
  const auto target_dist = graph.at(other)->distance;
  auto min_weight = std::min(source_dist, target_dist);
  for (const auto& index : path) {
    const auto* voxel = gvd.getVoxelPtr(index);
    if (!voxel) {
      if (optimistic) {
        continue;  // assume that archived voxels remain free
      } else {
        return nullptr;
      }
    }

    if (!voxel->observed || voxel->distance <= min_clearance_m) {
      return nullptr;
    }

    if (voxel->distance < min_weight) {
      min_weight = voxel->distance;
    }
  }

  return std::make_unique<EdgeAttributes>(min_weight);
}

void findFreespaceEdges(const PlaceGraph& graph,
                        const GvdLayer& gvd,
                        const NodeIndexMap& indices,
                        double max_length_m,
                        double min_clearance_m,
                        EdgeInfoMap& proposed_edges) {
  auto components = graph.connected_components();
  if (components.size() <= 1) {
    return;  // nothing to do
  }

  auto first_component = components.front();
  for (size_t i = 1; i < components.size(); ++i) {
    const auto& component = components[i];

    bool inserted_edge = false;
    for (const auto& source : component) {
      const auto target = getBestNode(graph, source, first_component, max_length_m);
      if (!target || graph.has(source, *target)) {
        continue;  // graph should never has edges connecting components, but...
      }

      const EdgeKey key(source, *target);
      auto info = getFreespaceEdgeInfo(
          graph, gvd, indices, key.k1, key.k2, min_clearance_m, false);
      inserted_edge |= info != nullptr;
      if (info) {
        proposed_edges.emplace(key, std::move(info));
      }
    }

    if (inserted_edge) {
      // merge components if an edge was inserted
      first_component.insert(first_component.end(), component.begin(), component.end());
    }
  }
}

}  // namespace hydra::places
