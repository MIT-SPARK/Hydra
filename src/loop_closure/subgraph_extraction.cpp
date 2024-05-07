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
#include "hydra/loop_closure/subgraph_extraction.h"

#include <glog/logging.h>

namespace hydra {

SubgraphConfig::SubgraphConfig(double radius_m)
    : fixed_radius(true), max_radius_m(radius_m) {}

SubgraphConfig::SubgraphConfig() = default;

void getObjectsWithinRadius(const DynamicSceneGraph& graph,
                            const Eigen::Vector3d& origin,
                            NodeId parent,
                            double radius_m,
                            std::set<NodeId>& found) {
  std::deque<NodeId> frontier{parent};
  std::unordered_set<NodeId> visited{parent};
  graph_utilities::breadthFirstSearch(
      graph.getLayer(DsgLayers::PLACES),
      frontier,
      visited,
      [&](const auto& node) {
        for (const auto& child : node.children()) {
          if (graph.isDynamic(child)) {
            continue;
          }

          if ((origin - graph.getPosition(child)).norm() < radius_m) {
            return true;
          }
        }

        return (origin - node.attributes().position).norm() < radius_m;
      },
      [](const auto&) { return true; },
      [&](const SceneGraphLayer&, NodeId node) {
        for (const auto& child : graph.getNode(node).children()) {
          if (graph.isDynamic(child)) {
            continue;
          }

          if ((origin - graph.getPosition(child)).norm() >= radius_m) {
            continue;
          }

          found.insert(child);
        }
      });
}

void getPlacesWithinRadius(const DynamicSceneGraph& graph,
                           const Eigen::Vector3d& origin,
                           NodeId parent,
                           double radius_m,
                           std::set<NodeId>& found) {
  std::deque<NodeId> frontier{parent};
  std::unordered_set<NodeId> visited{parent};
  graph_utilities::breadthFirstSearch(
      graph.getLayer(DsgLayers::PLACES),
      frontier,
      visited,
      [&](const auto& node) {
        return (origin - node.attributes().position).norm() < radius_m;
      },
      [](const auto&) { return true; },
      [&](const SceneGraphLayer&, NodeId node) { found.insert(node); });
}

std::set<NodeId> getFilteredNodeSet(const SubgraphConfig& config,
                                    const DynamicSceneGraph& graph,
                                    const Eigen::Vector3d& origin,
                                    const std::set<NodeId>& found) {
  std::vector<std::pair<double, NodeId>> candidates;
  for (const auto node : found) {
    const double distance_m = (graph.getPosition(node) - origin).norm();
    candidates.push_back({distance_m, node});
  }
  std::sort(candidates.begin(), candidates.end());

  std::set<NodeId> valid;
  std::optional<double> last_distance;
  for (const auto& candidate : candidates) {
    if (candidate.first < config.min_radius_m) {
      // we take as many nodes as possible within the min radius
      valid.insert(candidate.second);
      last_distance = candidate.first;
      continue;
    }

    if (candidate.first >= config.max_radius_m) {
      break;  // we've hit candidates that are outside the radius we want
    }

    if (last_distance && candidate.first == last_distance) {
      // make sure we don't exit early if we have equidistant nodes (unlikely)
      valid.insert(candidate.second);
      continue;
    }

    last_distance = candidate.first;
    if (valid.size() >= config.min_nodes) {
      break;  // if we hit this point, we have all the nodes within the min radius, so
              // okay to exit
    }

    valid.insert(candidate.second);
  }

  return valid;
}

std::set<NodeId> getSubgraphNodes(const SubgraphConfig& config,
                                  const DynamicSceneGraph& graph,
                                  NodeId root_node,
                                  bool is_places) {
  Eigen::Vector3d origin;
  try {
    origin = graph.getPosition(root_node);
  } catch (const std::out_of_range& e) {
    LOG(ERROR) << "Invalid root node " << NodeSymbol(root_node).getLabel() << ": "
               << e.what();
    return {};
  }

  std::set<NodeId> found;
  if (is_places) {
    getPlacesWithinRadius(graph, origin, root_node, config.max_radius_m, found);
  } else {
    getObjectsWithinRadius(graph, origin, root_node, config.max_radius_m, found);
  }

  if (config.fixed_radius) {
    return found;
  }

  return getFilteredNodeSet(config, graph, origin, found);
}

}  // namespace hydra
