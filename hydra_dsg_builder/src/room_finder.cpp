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
#include "hydra_dsg_builder/room_finder.h"
#include "hydra_dsg_builder/graph_filtration.h"
#include "hydra_dsg_builder/room_colormap.h"
#include "hydra_dsg_builder/room_helpers.h"

#include <glog/logging.h>
#include <Eigen/Dense>
#include <algorithm>
#include <queue>

namespace hydra {

struct IndexTimePair {
  size_t index;
  uint64_t time_ns;

  bool operator>(const IndexTimePair& other) const { return time_ns > other.time_ns; }
};

using Cluster = ClusterResults::Cluster;
using Clusters = ClusterResults::Clusters;
using IndexTimePairQueue = std::priority_queue<IndexTimePair,
                                               std::vector<IndexTimePair>,
                                               std::greater<IndexTimePair>>;

void fillIndexQueue(const SceneGraphLayer& places,
                    const Clusters& clusters,
                    IndexTimePairQueue& queue,
                    size_t min_room_size) {
  for (const auto id_cluster_pair : clusters) {
    if (id_cluster_pair.second.size() < min_room_size) {
      continue;
    }

    uint64_t oldest_time_ns = std::numeric_limits<uint64_t>::max();
    for (const auto& place : id_cluster_pair.second) {
      const auto& attrs = places.getNode(place)->get().attributes();
      if (attrs.last_update_time_ns < oldest_time_ns) {
        oldest_time_ns = attrs.last_update_time_ns;
      }
    }

    queue.push({id_cluster_pair.first, oldest_time_ns});
  }
}

RoomFinder::RoomFinder(const RoomFinderConfig& config) : config_(config) {}

InitialClusters RoomFinder::getBestComponents(const SceneGraphLayer& places) const {
  const auto filtration = getGraphFiltration(
      places, config_.min_component_size, config_.dilation_diff_threshold_m);

  VLOG(10) << "[RoomFinder] Filtration: " << filtration;

  const auto window =
      getTrimmedFiltration(filtration, config_.min_dilation_m, config_.max_dilation_m);
  const auto candidate = getLongestSequence(filtration, window.first, window.second);
  if (!candidate) {
    return {};
  }

  const auto info = *candidate;

  VLOG(3) << "[RoomFinder] Best threshold: " << info.distance << " ("
          << info.num_components << " components)";

  const auto components = graph_utilities::getConnectedComponents(
      places,
      [&](const SceneGraphNode& node) {
        return node.attributes<PlaceNodeAttributes>().distance > info.distance;
      },
      [&](const SceneGraphEdge& edge) { return edge.info->weight > info.distance; });

  InitialClusters filtered;
  for (const auto& component : components) {
    if (component.size() < config_.min_component_size) {
      continue;
    }

    filtered.push_back(component);
  }

  return filtered;
}

SceneGraphLayer::Ptr RoomFinder::findRooms(const SceneGraphLayer& places) {
  VLOG(3) << "[Room Finder] Detecting rooms for " << places.numNodes() << " nodes";

  const auto components = getBestComponents(places);
  if (components.empty()) {
    VLOG(1) << "[Room Finder] No rooms found";
    return nullptr;
  }

  last_results_.clear();
  switch (config_.clustering_mode) {
    case RoomClusterMode::MODULARITY:
      last_results_ = clusterGraphByModularity(
          places, components, config_.max_modularity_iters, config_.modularity_gamma);
      break;
    case RoomClusterMode::NONE:
    default:
      last_results_.fillFromInitialClusters(components);
      break;
  }

  if (!last_results_.valid) {
    LOG(WARNING) << "[Room Finder] clustering failed: using components";
    last_results_.fillFromInitialClusters(components);
  }

  cluster_room_map_.clear();
  return makeRoomLayer(places);
}

SceneGraphLayer::Ptr RoomFinder::makeRoomLayer(const SceneGraphLayer& places) {
  IsolatedSceneGraphLayer::Ptr rooms(new IsolatedSceneGraphLayer(DsgLayers::ROOMS));

  // organize rooms by their oldest place
  IndexTimePairQueue queue;
  fillIndexQueue(places, last_results_.clusters, queue, config_.min_room_size);

  NodeSymbol room_id(config_.room_prefix, 0);
  while (!queue.empty()) {
    const auto cluster_index = queue.top().index;
    queue.pop();

    cluster_room_map_[cluster_index] = room_id;
    const auto& cluster = last_results_.clusters.at(cluster_index);

    auto attrs = std::make_unique<RoomNodeAttributes>();
    // TODO(nathan) define unknown label somewhere
    attrs->semantic_label = 0;
    attrs->name = room_id.getLabel();
    attrs->color = getRoomColor(room_id.categoryId());
    attrs->position = getRoomPosition(places, cluster);

    rooms->emplaceNode(room_id, std::move(attrs));
    ++room_id;
  }

  addEdgesToRoomLayer(places, last_results_.labels, cluster_room_map_, *rooms);
  return rooms;
}

void RoomFinder::addRoomPlaceEdges(DynamicSceneGraph& graph) const {
  for (const auto& id_node_pair : graph.getLayer(DsgLayers::PLACES).nodes()) {
    const auto cluster = last_results_.labels.find(id_node_pair.first);
    if (cluster == last_results_.labels.end()) {
      continue;
    }

    const auto room = cluster_room_map_.find(cluster->second);
    if (room == cluster_room_map_.end()) {
      continue;
    }

    graph.insertEdge(room->second, id_node_pair.first);
  }
}

}  // namespace hydra
