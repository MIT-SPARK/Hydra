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
#include "hydra/regions/region_extractor.h"

#include <config_utilities/config.h>
#include <config_utilities/types/conversions.h>
#include <glog/logging.h>

#include <Eigen/Dense>
#include <algorithm>
#include <queue>

namespace hydra {

using Cluster = ClusterResults::Cluster;
using Clusters = ClusterResults::Clusters;
using IndexTimePairQueue = std::priority_queue<IndexTimePair,
                                               std::vector<IndexTimePair>,
                                               std::greater<IndexTimePair>>;

void fillIndexQueue(const SceneGraphLayer& places,
                    const Clusters& clusters,
                    IndexTimePairQueue& queue,
                    size_t min_room_size) {
  for (const auto& id_cluster_pair : clusters) {
    if (id_cluster_pair.second.size() < min_room_size) {
      continue;
    }

    uint64_t oldest_time_ns = std::numeric_limits<uint64_t>::max();
    for (const auto& place : id_cluster_pair.second) {
      const auto& attrs = places.getNode(place).attributes();
      if (attrs.last_update_time_ns < oldest_time_ns) {
        oldest_time_ns = attrs.last_update_time_ns;
      }
    }

    queue.push({id_cluster_pair.first, oldest_time_ns});
  }
}

void declare_config(RegionExtractor::Config& config) {
  using namespace config;
  name("RoomFinder::Config");
  field<CharConversion>(config.prefix, "prefix");
  field(config.min_region_size, "min_region_size");
}

RegionExtractor::RegionExtractor(const Config& config) : config(config) {}

RegionExtractor::~RegionExtractor() = default;

SceneGraphLayer::Ptr RegionExtractor::extract(const SceneGraphLayer& places) {
  VLOG(2) << "[Room Finder] Detecting rooms for " << places.numNodes() << " nodes";

  cluster_room_map_.clear();

  SceneGraphLayer::Ptr rooms(new SceneGraphLayer(DsgLayers::ROOMS));

  // organize rooms by their oldest place
  IndexTimePairQueue queue;
  fillIndexQueue(places, last_results_.clusters, queue, config.min_region_size);

  NodeSymbol room_id(config.prefix, 0);
  while (!queue.empty()) {
    const auto cluster_index = queue.top().index;
    queue.pop();

    cluster_room_map_[cluster_index] = room_id;
    const auto& cluster = last_results_.clusters.at(cluster_index);

    auto attrs = std::make_unique<RoomNodeAttributes>();
    // TODO(nathan) define unknown label somewhere
    attrs->semantic_label = 0;
    attrs->position = getRoomPosition(places, cluster);

    rooms->emplaceNode(room_id, std::move(attrs));
    ++room_id;
  }

  addEdgesToRoomLayer(places, last_results_.labels, cluster_room_map_, *rooms);
  return rooms;
}

void RegionExtractor::addRoomPlaceEdges(DynamicSceneGraph& graph) const {
  for (const auto& id_node_pair : graph.getLayer(DsgLayers::PLACES).nodes()) {
    const auto cluster = last_results_.labels.find(id_node_pair.first);
    if (cluster == last_results_.labels.end()) {
      continue;
    }

    const auto room = cluster_room_map_.find(cluster->second);
    if (room == cluster_room_map_.end()) {
      continue;
    }

    // add edge enforcing parent invariants
    graph.insertEdge(room->second, id_node_pair.first, nullptr, true);
  }
}

}  // namespace hydra
