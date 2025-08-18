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
#include "hydra/rooms/room_finder.h"

#include <config_utilities/config.h>
#include <config_utilities/types/conversions.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/graph_utilities.h>

#include <Eigen/Dense>
#include <algorithm>
#include <queue>

#include "hydra/rooms/graph_filtration.h"
#include "hydra/rooms/room_utilities.h"

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

// TODO(nathan) embed in graph metadata
void logFiltration(const Filtration& /* filtration */,
                   const std::pair<size_t, size_t>& /* window */,
                   double /* distance */) {
  LOG(FATAL) << "Not implemented yet!";
}

void declare_config(RoomFinder::Config& conf) {
  using namespace config;
  name("RoomFinder::Config");
  field<CharConversion>(conf.room_prefix, "prefix");
  field(conf.min_dilation_m, "min_dilation_m", "m");
  field(conf.max_dilation_m, "max_dilation_m", "m");
  field(conf.min_window_size, "min_window_size");
  field(conf.clip_dilation_window_to_max, "clip_dilation_window_to_max");
  field(conf.min_component_size, "min_component_size");
  field(conf.min_room_size, "min_room_size");
  enum_field(conf.dilation_threshold_mode,
             "dilation_threshold_mode",
             {{DilationThresholdMode::REPEATED, "REPEATED"},
              {DilationThresholdMode::LONGEST_LIFETIME, "LONGEST_LIFETIME"},
              {DilationThresholdMode::PLATEAU, "PLATEAU"},
              {DilationThresholdMode::PLATEAU_THRESHOLD, "PLATEAU_THRESHOLD"}});
  field(conf.min_lifetime_length_m, "min_lifetime_length_m");
  field(conf.plateau_ratio, "plateau_ratio");
  field(conf.max_modularity_iters, "max_modularity_iters");
  field(conf.modularity_gamma, "modularity_gamma");
  enum_field(conf.clustering_mode,
             "clustering_mode",
             {{RoomClusterMode::MODULARITY, "MODULARITY"},
              {RoomClusterMode::MODULARITY, "MODULARITY_DISTANCE"},
              {RoomClusterMode::NEIGHBORS, "NEIGHBORS"},
              {RoomClusterMode::NONE, "NONE"}});
  field(conf.dilation_diff_threshold_m, "dilation_diff_threshold_m", "m");
  field<Path::Absolute>(conf.log_path, "log_path");
  field(conf.log_filtrations, "log_filtrations");
  field(conf.log_place_graphs, "log_place_graphs");
  // TODO(nathan) checks
}

RoomFinder::RoomFinder(const Config& config)
    : config(config::checkValid(config)), distance_adaptor_(new DistanceAdaptor()) {
  if (!config.log_path.empty()) {
    LOG(FATAL) << "Logging not implemented!";
    return;
  }
}

RoomFinder::~RoomFinder() {}

InitialClusters RoomFinder::getBestComponents(const SceneGraphLayer& places) const {
  BarcodeTracker tracker(config.min_component_size);
  const auto filtration = getGraphFiltration(
      places,
      tracker,
      config.dilation_diff_threshold_m,
      [this](const DisjointSet& components) -> size_t {
        size_t num_components = 0;
        for (const auto id_size_pair : components.sizes) {
          if (id_size_pair.second >= config.min_component_size) {
            ++num_components;
          }
        }
        return num_components;
      },
      false,
      *distance_adaptor_);

  VLOG(10) << "[RoomFinder] Filtration: " << filtration;

  auto window = getTrimmedFiltration(filtration,
                                     config.min_dilation_m,
                                     config.max_dilation_m,
                                     config.clip_dilation_window_to_max);
  if (window.first >= filtration.size()) {
    return {};
  }

  if (config.clip_dilation_window_to_max) {
    const double window_size =
        filtration[window.second].distance - filtration[window.first].distance;
    if (window_size < config.min_window_size) {
      VLOG(2) << "[RoomFinder] Bad window bounds: [" << config.min_dilation_m << ", "
              << config.max_dilation_m << "],  window: [" << window.first << ", "
              << window.second << "]" << " with size: " << window_size;

      window = getTrimmedFiltration(
          filtration, config.min_dilation_m, config.max_dilation_m, false);
    }
  }

  VLOG(2) << "[RoomFinder] Bounds: [" << config.min_dilation_m << ", "
          << config.max_dilation_m << "],  window: [" << window.first << ", "
          << window.second << "]";

  std::optional<FiltrationInfo> candidate = std::nullopt;
  switch (config.dilation_threshold_mode) {
    case DilationThresholdMode::REPEATED:
      candidate = getLongestSequence(filtration, window.first, window.second);
      break;
    case DilationThresholdMode::LONGEST_LIFETIME:
      candidate = getLongestLifetimeDilation(filtration,
                                             tracker.barcodes,
                                             config.min_lifetime_length_m,
                                             window.first,
                                             window.second);
      break;
    case DilationThresholdMode::PLATEAU:
    case DilationThresholdMode::PLATEAU_THRESHOLD:
    default:
      const bool use_threshold =
          config.dilation_threshold_mode == DilationThresholdMode::PLATEAU_THRESHOLD;
      candidate = getBestPlateau(
          filtration,
          use_threshold ? config.min_lifetime_length_m : config.plateau_ratio,
          window.first,
          window.second,
          use_threshold);
      break;
  }

  if (!candidate) {
    return {};
  }

  const auto info = *candidate;
  VLOG(2) << "[RoomFinder] Best threshold: " << info.distance << " ("
          << info.num_components << " components)";

  if (!config.log_path.empty()) {
    if (config.log_place_graphs) {
      LOG(FATAL) << "Not implemented currently";
    }

    logFiltration(filtration, window, info.distance);
    logged_once_ = true;
  }

  const auto components = graph_utilities::getConnectedComponents(
      places,
      [&](const SceneGraphNode& node) {
        return (*distance_adaptor_)(node) > info.distance;
      },
      [&](const SceneGraphEdge& edge) {
        return (*distance_adaptor_)(edge) > info.distance;
      });

  InitialClusters filtered;
  for (const auto& component : components) {
    if (component.size() < config.min_component_size) {
      continue;
    }

    filtered.push_back(component);
  }

  return filtered;
}

void RoomFinder::setupDistanceAdaptor(const SceneGraphLayer& places) {
  // NOTE(lschmid): For now try to figure out which adaptor to use based on the places.
  // Assumes there is only one kind of place attributes in the layer.
  if (places.numNodes() == 0) {
    return;
  }
  const auto& node = places.nodes().begin()->second;
  const auto place_attrs = node->tryAttributes<spark_dsg::PlaceNodeAttributes>();
  if (place_attrs) {
    distance_adaptor_ = std::make_unique<DistanceAdaptor>();
    return;
  }
  const auto trav_attrs =
      node->tryAttributes<spark_dsg::TraversabilityNodeAttributes>();
  if (trav_attrs) {
    distance_adaptor_ = std::make_unique<TraversabilityDistanceAdaptor>(places);
    return;
  }

  LOG(ERROR) << "[RoomFinder] Unknown place attributes to create distance adaptor.";
}

SceneGraphLayer::Ptr RoomFinder::findRooms(const SceneGraphLayer& places) {
  VLOG(2) << "[Room Finder] Detecting rooms for " << places.numNodes() << " nodes";
  setupDistanceAdaptor(places);

  const auto components = getBestComponents(places);
  if (components.empty()) {
    VLOG(2) << "[Room Finder] No components found";
    return nullptr;
  }

  last_results_.clear();
  switch (config.clustering_mode) {
    case RoomClusterMode::MODULARITY:
      last_results_ = clusterGraphByModularity(
          places, components, config.max_modularity_iters, config.modularity_gamma);
      break;
    case RoomClusterMode::MODULARITY_DISTANCE:
      last_results_ = clusterGraphByModularity(
          places,
          components,
          [](const SceneGraphLayer& G, NodeId n1, NodeId n2) {
            // weight should be 1 / distance
            return 1.0 / (getNodePosition(G, n1) - getNodePosition(G, n2)).norm();
          },
          config.max_modularity_iters,
          config.modularity_gamma);
      break;
    case RoomClusterMode::NEIGHBORS:
      last_results_ = clusterGraphByNeighbors(places, components);
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
  SceneGraphLayer::Ptr rooms(new SceneGraphLayer(DsgLayers::ROOMS));

  // organize rooms by their oldest place
  IndexTimePairQueue queue;
  fillIndexQueue(places, last_results_.clusters, queue, config.min_room_size);

  NodeSymbol room_id(config.room_prefix, 0);
  while (!queue.empty()) {
    const auto cluster_index = queue.top().index;
    queue.pop();

    cluster_room_map_[cluster_index] = room_id;
    const auto& cluster = last_results_.clusters.at(cluster_index);

    auto attrs = std::make_unique<RoomNodeAttributes>();
    // TODO(nathan) define unknown label somewhere
    attrs->semantic_label = 0;
    attrs->position = getRoomPosition(places, cluster, *distance_adaptor_);
    rooms->emplaceNode(room_id, std::move(attrs));
    ++room_id;
  }

  addEdgesToRoomLayer(places, last_results_.labels, cluster_room_map_, *rooms);
  return rooms;
}

void RoomFinder::addRoomPlaceEdges(DynamicSceneGraph& graph,
                                   const std::string& layer) const {
  for (const auto& id_node_pair : graph.getLayer(layer).nodes()) {
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
