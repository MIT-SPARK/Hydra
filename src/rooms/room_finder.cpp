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

void logFiltration(std::ostream& fout,
                   const Filtration& filtration,
                   const std::pair<size_t, size_t>& window,
                   double distance) {
  fout << "{";
  fout << "\"start\":" << window.first << ",\"end\":" << window.second
       << ",\"threshold\":" << distance << ",\"filtration\":[";
  auto iter = filtration.begin();
  while (iter != filtration.end()) {
    fout << "{\"c\":" << iter->num_components << ",\"d\":" << iter->distance << "}";
    ++iter;
    if (iter != filtration.end()) {
      fout << ",";
    }
  }
  fout << "]},";
}

RoomExtents load_room_extents(std::filesystem::path path) {
  if (path != "") {
    return RoomExtents(path);
  }
  return RoomExtents(std::vector<std::vector<spark_dsg::BoundingBox>>());
}

RoomFinder::RoomFinder(const RoomFinderConfig& config)
    : config(config::checkValid(config)),
      room_extents(load_room_extents(config.ground_truth_rooms_path)),
      distance_adaptor_(new DistanceAdaptor()) {}

RoomFinder::~RoomFinder() {
  if (log_file_) {
    if (logged_once_) {
      const size_t curr_pos = log_file_->tellp();
      log_file_->seekp(curr_pos - 1);
    }

    (*log_file_) << "]";
    if (graph_log_file_) {
      graph_log_file_->flush();
      graph_log_file_->close();
      graph_log_file_.reset();

      auto& fout = *log_file_;
      fout << ",\"offsets\":[";
      for (size_t i = 0; i < graph_entries_.size(); ++i) {
        const auto& entry = graph_entries_[i];
        fout << "{\"index\":" << i;
        fout << ",\"offset\":" << entry.offset;
        fout << ",\"size\":" << entry.size;
        fout << ",\"num_nodes\":" << entry.num_nodes;
        fout << ",\"num_edges\":" << entry.num_edges;
        fout << "}";
        if (i + 1 < graph_entries_.size()) {
          fout << ",";
        }
      }
      fout << "]";
    }

    (*log_file_) << "}" << std::endl;
    log_file_->flush();
    log_file_->close();
    log_file_.reset();
  }
}

void RoomFinder::enableLogging(const std::string& log_path) {
  if (!config.log_filtrations) {
    return;
  }

  log_file_.reset(new std::ofstream(log_path + ".json"));
  (*log_file_) << "{\"contents\":[";

  if (!config.log_place_graphs) {
    return;
  }

  const std::string gname = log_path + ".graphs";
  graph_log_file_.reset(new std::ofstream(gname, std::ios::binary));
}

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
              << window.second << "]"
              << " with size: " << window_size;

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

  if (log_file_) {
    if (graph_log_file_) {
      LOG(FATAL) << "Not implemented currently";
      /* const auto bson_str = places.toBson();*/
      /* graph_entries_.push_back(*/
      /*{graph_offset_, bson_str.size(), places.numNodes(), places.numEdges()});*/
      /*graph_log_file_->write(bson_str.data(), bson_str.size());*/
      /*graph_offset_ += bson_str.size();*/
    }

    logFiltration(*log_file_, filtration, window, info.distance);
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

  if (config.clustering_mode == RoomClusterMode::GROUND_TRUTH) {
    last_results_ = clusterGraphByGt(places, room_extents);
    cluster_room_map_.clear();
    return makeRoomLayer(places);
  }

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
    case RoomClusterMode::GROUND_TRUTH:
      last_results_ = clusterGraphByGt(places, room_extents);
      LOG(WARNING) << "Got GT results";
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

void RoomFinder::fillClusterMap(const SceneGraphLayer& places,
                                ClusterMap& assignments) const {
  assignments.clear();
  for (const auto& id_node_pair : places.nodes()) {
    const auto place_id = id_node_pair.first;

    const auto cluster = last_results_.labels.find(place_id);
    if (cluster == last_results_.labels.end()) {
      continue;
    }

    const auto room = cluster_room_map_.find(cluster->second);
    if (room == cluster_room_map_.end()) {
      continue;
    }

    const auto room_id = room->second;
    auto assignment = assignments.find(room_id);
    if (assignment == assignments.end()) {
      assignment = assignments.emplace(room_id, std::vector<NodeId>()).first;
    }

    assignment->second.push_back(place_id);
  }
}

}  // namespace hydra
