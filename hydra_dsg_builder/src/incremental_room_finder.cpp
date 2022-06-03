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
#include "hydra_dsg_builder/incremental_room_finder.h"

#include <hydra_utils/timing_utilities.h>
#include <spark_dsg/adjacency_matrix.h>
#include <voxblox/core/color.h>

#include <Eigen/Dense>

#include <algorithm>

namespace hydra {
namespace incremental {

using hydra::timing::ScopedTimer;

// TODO(nathan) read in qualitative cmap
inline voxblox::Color getRoomColor(const NodeId& room_id) {
  static std::vector<double> taps{0.0, 0.1, 0.8, 0.35, 0.55, 0.9, 0.05, 0.7, 0.2, 0.65};
  return voxblox::rainbowColorMap(taps.at(room_id % taps.size()));
}

ClusterResults createClustersFromComponents(const Components& components) {
  ClusterResults to_return;
  to_return.valid = true;
  to_return.total_iters = 0;
  for (size_t i = 0; i < components.size(); ++i) {
    const auto& component = components.at(i);
    to_return.clusters[i] =
        std::unordered_set<NodeId>(component.begin(), component.end());
    for (const auto& node_id : component) {
      to_return.labels[node_id] = i;
    }
  }

  return to_return;
}

Eigen::MatrixXd getEigenvectorsDense(const SceneGraphLayer& layer,
                                     const std::map<NodeId, size_t> ordering,
                                     size_t k) {
  Eigen::MatrixXd L = getLaplacian(layer, ordering, [&](NodeId source, NodeId target) {
    return layer.getEdge(source, target).value().get().info->weight;
  });
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(L);
  Eigen::MatrixXd v = solver.eigenvectors().block(0, 0, L.rows(), k);
  return v;
}

ClusterResults clusterGraph(const SceneGraphLayer& layer,
                            const Components& components,
                            size_t max_iters,
                            bool use_sparse) {
  std::map<NodeId, size_t> ordering;
  size_t index = 0;
  for (const auto& id_node_pair : layer.nodes()) {
    ordering[id_node_pair.first] = index;
    ++index;
  }

  // seed means with component values
  const size_t k = components.size();
  Eigen::MatrixXd v;
  if (use_sparse) {
    LOG(ERROR) << "Sparse Eigen Decomposition not supported";
    return {};
  } else {
    v = getEigenvectorsDense(layer, ordering, k);
  }

  Eigen::MatrixXd means = Eigen::MatrixXd::Zero(components.size(), components.size());

  std::unordered_set<NodeId> fixed;
  std::map<NodeId, size_t> labels;
  std::map<size_t, size_t> cluster_sizes;

  for (size_t i = 0; i < components.size(); ++i) {
    const auto& component = components[i];
    for (const auto& node_id : component) {
      means.row(i) += v.row(ordering.at(node_id));
      fixed.insert(node_id);
      labels[node_id] = i;
    }

    means.row(i) /= component.size();
    cluster_sizes[i] = component.size();
  }

  std::map<NodeId, size_t> prev_labels = labels;
  size_t iter;  // for statistics
  for (iter = 0; iter < max_iters; ++iter) {
    // assign unfixed nodes to cluster
    for (const auto& id_node_pair : layer.nodes()) {
      if (fixed.count(id_node_pair.first)) {
        continue;
      }

      double min_distance = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < k; ++i) {
        const double curr_distance =
            (v.row(ordering.at(id_node_pair.first)) - means.row(i)).norm();
        if (curr_distance < min_distance) {
          if (labels.count(id_node_pair.first)) {
            cluster_sizes[labels.at(id_node_pair.first)] -= 1;
          }

          labels[id_node_pair.first] = i;
          cluster_sizes[i] += 1;
          min_distance = curr_distance;
        }
      }
    }

    if (prev_labels == labels) {
      break;
    }
    prev_labels = labels;

    // compute cluster sums
    means.setZero();
    for (const auto& id_cluster_pair : labels) {
      means.row(id_cluster_pair.second) += v.row(ordering.at(id_cluster_pair.first));
    }

    // convert to mean
    for (const auto& cluster_size_pair : cluster_sizes) {
      means.row(cluster_size_pair.first) /= cluster_size_pair.second;
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

ClusterResults clusterGraphByModularity(const SceneGraphLayer& layer,
                                        const Components& components,
                                        size_t max_iters,
                                        double gamma) {
  std::map<NodeId, double> degrees;
  std::map<NodeId, std::map<NodeId, double>> neighbors;
  for (const auto& id_node_pair : layer.nodes()) {
    double degree = 0.0;
    neighbors[id_node_pair.first] = std::map<NodeId, double>();
    for (const auto& sibling : id_node_pair.second->siblings()) {
      const double edge_weight =
          layer.getEdge(id_node_pair.first, sibling).value().get().info->weight;
      degree += edge_weight;
      neighbors[id_node_pair.first][sibling] = edge_weight;
    }
    degrees[id_node_pair.first] = degree;
  }

  const double m = layer.numEdges();

  std::map<size_t, double> community_degrees;
  std::map<NodeId, size_t> labels;
  for (size_t i = 0; i < components.size(); ++i) {
    const auto& component = components[i];
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
      size_t best_community = components.size();
      for (const auto& id_weight_pair : community_weights) {
        const double gain =
            2 * id_weight_pair.second -
            gamma * (community_degrees.at(id_weight_pair.first) * node_degree) / m;
        if (gain > best_gain) {
          best_gain = gain;
          best_community = id_weight_pair.first;
        }
      }

      if (best_community == components.size()) {
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

Components filterComponents(const Components& original, size_t min_size) {
  std::vector<std::vector<NodeId>> filtered;
  filtered.reserve(original.size());
  for (const auto& component : original) {
    if (component.size() < min_size) {
      continue;
    }

    filtered.push_back(component);
  }

  return filtered;
}

RoomMap getPreviousPlaceRoomMap(const DynamicSceneGraph& graph,
                                const SceneGraphLayer& active_places) {
  RoomMap active_rooms;

  for (const auto& id_node_pair : active_places.nodes()) {
    std::optional<NodeId> parent = graph.getNode(id_node_pair.first)->get().getParent();
    if (parent) {
      const SceneGraphNode& room_node = graph.getNode(*parent).value();
      if (active_rooms.count(room_node.id)) {
        continue;
      }

      active_rooms[room_node.id] = room_node.children();
    }
  }
  return active_rooms;
}

void addMissingEdges(const DynamicSceneGraph& graph,
                     SceneGraphLayer& new_layer,
                     const SceneGraphNode& node) {
  for (const auto& sibling : node.siblings()) {
    if (!new_layer.hasNode(sibling)) {
      continue;
    }

    if (new_layer.hasEdge(node.id, sibling)) {
      continue;
    }

    const auto& info = *(graph.getEdge(node.id, sibling)->get().info);
    new_layer.insertEdge(node.id, sibling, std::make_unique<EdgeAttributes>(info));
  }
}

IsolatedSceneGraphLayer::Ptr getActiveSubgraph(const DynamicSceneGraph& graph,
                                               LayerId layer_id,
                                               const ActiveNodeSet& active_nodes) {
  IsolatedSceneGraphLayer::Ptr subgraph(new IsolatedSceneGraphLayer(layer_id));

  for (const auto& node_id : active_nodes) {
    if (!graph.hasNode(node_id)) {
      // ideally, the active nodes would all be valid
      continue;
    }

    const SceneGraphNode& node = *graph.getNode(node_id);
    const auto& attrs = node.attributes<PlaceNodeAttributes>();
    PlaceNodeAttributes::Ptr new_attrs(new PlaceNodeAttributes(attrs));
    subgraph->emplaceNode(node_id, std::move(new_attrs));

    addMissingEdges(graph, *subgraph, node);
  }

  return subgraph;
}

template <typename A, typename B>
double getOverlap(const A& lhs, const B& rhs) {
  if (lhs.empty() || rhs.empty()) {
    return 0.0;
  }

  size_t num_same = 0;
  for (const auto& entry : lhs) {
    num_same += rhs.count(entry);
  }

  const double min_size = std::min(lhs.size(), rhs.size());
  return static_cast<double>(num_same) / min_size;
}

std::map<NodeId, size_t> mapRoomsToClusters(const ClusterResults& cluster_results,
                                            const RoomMap& previous_rooms,
                                            double min_overlap) {
  std::map<NodeId, size_t> best_clusters;
  for (const auto& room_contents_pair : previous_rooms) {
    // set iou to be outside feasible range to ensure best_cluster is initialized if
    // used
    double best_overlap = -1.0;
    size_t best_cluster = 0;
    for (const auto& id_cluster_pair : cluster_results.clusters) {
      const double overlap =
          getOverlap(id_cluster_pair.second, room_contents_pair.second);
      if (overlap > best_overlap) {
        best_overlap = overlap;
        best_cluster = id_cluster_pair.first;
      }
    }

    if (best_overlap < min_overlap) {
      continue;
    }

    best_clusters[room_contents_pair.first] = best_cluster;
  }

  return best_clusters;
}

void addNewRoomNode(DynamicSceneGraph& graph, NodeSymbol node_id, uint8_t label) {
  RoomNodeAttributes::Ptr room_attrs = std::make_unique<RoomNodeAttributes>();
  room_attrs->semantic_label = label;
  room_attrs->name = node_id.getLabel();
  const voxblox::Color& room_color = getRoomColor(node_id.categoryId());
  room_attrs->color << room_color.r, room_color.g, room_color.b;

  graph.emplaceNode(
      static_cast<LayerId>(DsgLayers::ROOMS), node_id, std::move(room_attrs));
}

void updateRoomCentroid(const DynamicSceneGraph& graph, NodeId room_id) {
  const SceneGraphNode& room = graph.getNode(room_id).value();
  if (!room.hasChildren()) {
    return;
  }

  Eigen::Vector3d room_position = Eigen::Vector3d::Zero();
  for (const auto& child : room.children()) {
    room_position += graph.getPosition(child);
  }
  room_position /= room.children().size();

  bool room_in_freespace = false;
  double best_distance = std::numeric_limits<double>::infinity();
  // this gets overwritten before it gets used
  NodeId best_node = 0;
  for (const auto& child : room.children()) {
    const auto& attrs =
        graph.getNode(child).value().get().attributes<PlaceNodeAttributes>();
    double room_distance = (room_position - attrs.position).norm();
    if (room_distance <= attrs.distance) {
      room_in_freespace = true;
      break;
    }

    double distance_to_freespace = room_distance - attrs.distance;
    if (distance_to_freespace < best_distance) {
      best_distance = distance_to_freespace;
      best_node = child;
    }
  }

  if (room_in_freespace) {
    room.attributes().position = room_position;
  } else {
    const auto& attrs =
        graph.getNode(best_node).value().get().attributes<PlaceNodeAttributes>();
    // project room to edge of nearest place freespace
    room.attributes().position =
        attrs.position + (room_position - attrs.position).normalized() * attrs.distance;
  }
}

RoomFinder::RoomFinder(const RoomFinder::Config& config)
    : config_(config), next_room_id_(config.room_prefix, 0) {}

std::vector<double> RoomFinder::getThresholds() const {
  std::vector<double> thresholds;
  thresholds.push_back(config_.min_dilation_m);
  thresholds.push_back(config_.max_dilation_m);

  const double step =
      (config_.max_dilation_m - config_.min_dilation_m) / (config_.num_steps + 1);
  for (size_t i = 1; i <= config_.num_steps; ++i) {
    thresholds.push_back(config_.min_dilation_m + i * step);
  }

  std::sort(thresholds.begin(), thresholds.end());
  return thresholds;
}

std::optional<size_t> getLongestSequence(const std::vector<size_t>& values) {
  if (values.empty()) {
    return std::nullopt;
  }

  if (!values.front()) {
    return std::nullopt;
  }

  if (values.size() == 1) {
    return 0;
  }

  size_t best_start = 0;
  size_t best_count = 0;

  size_t start = 0;
  size_t count = 1;
  for (size_t i = 1; i < values.size(); ++i) {
    if (values[i] == values[i - 1]) {
      ++count;
      continue;
    }

    // we've broken the sequence
    if (count > best_count) {
      best_count = count;
      best_start = start;
    }

    if (!values[i]) {
      break;
    }

    start = i;
    count = 1;
  }

  if (count > best_count) {
    best_count = count;
    best_start = start;
  }

  if (!best_count) {
    return std::nullopt;
  }

  return best_start;
}

std::optional<size_t> getMedianComponentSize(const std::vector<size_t>& values) {
  if (values.empty()) {
    return std::nullopt;
  }

  if (!values.front()) {
    return std::nullopt;
  }

  if (values.size() == 1) {
    return 0;
  }

  size_t max_value = 0;
  size_t max_index = 0;
  for (size_t i = 0; i < values.size(); ++i) {
    if (values.at(i) >= max_value) {
      max_value = values.at(i);
      max_index = i;
    }
  }

  std::vector<size_t> valid_values(values.begin(), values.begin() + max_index + 1);
  std::sort(valid_values.begin(), valid_values.end());
  const size_t median_idx =
      std::ceil(static_cast<double>(valid_values.size() - 1) / 2.0);
  const size_t median_value = valid_values[median_idx];
  for (size_t i = 0; i < max_index; ++i) {
    if (values[i] == median_value) {
      return i;
    }
  }

  // should never be reached
  return std::nullopt;
}

Components RoomFinder::getBestComponents(const SceneGraphLayer& places,
                                         const std::vector<double>& thresholds) const {
  std::vector<size_t> num_components;
  for (const auto& threshold : thresholds) {
    auto components_orig = graph_utilities::getConnectedComponents(
        places,
        [&](const SceneGraphNode& node) {
          return node.attributes<PlaceNodeAttributes>().distance > threshold;
        },
        [&](const SceneGraphEdge& edge) { return edge.info->weight > threshold; });
    auto components = filterComponents(components_orig, config_.min_component_size);
    num_components.push_back(components.size());
  }

  VLOG(3) << "Component sequence: " << displayNodeSymbolContainer(num_components);
  // auto best_sequence_start = getLongestSequence(num_components);
  auto best_sequence_start = getMedianComponentSize(num_components);
  if (!best_sequence_start) {
    return Components();
  }

  const double best_threshold = thresholds.at(*best_sequence_start);
  VLOG(3) << " Best threshold: " << best_threshold << " (index " << *best_sequence_start
          << ")";

  auto best_components = graph_utilities::getConnectedComponents(
      places,
      [&](const SceneGraphNode& node) {
        return node.attributes<PlaceNodeAttributes>().distance > best_threshold;
      },
      [&](const SceneGraphEdge& edge) { return edge.info->weight > best_threshold; });
  return filterComponents(best_components, config_.min_component_size);
}

void pruneClusters(const DynamicSceneGraph& graph, ClusterResults& results) {
  auto iter = results.clusters.begin();
  while (iter != results.clusters.end()) {
    auto node_iter = iter->second.begin();
    while (node_iter != iter->second.end()) {
      if (!graph.hasNode(*node_iter)) {
        node_iter = iter->second.erase(node_iter);
      } else {
        ++node_iter;
      }
    }

    if (iter->second.empty()) {
      iter = results.clusters.erase(iter);
    } else {
      ++iter;
    }
  }
}

using RoomClusterMap = std::map<NodeId, size_t>;

struct IndexTimePair {
  size_t index;
  uint64_t time_ns;

  inline bool operator>(const IndexTimePair& other) const {
    return time_ns > other.time_ns;
  }
};

using IndexTimePairQueue = std::priority_queue<IndexTimePair,
                                               std::vector<IndexTimePair>,
                                               std::greater<IndexTimePair>>;

void RoomFinder::assignRooms(SharedDsgInfo& dsg, ClusterResults& cluster_results) {
  pruneClusters(*dsg.graph, cluster_results);

  IndexTimePairQueue index_queue;
  for (const auto id_cluster_pair : cluster_results.clusters) {
    if (id_cluster_pair.second.size() < config_.min_room_size) {
      continue;
    }

    uint64_t oldest_time_ns = std::numeric_limits<uint64_t>::max();
    for (const auto& place : id_cluster_pair.second) {
      const auto& attrs = dsg.graph->getNode(place)->get().attributes();
      if (attrs.last_update_time_ns < oldest_time_ns) {
        oldest_time_ns = attrs.last_update_time_ns;
      }
    }

    index_queue.push({id_cluster_pair.first, oldest_time_ns});
  }

  NodeSymbol next_id(config_.room_prefix, 0);
  while (!index_queue.empty()) {
    const auto next_cluster = index_queue.top();
    index_queue.pop();

    addNewRoomNode(*dsg.graph, next_id, config_.room_semantic_label);

    const auto& cluster = cluster_results.clusters.at(next_cluster.index);
    for (const auto& node_id : cluster) {
      dsg.graph->insertEdge(next_id, node_id);
    }

    incremental::updateRoomCentroid(*dsg.graph, next_id);
    ++next_id;
  }
}

void RoomFinder::updateRoomsFromClusters(SharedDsgInfo& dsg,
                                         ClusterResults& cluster_results,
                                         const RoomMap& previous_rooms,
                                         const ActiveNodeSet& active_nodes) {
  pruneClusters(*dsg.graph, cluster_results);

  std::map<NodeId, size_t> rooms_to_clusters = mapRoomsToClusters(
      cluster_results, previous_rooms, config_.room_vote_min_overlap);

  std::map<size_t, NodeId> clusters_to_rooms;
  for (const auto& room_cluster_pair : rooms_to_clusters) {
    clusters_to_rooms[room_cluster_pair.second] = room_cluster_pair.first;
  }

  std::set<NodeId> seen_rooms;
  std::set<NodeId> seen_nodes;
  for (const auto id_cluster_pair : cluster_results.clusters) {
    const bool has_association = clusters_to_rooms.count(id_cluster_pair.first);
    if (!has_association && id_cluster_pair.second.size() < config_.min_room_size) {
      // we can reject new nodes immediately, but previous room nodes may meet min
      // size threshold by union of previous nodes and the new associated cluster
      continue;
    }

    NodeId room_id;
    if (has_association) {
      room_id = clusters_to_rooms.at(id_cluster_pair.first);
    } else {
      room_id = next_room_id_;
      addNewRoomNode(*dsg.graph, next_room_id_, config_.room_semantic_label);
      next_room_id_++;
    }
    seen_rooms.insert(room_id);

    for (const auto& node_id : id_cluster_pair.second) {
      const SceneGraphNode& child_node = dsg.graph->getNode(node_id).value();
      auto parent = child_node.getParent();
      if (parent && *parent != room_id) {
        dsg.graph->removeEdge(*parent, node_id);
      }

      dsg.graph->insertEdge(room_id, node_id);
      seen_nodes.insert(node_id);
    }
  }

  std::set<NodeId> empty_rooms;
  const auto& rooms = dsg.graph->getLayer(DsgLayers::ROOMS);
  for (const auto& id_node_pair : rooms.nodes()) {
    if (id_node_pair.second->children().size() < config_.min_room_size) {
      empty_rooms.insert(id_node_pair.first);
    } else {
      incremental::updateRoomCentroid(*dsg.graph, id_node_pair.first);
    }
  }

  for (const auto& room : empty_rooms) {
    dsg.graph->removeNode(room);
  }

  for (const auto& node_id : active_nodes) {
    if (seen_nodes.count(node_id)) {
      continue;
    }

    if (!dsg.graph->hasNode(node_id)) {
      continue;
    }

    const SceneGraphNode& node = dsg.graph->getNode(node_id).value();
    std::optional<NodeId> parent = node.getParent();
    if (parent) {
      dsg.graph->removeEdge(node_id, *parent);
    }
  }
}

void updateRoomEdges(DynamicSceneGraph& graph,
                     const RoomMap& previous_rooms,
                     const ActiveNodeSet& active_nodes) {
  if (previous_rooms.size() > 1) {
    auto first_iter = previous_rooms.begin();
    while (first_iter != previous_rooms.end()) {
      auto second_iter = first_iter;
      ++second_iter;
      while (second_iter != previous_rooms.end()) {
        graph.removeEdge(first_iter->first, second_iter->first);
        ++second_iter;
      }

      ++first_iter;
    }
  }

  for (const auto& node_id : active_nodes) {
    auto node = graph.getNode(node_id);
    if (!node) {
      continue;
    }
    auto parent = node->get().getParent();

    for (const auto& sibling_id : node->get().siblings()) {
      const SceneGraphNode& sibling = graph.getNode(sibling_id).value();
      auto sibling_parent = sibling.getParent();
      if (!parent || !sibling_parent || *parent == *sibling_parent) {
        continue;
      }

      // TODO(nathan) not necessary, but nice logically
      if (graph.hasEdge(*parent, *sibling_parent)) {
        continue;
      }

      graph.insertEdge(*parent, *sibling_parent);
    }
  }
}

void removeOldRooms(SharedDsgInfo& dsg) {
  std::vector<NodeId> to_remove;
  const auto& rooms = dsg.graph->getLayer(DsgLayers::ROOMS);
  for (const auto& id_node_pair : rooms.nodes()) {
    to_remove.push_back(id_node_pair.first);
  }

  for (const auto node_id : to_remove) {
    dsg.graph->removeNode(node_id);
  }
}

void RoomFinder::findRooms(SharedDsgInfo& dsg, const ActiveNodeSet& active_nodes) {
  RoomMap previous_rooms;
  IsolatedSceneGraphLayer::Ptr active_places;
  {  // start dsg critical section
    std::unique_lock<std::mutex> graph_lock(dsg.mutex);
    active_places = getActiveSubgraph(
        *dsg.graph, static_cast<LayerId>(DsgLayers::PLACES), active_nodes);

    if (config_.use_previous_rooms) {
      previous_rooms = getPreviousPlaceRoomMap(*dsg.graph, *active_places);
    }

  }  // end dsg critical section

  std::vector<double> thresholds = getThresholds();

  Components components = getBestComponents(*active_places, thresholds);
  if (components.empty()) {
    VLOG(1) << "No rooms found";
    return;
  }

  ClusterResults clusters;

  {  // clustering scope
    ScopedTimer timer("frontend/room_clustering", true, 2, true);
    switch (config_.clustering_mode) {
      case Config::ClusterMode::SPECTRAL:
        clusters = clusterGraph(*active_places,
                                components,
                                config_.max_kmeans_iters,
                                config_.use_sparse_eigen_decomp);
        break;
      case Config::ClusterMode::MODULARITY:
        clusters = clusterGraphByModularity(*active_places,
                                            components,
                                            config_.max_modularity_iters,
                                            config_.modularity_gamma);
        break;
      case Config::ClusterMode::NONE:
      default:
        clusters = createClustersFromComponents(components);
        break;
    }
  }  // clustering scope

  if (!clusters.valid) {
    LOG(WARNING) << "[Room Finder] Room clustering failed. Keeping previous rooms!";
    return;
  }

  {  // start dsg critical section
    std::unique_lock<std::mutex> graph_lock(dsg.mutex);
    if (config_.use_previous_rooms) {
      updateRoomsFromClusters(dsg, clusters, previous_rooms, active_nodes);
    } else {
      removeOldRooms(dsg);
      assignRooms(dsg, clusters);
    }
    updateRoomEdges(*dsg.graph, previous_rooms, active_nodes);
  }  // end dsg critical section
}

}  // namespace incremental
}  // namespace hydra
