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
#include "hydra/rooms/graph_filtration.h"

#include <glog/logging.h>

#include <iomanip>

namespace hydra {

BarcodeTracker::BarcodeTracker() : BarcodeTracker(0) {}

BarcodeTracker::BarcodeTracker(size_t min_component_size)
    : min_component_size(min_component_size) {}

void BarcodeTracker::addNode(NodeId node, double distance) {
  if (min_component_size <= 1) {
    barcodes.emplace(node, ComponentLifetime{0.0, distance});
  }
}

bool BarcodeTracker::doUnion(DisjointSet& components,
                             const std::unordered_map<NodeId, double>& node_distances,
                             NodeId lhs,
                             NodeId rhs,
                             double distance) {
  NodeId lhs_set = components.findSet(lhs);
  NodeId rhs_set = components.findSet(rhs);

  // note that this works: nothing in disjoint set relies on lhs and rhs, just their
  // parents
  const auto rhs_better = node_distances.at(rhs_set) >= node_distances.at(lhs_set);
  const auto erased = components.doUnion(lhs_set, rhs_set, rhs_better);
  if (!erased) {
    return false;
  }

  if (erased.value() == lhs_set) {
    // mirror the ordering in disjointSet. rhs is always erased, so
    // if lhs is the return value from doUnion, lhs was erased
    std::swap(lhs_set, rhs_set);
  }

  auto liter = barcodes.find(lhs_set);
  const auto new_size = components.sizes.at(lhs_set);
  if (liter == barcodes.end() && new_size >= min_component_size) {
    // mark start of new component
    liter = barcodes.emplace(lhs_set, ComponentLifetime{0.0, distance}).first;
  }

  auto riter = barcodes.find(rhs_set);
  if (riter != barcodes.end()) {
    // mark exit of component
    riter->second.start = distance;
  }

  return true;
}

struct Entry {
  double distance;
  NodeId source;
  std::optional<NodeId> target;

  Entry() : Entry(0.0, 0) {}

  Entry(double distance, NodeId source)
      : distance(distance), source(source), target(std::nullopt) {}

  Entry(double distance, NodeId source, NodeId target)
      : distance(distance), source(source), target(target) {}

  inline bool operator<(const Entry& other) const { return distance < other.distance; }
};

struct NodePair {
  NodeId source;
  NodeId target;
};

using UnusedEdgeMap = std::map<NodeId, std::list<NodePair>>;

std::ostream& operator<<(std::ostream& out, const FiltrationInfo& info) {
  return out << "<dist=" << info.distance << ", size=" << info.num_components << ">";
}

std::ostream& operator<<(std::ostream& out, const Filtration& info) {
  auto iter = info.begin();
  out << "[";
  out << std::setprecision(3);
  while (iter != info.end()) {
    out << "{d=" << iter->distance << ", s=" << iter->num_components << "}";
    ++iter;
    if (iter != info.end()) {
      out << ", ";
    }
  }
  out << "]";
  return out;
}

std::ostream& operator<<(std::ostream& out, const Entry& e) {
  return out << "<dist=" << e.distance << ", s=" << e.source
             << ", t=" << (e.target ? std::to_string(e.target.value()) : "n/a") << ">";
}

void fillEntries(const SceneGraphLayer& layer,
                 std::vector<Entry>& entries,
                 std::unordered_map<NodeId, double>& node_distances,
                 bool include_nodes) {
  entries.reserve(layer.edges().size() + layer.nodes().size());

  for (const auto& id_edge_pair : layer.edges()) {
    const auto& edge = id_edge_pair.second;
    entries.push_back({edge.info->weight, edge.source, edge.target});
  }

  for (auto&& [id, node] : layer.nodes()) {
    const auto distance = node->attributes<PlaceNodeAttributes>().distance;
    node_distances.emplace(id, distance);

    if (include_nodes) {
      entries.push_back({distance, id});
    }
  }

  std::make_heap(entries.begin(), entries.end());
}

bool updateComponentsFromEdge(NodeId source,
                              NodeId target,
                              double edge_distance,
                              DisjointSet& components,
                              BarcodeTracker& tracker,
                              UnusedEdgeMap& unused_edges,
                              std::unordered_map<NodeId, double>& node_distances) {
  const bool has_source = components.hasSet(source);
  const bool has_target = components.hasSet(target);

  if (!has_source) {
    auto iter = unused_edges.find(source);
    if (iter == unused_edges.end()) {
      iter = unused_edges.emplace(source, std::list<NodePair>()).first;
    }
    iter->second.push_back({source, target});
  }

  if (!has_target) {
    auto iter = unused_edges.find(target);
    if (iter == unused_edges.end()) {
      iter = unused_edges.emplace(target, std::list<NodePair>()).first;
    }
    iter->second.push_back({source, target});
  }

  if (!has_source || !has_target) {
    return false;
  }

  tracker.doUnion(components, node_distances, source, target, edge_distance);
  return true;
}

void updateComponentsFromNode(NodeId node,
                              DisjointSet& components,
                              BarcodeTracker& tracker,
                              UnusedEdgeMap& unused_edges,
                              std::unordered_map<NodeId, double>& node_distances) {
  // create new set for the node
  const auto curr_distance = node_distances.at(node);
  if (components.addSet(node)) {
    tracker.addNode(node, curr_distance);
  }

  // check if there were any edges pending on this node
  const auto edge_iter = unused_edges.find(node);
  if (edge_iter == unused_edges.end()) {
    return;
  }

  // check if any pending edges can be added back in
  for (const auto& edge : edge_iter->second) {
    if (!components.hasSet(edge.source) || !components.hasSet(edge.target)) {
      // this will be handled once the other node ends up being valid
      continue;
    }

    // "delayed" edges, i.e. those where a member node has to be added before they can
    // be added to the disjoint set, get assigned the distance of the node that needed
    // to be added last to the lifetime
    tracker.doUnion(
        components, node_distances, edge.source, edge.target, curr_distance);
  }

  unused_edges.erase(edge_iter);
}

Filtration getGraphFiltration(const SceneGraphLayer& layer, double diff_threshold_m) {
  BarcodeTracker tracker;
  return getGraphFiltration(
      layer, tracker, diff_threshold_m, [](const DisjointSet& components) -> size_t {
        return components.sizes.size();
      });
}

Filtration getGraphFiltration(const SceneGraphLayer& layer,
                              size_t min_component_size,
                              double diff_threshold_m) {
  BarcodeTracker tracker;
  return getGraphFiltration(
      layer,
      tracker,
      diff_threshold_m,
      [&min_component_size](const DisjointSet& components) -> size_t {
        size_t num_components = 0;
        for (const auto id_size_pair : components.sizes) {
          if (id_size_pair.second >= min_component_size) {
            ++num_components;
          }
        }
        return num_components;
      });
}

Filtration getGraphFiltration(const SceneGraphLayer& layer,
                              BarcodeTracker& tracker,
                              double diff_threshold_m,
                              const ComponentCallback& count_components,
                              bool include_nodes) {
  std::vector<Entry> entries;
  std::unordered_map<NodeId, double> node_distances;
  fillEntries(layer, entries, node_distances, include_nodes);

  DisjointSet components;
  UnusedEdgeMap unused_edges;
  if (!include_nodes) {
    // seed components with all nodes if we're not including nodes in the filtration
    for (const auto& id_node_pair : layer.nodes()) {
      updateComponentsFromNode(
          id_node_pair.first, components, tracker, unused_edges, node_distances);
    }
  }

  std::list<FiltrationInfo> filtration;
  while (!entries.empty()) {
    std::pop_heap(entries.begin(), entries.end());
    const auto x = entries.back();
    entries.pop_back();

    VLOG(10) << "Processing " << x;

    bool change_in_components = true;
    if (x.target) {
      change_in_components = updateComponentsFromEdge(x.source,
                                                      x.target.value(),
                                                      x.distance,
                                                      components,
                                                      tracker,
                                                      unused_edges,
                                                      node_distances);
    } else {
      updateComponentsFromNode(
          x.source, components, tracker, unused_edges, node_distances);
    }

    if (!change_in_components) {
      continue;
    }

    const auto num_components = count_components(components);
    if (filtration.empty()) {
      filtration.push_front({x.distance, num_components});
      VLOG(10) << filtration.front();
      continue;
    }

    // this may help smooth the resulting filtration a little
    const double diff_m = std::abs(filtration.front().distance - x.distance);
    if (diff_m < diff_threshold_m) {
      filtration.front().num_components = num_components;
    } else {
      filtration.push_front({x.distance, num_components});
    }

    VLOG(10) << filtration.front();
  }

  return Filtration(filtration.begin(), filtration.end());
}

std::pair<size_t, size_t> getTrimmedFiltration(const Filtration& filtration,
                                               double min_dilation_m,
                                               double max_dilation_m,
                                               bool clip_to_max) {
  size_t start_index = 0;
  size_t end_index = filtration.size();

  size_t max_value = 0;
  size_t max_index = filtration.size();
  for (size_t i = 0; i < filtration.size(); ++i) {
    const auto& info = filtration[i];
    if (info.distance < min_dilation_m) {
      // once the dilation exceeds the minimum, this stops updating
      start_index = i + 1;
    }

    if (info.distance < max_dilation_m) {
      // once the dilation exceeds the minimum, this stops updating
      end_index = i;
    }

    if (info.num_components > max_value) {
      max_value = info.num_components;
      max_index = i;
    }
  }

  // if peak occurs before end of window, return window through peak instead
  if (clip_to_max && max_index < end_index) {
    end_index = max_index + 1;
  }

  return {start_index, end_index};
}

std::optional<FiltrationInfo> getLongestSequence(const Filtration& values,
                                                 size_t start_index,
                                                 size_t end_index) {
  if (start_index >= values.size()) {
    return std::nullopt;
  }

  if (end_index > values.size()) {
    end_index = values.size();
  }

  if (start_index == end_index) {
    return std::nullopt;
  }

  size_t best_start = 0;
  size_t best_count = 0;

  size_t start = 0;
  size_t count = 1;
  for (size_t i = start_index + 1; i < end_index; ++i) {
    if (values[i].num_components == values[i - 1].num_components) {
      if (!values[i].num_components) {
        count = 0;  // disable any zero-component entries from being considered
      } else {
        ++count;
      }
      continue;
    }

    // we've broken the sequence
    if (count > best_count) {
      best_count = count;
      best_start = start;
    }

    start = i;
    count = 1;
  }

  if (count > best_count) {
    best_count = count;
    best_start = start;
  }

  if (values[best_start].num_components) {
    return values[best_start];
  } else {
    return std::nullopt;
  }
}

size_t getMedianComponents(const Filtration& values,
                           size_t start_index,
                           size_t end_index) {
  std::vector<size_t> components;
  for (size_t i = start_index; i < end_index; ++i) {
    components.push_back(values.at(i).num_components);
  }

  std::sort(components.begin(), components.end());
  CHECK(!components.empty());
  const size_t median_idx = std::ceil(static_cast<double>(components.size() - 1) / 2.0);
  return components[median_idx];
}

double computeTotalLifetime(const std::vector<ComponentLifetime>& lifetimes,
                            double distance) {
  double curr_lifetime = 0.0;
  for (const auto& lifetime : lifetimes) {
    if (distance < lifetime.start || distance > lifetime.end) {
      continue;
    }

    curr_lifetime += lifetime.end - lifetime.start;
  }

  return curr_lifetime;
}

std::optional<FiltrationInfo> getLongestLifetimeDilation(const Filtration& values,
                                                         const LifetimeMap& lifetimes,
                                                         double min_component_lifetime,
                                                         size_t start_index,
                                                         size_t end_index) {
  if (start_index >= values.size()) {
    return std::nullopt;
  }

  if (end_index > values.size()) {
    end_index = values.size();
  }

  const double start_distance = values[start_index].distance;
  const double end_distance = values[end_index].distance;
  std::vector<ComponentLifetime> filtered_lifetimes;
  for (const auto& id_lifetime_pair : lifetimes) {
    const auto& lifetime = id_lifetime_pair.second;
    if (lifetime.start >= end_distance) {
      continue;
    }

    if (lifetime.end <= start_distance) {
      continue;
    }

    if (lifetime.end - lifetime.start < min_component_lifetime) {
      continue;
    }

    filtered_lifetimes.push_back(lifetime);
  }

  auto longest_index_start = getLongestSequence(values, start_index, end_index);
  size_t num_components;
  if (longest_index_start) {
    num_components = longest_index_start->num_components;
  } else {
    num_components = getMedianComponents(values, start_index, end_index);
  }

  double best_lifetime = 0.0;
  std::optional<size_t> best_index = std::nullopt;
  for (size_t i = start_index; i < end_index; ++i) {
    const auto& info = values[i];
    if (info.num_components != num_components) {
      continue;
    }

    const double curr_total = computeTotalLifetime(filtered_lifetimes, info.distance);
    if (curr_total > best_lifetime || !best_index) {
      best_index = i;
      best_lifetime = curr_total;
    }
  }

  if (!best_index) {
    return std::nullopt;
  }

  return values[*best_index];
}

std::optional<FiltrationInfo> getBestPlateau(const Filtration& values,
                                             double ratio,
                                             size_t start_index,
                                             size_t end_index,
                                             bool use_threshold) {
  if (start_index >= values.size()) {
    return std::nullopt;
  }

  if (end_index > values.size()) {
    end_index = values.size();
  }

  if (start_index == end_index) {
    return std::nullopt;
  }

  std::vector<size_t> seq_values{values[start_index].num_components};
  std::vector<std::pair<size_t, size_t>> sequences{{start_index, start_index}};
  for (size_t i = start_index + 1; i < end_index; ++i) {
    if (values[i].num_components != seq_values.back()) {
      seq_values.push_back(values[i].num_components);
      sequences.push_back({i, i});
    } else {
      sequences.back().second = i;
    }
  }

  std::vector<double> lifetimes;
  for (const auto& start_end_pair : sequences) {
    lifetimes.push_back(values[start_end_pair.second].distance -
                        values[start_end_pair.first].distance);
  }

  if (VLOG_IS_ON(20)) {
    VLOG(20) << "Sequences:";
    for (size_t i = 0; i < sequences.size(); ++i) {
      VLOG(20) << "  - " << i << ": " << seq_values.at(i) << " -> ["
               << sequences[i].first << ", " << sequences[i].second
               << "], lifetime=" << lifetimes[i];
    }
  }

  CHECK(!lifetimes.empty());
  const double max_lifetime = *std::max_element(lifetimes.begin(), lifetimes.end());
  const double threshold = use_threshold ? ratio : ratio * max_lifetime;
  VLOG(5) << "[Room Finder] Max lifetime: " << max_lifetime << ", Ratio: " << ratio
          << ", Threshold: " << threshold;

  size_t best_sequence = 0;
  size_t best_components = 0;
  VLOG(20) << "Best components:";
  for (size_t i = 0; i < lifetimes.size(); ++i) {
    if (lifetimes[i] < threshold) {
      VLOG(20) << "  - " << i << ": below threshold (" << lifetimes[i] << " < "
               << threshold << ")";
      continue;
    }

    const auto curr_components = values[sequences[i].first].num_components;
    VLOG(20) << "  - " << i << ": components=" << curr_components
             << " (best=" << best_components << ")";
    // TODO(nathan) might want >= instead
    if (curr_components > best_components) {
      best_sequence = i;
      best_components = curr_components;
    }
  }

  return values[sequences[best_sequence].first];
}

}  // namespace hydra
