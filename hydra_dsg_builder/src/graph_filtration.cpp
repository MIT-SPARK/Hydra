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
#include "hydra_dsg_builder/graph_filtration.h"
#include "hydra_dsg_builder/minimum_spanning_tree.h"

#include <glog/logging.h>
#include <iomanip>

namespace hydra {

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

void fillEntries(const SceneGraphLayer& layer, std::vector<Entry>& entries) {
  entries.reserve(layer.edges().size() + layer.nodes().size());

  for (const auto& id_edge_pair : layer.edges()) {
    const auto& edge = id_edge_pair.second;
    entries.push_back({edge.info->weight, edge.source, edge.target});
  }

  for (auto&& [id, node] : layer.nodes()) {
    entries.push_back({node->attributes<PlaceNodeAttributes>().distance, id});
  }

  std::make_heap(entries.begin(), entries.end());
}

bool updateComponentsFromEdge(NodeId source,
                              NodeId target,
                              DisjointSet& components,
                              UnusedEdgeMap& unused_edges) {
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

  components.doUnion(source, target);
  return true;
}

void updateComponentsFromNode(NodeId node,
                              DisjointSet& components,
                              UnusedEdgeMap& unused_edges) {
  // create new set for the node
  components.addSet(node);

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

    components.doUnion(edge.source, edge.target);
  }

  unused_edges.erase(edge_iter);
}

Filtration getGraphFiltration(const SceneGraphLayer& layer, double diff_threshold_m) {
  return getGraphFiltration(
      layer, diff_threshold_m, [](const ComponentSizeMap& sizes) -> size_t {
        return sizes.size();
      });
}

Filtration getGraphFiltration(const SceneGraphLayer& layer,
                              size_t min_component_size,
                              double diff_threshold_m) {
  return getGraphFiltration(
      layer,
      diff_threshold_m,
      [&min_component_size](const ComponentSizeMap& sizes) -> size_t {
        size_t num_components = 0;
        for (const auto id_size_pair : sizes) {
          if (id_size_pair.second >= min_component_size) {
            ++num_components;
          }
        }
        return num_components;
      });
}

// TODO(nathan) lambda functions for filtering component sizes
Filtration getGraphFiltration(const SceneGraphLayer& layer,
                              double diff_threshold_m,
                              const ComponentCallback& count_components) {
  std::vector<Entry> entries;
  fillEntries(layer, entries);

  UnusedEdgeMap unused_edges;
  DisjointSet components;
  std::list<FiltrationInfo> filtration;
  while (!entries.empty()) {
    std::pop_heap(entries.begin(), entries.end());
    const auto x = entries.back();
    entries.pop_back();

    VLOG(10) << "Processing " << x;

    bool change_in_components = true;
    if (x.target) {
      change_in_components = updateComponentsFromEdge(
          x.source, x.target.value(), components, unused_edges);
    } else {
      updateComponentsFromNode(x.source, components, unused_edges);
    }

    if (!change_in_components) {
      continue;
    }

    const auto num_components = count_components(components.sizes);
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

  // TODO(nathan) should handle everything in descending order instead
  return Filtration(filtration.begin(), filtration.end());
}

std::pair<size_t, size_t> getTrimmedFiltration(const Filtration& filtration,
                                               double min_dilation_m,
                                               double max_dilation_m) {
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
  if (max_index < end_index) {
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

}  // namespace hydra
