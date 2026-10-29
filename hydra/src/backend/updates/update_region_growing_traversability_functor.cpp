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
#include "hydra/backend/updates/update_region_growing_traversability_functor.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/validation.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/scene_graph.h>
#include <spark_dsg/traversability_boundary.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <numbers>

#include "hydra/utils/timing_utilities.h"

using namespace spark_dsg;

namespace hydra {

using Timer = timing::ScopedTimer;
using spark_dsg::TravNodeAttributes;

void declare_config(UpdateRegionGrowingTraversabilityFunctor::Config& config) {
  using namespace config;
  name("UpdateRegionGrowingTraversabilityFunctor::Config");
  base<VerbosityConfig>(config);
  field(config.layer, "layer");
  field(config.deformation, "deformation");
  field(config.pairwise_overlap, "pairwise_overlap");
  field(config.union_overlap, "union_overlap");
  field(config.overlap_samples, "overlap_samples");
  check(config.pairwise_overlap, GE, 0.0, "pairwise_overlap");
  check(config.union_overlap, GE, 0.0, "union_overlap");
  check(config.overlap_samples, GT, size_t(0), "overlap_samples");
}

namespace {

// Estimate the fraction of the area of 'attrs' that is covered by the union of 'others'
// by sampling the star-shaped boundary in polar coordinates.
double coveredFraction(const TravNodeAttributes& attrs,
                       const std::vector<const TravNodeAttributes*>& others,
                       size_t radial_samples) {
  const size_t N = attrs.radii.size();
  if (N == 0 || others.empty()) {
    return 0.0;
  }

  double total = 0.0;
  double covered = 0.0;
  for (size_t i = 0; i < N; ++i) {
    const double angle = 2.0 * std::numbers::pi * (static_cast<double>(i) + 0.5) /
                         static_cast<double>(N);
    const double radius = 0.5 * (attrs.radii[i] + attrs.radii[(i + 1) % N]);
    const Eigen::Vector3d dir(std::cos(angle), std::sin(angle), 0.0);
    for (size_t k = 0; k < radial_samples; ++k) {
      // Weight by the polar area element r * dr.
      const double r =
          (static_cast<double>(k) + 0.5) / static_cast<double>(radial_samples) * radius;
      const Eigen::Vector3d point = attrs.position + r * dir;
      total += r;
      for (const auto other : others) {
        if (other->contains(point)) {
          covered += r;
          break;
        }
      }
    }
  }
  return total > 0.0 ? covered / total : 0.0;
}

}  // namespace

static const auto registration =
    config::RegistrationWithConfig<UpdateFunctor,
                                   UpdateRegionGrowingTraversabilityFunctor,
                                   UpdateRegionGrowingTraversabilityFunctor::Config>(
        "UpdateRegionGrowingTraversabilityFunctor");

UpdateRegionGrowingTraversabilityFunctor::UpdateRegionGrowingTraversabilityFunctor(
    const Config& config)
    : config(config::checkValid(config)),
      deformation_interpolator_(config.deformation) {}

UpdateFunctor::Hooks UpdateRegionGrowingTraversabilityFunctor::hooks() const {
  auto my_hooks = UpdateFunctor::hooks();
  my_hooks.find_merges = [this](const SceneGraph& dsg,
                                const UpdateInfo::ConstPtr& info) {
    return findNodeMerges(dsg, info);
  };
  my_hooks.merge = [this](const SceneGraph& dsg, const std::vector<NodeId>& merge_ids) {
    return mergeNodes(dsg, merge_ids);
  };
  my_hooks.cleanup = [this](const UpdateInfo::ConstPtr& info,
                            SceneGraph&,
                            SharedDsgInfo* dsg) { cleanup(info, dsg); };
  return my_hooks;
}

void UpdateRegionGrowingTraversabilityFunctor::call(
    const SceneGraph& unmerged,
    SharedDsgInfo& dsg,
    const UpdateInfo::ConstPtr& info) const {
  Timer timer("backend/update_traversability", info->timestamp_ns);
  if (!unmerged.hasLayer(config.layer)) {
    return;
  }

  // Update global poses (deformation) of all nodes.
  updateDeformation(unmerged, dsg, info);

  // In the case of loop closures, reset all added edges.
  merge_candidates_.clear();
  if (info->full_update) {
    resetAddedEdges(*dsg.graph);
    findInactiveEdges(*dsg.graph);
  }

  // Find and update all edges from active to inactive nodes.
  findActiveWindowEdges(*dsg.graph);

  // Remove active window edges that no longer overlap and archive ones that are now
  // inactive.
  if (info->full_update) {
    return;
  }
  pruneActiveWindowEdges(*dsg.graph);
}

void UpdateRegionGrowingTraversabilityFunctor::updateDeformation(
    const SceneGraph& unmerged,
    SharedDsgInfo& dsg,
    const UpdateInfo::ConstPtr& info) const {
  // Update global poses (deformation) of all nodes.
  const auto& places = unmerged.getLayer(config.layer);
  const auto view = info->full_update ? LayerView(places) : activeNodes(places);
  deformation_interpolator_.interpolateNodePositions(unmerged, *dsg.graph, info, view);
}

void UpdateRegionGrowingTraversabilityFunctor::resetAddedEdges(SceneGraph& dsg) const {
  EdgeSet to_remove;
  for (const auto& edge : dsg.getLayer(config.layer).edges()) {
    if (edge.attributes<EdgeAttributes>().weight < 0.0) {
      to_remove.insert(edge.key());
    }
  }
  for (const auto& edge_key : to_remove) {
    dsg.removeEdge(edge_key.k1, edge_key.k2);
  }
  active_edges_.clear();
}

void UpdateRegionGrowingTraversabilityFunctor::findInactiveEdges(
    SceneGraph& dsg) const {
  EdgeSet visited;
  for (const auto& node : dsg.getLayer(config.layer).nodes()) {
    const auto& from_attrs = node.attributes<TravNodeAttributes>();
    if (from_attrs.is_active) {
      continue;
    }

    // Find all overlapping inactive nodes.
    for (const auto to_id : findConnections(dsg, from_attrs)) {
      const EdgeKey edge_key(node.id, to_id);
      if (visited.count(edge_key)) {
        continue;
      }
      const auto& to_attrs = dsg.getNode(to_id).attributes<TravNodeAttributes>();
      if (to_attrs.is_active) {
        continue;
      }

      visited.insert(edge_key);
      if (from_attrs.intersects(to_attrs)) {
        // NOTE(lschmid): Weight of -2 indicates this is an inactive overlap edge.
        dsg.addOrUpdateEdge(node.id, to_id, std::make_unique<EdgeAttributes>(-2.0));
        merge_candidates_.insert(edge_key);
      }
    }
  }
}

void UpdateRegionGrowingTraversabilityFunctor::findActiveWindowEdges(
    SceneGraph& dsg) const {
  active_edges_.clear();
  const auto& layer = dsg.getLayer(config.layer);
  for (const auto& node : activeNodes(layer)) {
    const auto& from_attrs = node.attributes<TravNodeAttributes>();
    for (const auto to_id : findConnections(dsg, from_attrs)) {
      // NOTE(lschmid): Weight of -1 indicates this is an AW edge.
      dsg.addOrUpdateEdge(node.id, to_id, std::make_unique<EdgeAttributes>(-1.0));
      active_edges_.insert(EdgeKey(node.id, to_id));
    }
  }
}

void UpdateRegionGrowingTraversabilityFunctor::pruneActiveWindowEdges(
    SceneGraph& dsg) const {
  EdgeSet to_remove;
  for (const auto& edge : dsg.getLayer(config.layer).edges()) {
    const auto key = edge.key();
    if (active_edges_.count(key) || edge.attributes().weight != -1.0) {
      continue;
    }
    // Previously active edges to revisit
    const auto& attrs_1 = dsg.getNode(key.k1).attributes<TravNodeAttributes>();
    const auto& attrs_2 = dsg.getNode(key.k2).attributes<TravNodeAttributes>();
    if (!attrs_1.intersects(attrs_2)) {
      to_remove.insert(key);
      continue;
    }

    if (!attrs_1.is_active && !attrs_2.is_active) {
      // Move to inactive edges.
      dsg.getEdge(key.k1, key.k2).attributes().weight = -2.0;
      merge_candidates_.insert(key);
    }
  }

  for (const auto& edge_key : to_remove) {
    dsg.removeEdge(edge_key.k1, edge_key.k2);
  }
}

MergeList UpdateRegionGrowingTraversabilityFunctor::findNodeMerges(
    const SceneGraph& dsg, const UpdateInfo::ConstPtr& /* info */) const {
  // TODO(lschmid): Consider an incremental version in the future.
  MergeList result;
  std::set<NodeId> merged;
  std::map<NodeId, std::vector<NodeId>> neighbors;

  // Candidates are all inactive connections, as these already overlap.
  for (const auto& edge_key : merge_candidates_) {
    // Nodes may have been merged away in previous (exhaustive) passes.
    if (!dsg.hasNode(edge_key.k1) || !dsg.hasNode(edge_key.k2)) {
      continue;
    }
    neighbors[edge_key.k1].push_back(edge_key.k2);
    neighbors[edge_key.k2].push_back(edge_key.k1);
    if (merged.count(edge_key.k1) || merged.count(edge_key.k2)) {
      continue;
    }

    const auto& from_attrs = dsg.getNode(edge_key.k1).attributes<TravNodeAttributes>();
    const auto& to_attrs = dsg.getNode(edge_key.k2).attributes<TravNodeAttributes>();

    // Merge if either centroid is included in the other or either place is largely
    // covered by the other. The more covered place is merged into the other one.
    const bool from_included = from_attrs.contains(to_attrs.position);
    const bool to_included = to_attrs.contains(from_attrs.position);
    const double from_covered =
        coveredFraction(from_attrs, {&to_attrs}, config.overlap_samples);
    const double to_covered =
        coveredFraction(to_attrs, {&from_attrs}, config.overlap_samples);
    if (!to_included && !from_included &&
        std::max(from_covered, to_covered) < config.pairwise_overlap) {
      continue;
    }

    if (from_covered > to_covered) {
      // Merge from -> to
      result.push_back({edge_key.k1, edge_key.k2});
      merged.insert(edge_key.k1);
    } else {
      result.push_back({edge_key.k2, edge_key.k1});
      merged.insert(edge_key.k2);
    }
  }

  // Merge places that are covered by the union of their neighbors, e.g. when revisiting
  // previously mapped areas. Check newest places first, as these are the redundant
  // ones.
  std::vector<NodeId> to_check;
  for (const auto& [node_id, node_neighbors] : neighbors) {
    if (node_neighbors.size() > 1) {
      to_check.push_back(node_id);
    }
  }
  std::sort(to_check.begin(), to_check.end(), [&dsg](NodeId lhs, NodeId rhs) {
    return dsg.getNode(lhs).attributes<TravNodeAttributes>().first_observed_ns >
           dsg.getNode(rhs).attributes<TravNodeAttributes>().first_observed_ns;
  });

  for (const auto node_id : to_check) {
    if (merged.count(node_id)) {
      continue;
    }

    std::vector<const TravNodeAttributes*> others;
    NodeId best_id = node_id;
    double best_covered = 0.0;
    const auto& node_attrs = dsg.getNode(node_id).attributes<TravNodeAttributes>();
    for (const auto neighbor_id : neighbors.at(node_id)) {
      if (merged.count(neighbor_id)) {
        continue;
      }
      const auto& neighbor_attrs =
          dsg.getNode(neighbor_id).attributes<TravNodeAttributes>();
      others.push_back(&neighbor_attrs);
      const double covered =
          coveredFraction(node_attrs, {&neighbor_attrs}, config.overlap_samples);
      if (covered > best_covered) {
        best_covered = covered;
        best_id = neighbor_id;
      }
    }

    if (best_id == node_id ||
        coveredFraction(node_attrs, others, config.overlap_samples) <
            config.union_overlap) {
      continue;
    }
    result.push_back({node_id, best_id});
    merged.insert(node_id);
  }
  return result;
}

NodeAttributes::Ptr UpdateRegionGrowingTraversabilityFunctor::mergeNodes(
    const SceneGraph& dsg, const std::vector<NodeId>& merge_ids) const {
  auto result = dsg.getNode(merge_ids.front()).attributes().clone();
  return result;
}

void UpdateRegionGrowingTraversabilityFunctor::cleanup(const UpdateInfo::ConstPtr&,
                                                       SharedDsgInfo*) const {}

std::vector<NodeId> UpdateRegionGrowingTraversabilityFunctor::findConnections(
    const SceneGraph& dsg, const TravNodeAttributes& from_attrs) const {
  std::vector<NodeId> connections;
  // NOTE(lschmid): Radius search doesn't work right, brute force for now.
  for (const auto& to_node : dsg.getLayer(config.layer).nodes()) {
    const auto& to_attrs = to_node.attributes<TravNodeAttributes>();
    if (hasActiveOverlap(from_attrs, to_attrs)) {
      continue;
    }
    if (from_attrs.intersects(to_attrs)) {
      connections.emplace_back(to_node.id);
    }
  }
  return connections;
}

bool UpdateRegionGrowingTraversabilityFunctor::hasActiveOverlap(
    const TravNodeAttributes& attrs1, const TravNodeAttributes& attrs2) {
  // TODO(lschmid): Double check this is correct.
  if (attrs1.last_observed_ns < attrs2.first_observed_ns ||
      attrs1.first_observed_ns > attrs2.last_observed_ns) {
    return false;
  }
  return true;
}

LayerView UpdateRegionGrowingTraversabilityFunctor::activeNodes(
    const SceneGraphLayer& layer) {
  return LayerView(
      layer, [](const SceneGraphNode& node) { return node.attributes().is_active; });
}

}  // namespace hydra
