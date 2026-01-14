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
#include "hydra/backend/update_region_growing_traversability_functor.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/traversability_boundary.h>

#include <queue>

#include "hydra/common/global_info.h"
#include "hydra/utils/nearest_neighbor_utilities.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using Timer = timing::ScopedTimer;
using spark_dsg::TraversabilityNodeAttributes;

void declare_config(UpdateRegionGrowingTraversabilityFunctor::Config& config) {
  using namespace config;
  name("UpdateRegionGrowingTraversabilityFunctor::Config");
  field(config.layer, "layer");
  field(config.deformation, "deformation");
}

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
  my_hooks.find_merges = [this](const DynamicSceneGraph& dsg,
                                const UpdateInfo::ConstPtr& info) {
    return findNodeMerges(dsg, info);
  };
  my_hooks.merge = [this](const DynamicSceneGraph& dsg,
                          const std::vector<NodeId>& merge_ids) {
    return mergeNodes(dsg, merge_ids);
  };
  my_hooks.cleanup = [this](const UpdateInfo::ConstPtr& info, SharedDsgInfo* dsg) {
    cleanup(info, dsg);
  };
  return my_hooks;
}

void UpdateRegionGrowingTraversabilityFunctor::call(
    const DynamicSceneGraph& unmerged,
    SharedDsgInfo& dsg,
    const UpdateInfo::ConstPtr& info) const {
  Timer timer("backend/update_traversability", info->timestamp_ns);
  if (!unmerged.hasLayer(config.layer)) {
    return;
  }
  // Cache the max radius in the current layer for search.
  radius_ = 0.0f;
  const auto& layer = unmerged.getLayer(config.layer);
  for (const auto& [id, node] : layer.nodes()) {
    const auto& attrs = node->attributes<TraversabilityNodeAttributes>();
    radius_ = std::max(radius_, static_cast<float>(attrs.boundary.min.x()));
  }
  radius_ *= 2.0f;

  // Setup and state tracking.
  active_tracker_.clear();  // reset from previous pass

  // Update global poses (deformation) of all nodes.
  updateDeformation(unmerged, dsg, info);
  resetNeighborFinder(*dsg.graph);

  // Find and update all edges from active to inactive nodes.
  const auto active_edges = findActiveWindowEdges(*dsg.graph);
  pruneActiveWindowEdges(*dsg.graph, active_edges);
  previous_active_edges_ = std::move(active_edges);
}

void UpdateRegionGrowingTraversabilityFunctor::updateDeformation(
    const DynamicSceneGraph& unmerged,
    SharedDsgInfo& dsg,
    const UpdateInfo::ConstPtr& info) const {
  // Update global poses (deformation) of all nodes.
  const auto& places = unmerged.getLayer(config.layer);
  const auto view =
      info->loop_closure_detected ? LayerView(places) : active_tracker_.view(places);
  deformation_interpolator_.interpolateNodePositions(unmerged, *dsg.graph, info, view);
}

UpdateRegionGrowingTraversabilityFunctor::EdgeSet
UpdateRegionGrowingTraversabilityFunctor::findActiveWindowEdges(
    DynamicSceneGraph& dsg) const {
  if (!nn_) {
    return {};
  }

  // Compare all active against inactive candidate nodes.
  EdgeSet active_edges;
  const auto view = active_tracker_.view(dsg.getLayer(config.layer), true);
  for (const auto& node : view) {
    const auto& from_attrs = node.attributes<TraversabilityNodeAttributes>();
    if (!from_attrs.is_active) {
      continue;
    }
    for (const auto to_id : findConnections(dsg, from_attrs)) {
      // NOTE(lschmid): Weight of 0.0 indicates this is an AW edge.
      dsg.addOrUpdateEdge(node.id, to_id, std::make_unique<EdgeAttributes>(0.0));
      active_edges.insert(EdgeKey(node.id, to_id));
    }
  }
  return active_edges;
}

void UpdateRegionGrowingTraversabilityFunctor::pruneActiveWindowEdges(
    DynamicSceneGraph& dsg, const EdgeSet& active_edges) const {
  for (const auto& edge_key : previous_active_edges_) {
    if (active_edges.count(edge_key) || !dsg.hasEdge(edge_key.k1, edge_key.k2)) {
      continue;
    }
    const auto& attrs_1 =
        dsg.getNode(edge_key.k1).attributes<TraversabilityNodeAttributes>();
    const auto& attrs_2 =
        dsg.getNode(edge_key.k2).attributes<TraversabilityNodeAttributes>();
    if (!(attrs_1.is_active || attrs_2.is_active) ||
        !hasTraversableOverlap(attrs_1, attrs_2)) {
      dsg.removeEdge(edge_key.k1, edge_key.k2);
    }
  }
}

MergeList UpdateRegionGrowingTraversabilityFunctor::findNodeMerges(
    const DynamicSceneGraph& dsg, const UpdateInfo::ConstPtr& info) const {
  // Iteratively match all newly archived nodes against inactive candidate nodes.
  resetNeighborFinder(dsg);
  if (!nn_) {
    return {};
  }

  const auto& places = dsg.getLayer(config.layer);
  const auto view = info->loop_closure_detected ? LayerView(places)
                                                : active_tracker_.view(places, true);

  // Compare all (newly) archived nodes against inactive candidate nodes.
  MergeList result;
  std::set<NodeId> merged;
  for (const auto& from_node : view) {
    if (merged.count(from_node.id)) {
      continue;
    }

    auto& from_attrs = from_node.attributes<TraversabilityNodeAttributes>();
    if (from_attrs.is_active) {
      continue;
    }

    for (const auto to_id : nn_->findRadius(from_attrs.position, radius_, false)) {
      if (to_id == from_node.id || merged.count(to_id)) {
        continue;
      }
      auto& to_attrs = dsg.getNode(to_id).attributes<TraversabilityNodeAttributes>();
      if (to_attrs.is_active) {
        continue;
      }

      // Avoid merging nodes with active window (temporal) overlap.
      if (from_attrs.last_observed_ns < to_attrs.first_observed_ns ||
          from_attrs.first_observed_ns > to_attrs.last_observed_ns) {
        continue;
      }

      // Check boundaries.
      const float distance = (from_attrs.position - to_attrs.position).norm();
      const bool to_included = distance <= to_attrs.boundary.min.x();
      const bool from_included = distance <= from_attrs.boundary.min.x();

      if (!to_included && !from_included) {
        continue;
      }

      if (!from_included || to_attrs.boundary.min.x() > from_attrs.boundary.min.x()) {
        result.push_back({from_node.id, to_id});
        merged.insert(from_node.id);
        continue;
      }
      result.push_back({to_id, from_node.id});
      merged.insert(to_id);
    }
  }
  return result;
}

NodeAttributes::Ptr UpdateRegionGrowingTraversabilityFunctor::mergeNodes(
    const DynamicSceneGraph& dsg, const std::vector<NodeId>& merge_ids) const {
  auto result = dsg.getNode(merge_ids.front()).attributes().clone();
  return result;
}

void UpdateRegionGrowingTraversabilityFunctor::cleanup(const UpdateInfo::ConstPtr&,
                                                       SharedDsgInfo*) const {}

std::vector<NodeId> UpdateRegionGrowingTraversabilityFunctor::findConnections(
    const DynamicSceneGraph& dsg,
    const TraversabilityNodeAttributes& from_attrs) const {
  if (!nn_) {
    return {};
  }
  std::vector<NodeId> connections;
  for (const auto to_id : nn_->findRadius(from_attrs.position, radius_, false)) {
    const auto& to_attrs =
        dsg.getNode(to_id).attributes<TraversabilityNodeAttributes>();

    if (hasTraversableOverlap(from_attrs, to_attrs)) {
      connections.emplace_back(to_id);
    }
  }
  return connections;
}

bool UpdateRegionGrowingTraversabilityFunctor::hasTraversableOverlap(
    const TraversabilityNodeAttributes& from,
    const TraversabilityNodeAttributes& to) const {
  // Simple first implementation: Check the intersecting area meets the minimum place
  // size.
  const float distance = (from.position - to.position).norm();
  return distance < (from.boundary.min.x() + to.boundary.min.x());
}

void UpdateRegionGrowingTraversabilityFunctor::resetNeighborFinder(
    const DynamicSceneGraph& dsg) const {
  nn_ = NearestNodeFinder::fromLayer(
      dsg.getLayer(config.layer),
      [](const SceneGraphNode& node) { return !node.attributes().is_active; });
}

}  // namespace hydra
