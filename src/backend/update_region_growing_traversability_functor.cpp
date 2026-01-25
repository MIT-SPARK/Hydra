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
using spark_dsg::TravNodeAttributes;

void declare_config(UpdateRegionGrowingTraversabilityFunctor::Config& config) {
  using namespace config;
  name("UpdateRegionGrowingTraversabilityFunctor::Config");
  base<VerbosityConfig>(config);
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

  // Update global poses (deformation) of all nodes.
  updateDeformation(unmerged, dsg, info);

  // In the case of loop closures, reset all added edges.
  merge_candidates_.clear();
  if (info->loop_closure_detected) {
    resetAddedEdges(*dsg.graph);
    findInactiveEdges(*dsg.graph);
  }

  // Find and update all edges from active to inactive nodes.
  findActiveWindowEdges(*dsg.graph);

  // Remove active window edges that no longer overlap and archive ones that are now
  // inactive.
  if (info->loop_closure_detected) {
    return;
  }
  pruneActiveWindowEdges(*dsg.graph);
}

void UpdateRegionGrowingTraversabilityFunctor::updateDeformation(
    const DynamicSceneGraph& unmerged,
    SharedDsgInfo& dsg,
    const UpdateInfo::ConstPtr& info) const {
  // Update global poses (deformation) of all nodes.
  const auto& places = unmerged.getLayer(config.layer);
  const auto view =
      info->loop_closure_detected ? LayerView(places) : activeNodes(places);
  deformation_interpolator_.interpolateNodePositions(unmerged, *dsg.graph, info, view);
}

void UpdateRegionGrowingTraversabilityFunctor::resetAddedEdges(
    DynamicSceneGraph& dsg) const {
  EdgeSet to_remove;
  for (const auto& [key, edge] : dsg.getLayer(config.layer).edges()) {
    if (edge.attributes<EdgeAttributes>().weight < 0.0) {
      to_remove.insert(key);
    }
  }
  for (const auto& edge_key : to_remove) {
    dsg.removeEdge(edge_key.k1, edge_key.k2);
  }
  active_edges_.clear();
}

void UpdateRegionGrowingTraversabilityFunctor::findInactiveEdges(
    DynamicSceneGraph& dsg) const {
  EdgeSet visited;
  for (const auto& [from_id, node] : dsg.getLayer(config.layer).nodes()) {
    const auto& from_attrs = node->attributes<TravNodeAttributes>();
    if (from_attrs.is_active) {
      continue;
    }

    // Find all overlapping inactive nodes.
    for (const auto to_id : findConnections(dsg, from_attrs)) {
      const EdgeKey edge_key(from_id, to_id);
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
        dsg.addOrUpdateEdge(from_id, to_id, std::make_unique<EdgeAttributes>(-2.0));
        merge_candidates_.insert(edge_key);
      }
    }
  }
}

void UpdateRegionGrowingTraversabilityFunctor::findActiveWindowEdges(
    DynamicSceneGraph& dsg) const {
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
    DynamicSceneGraph& dsg) const {
  EdgeSet to_remove;
  for (const auto& [edge_key, edge] : dsg.getLayer(config.layer).edges()) {
    if (active_edges_.count(edge_key) || edge.attributes().weight != -1.0) {
      continue;
    }
    // Previously active edges to revisit
    const auto& attrs_1 = dsg.getNode(edge_key.k1).attributes<TravNodeAttributes>();
    const auto& attrs_2 = dsg.getNode(edge_key.k2).attributes<TravNodeAttributes>();
    if (!attrs_1.intersects(attrs_2)) {
      to_remove.insert(edge_key);
      continue;
    }

    if (!attrs_1.is_active && !attrs_2.is_active) {
      // Move to inactive edges.
      dsg.getEdge(edge_key.k1, edge_key.k2).attributes().weight = -2.0;
      merge_candidates_.insert(edge_key);
    }
  }

  for (const auto& edge_key : to_remove) {
    dsg.removeEdge(edge_key.k1, edge_key.k2);
  }
}

MergeList UpdateRegionGrowingTraversabilityFunctor::findNodeMerges(
    const DynamicSceneGraph& dsg, const UpdateInfo::ConstPtr& /* info */) const {
  // TODO(lschmid): Consider an incremental version in the future.
  MergeList result;
  std::set<NodeId> merged;
  // Candidates are all inactive connections, as these already overlap.
  for (const auto& edge_key : merge_candidates_) {
    if (merged.count(edge_key.k1) || merged.count(edge_key.k2)) {
      continue;
    }

    const auto& from_attrs = dsg.getNode(edge_key.k1).attributes<TravNodeAttributes>();
    const auto& to_attrs = dsg.getNode(edge_key.k2).attributes<TravNodeAttributes>();

    // Check boundaries. Merge if the centroids are included in the other's radius. If
    // both are included, keep the larger one.
    const bool from_included = from_attrs.contains(to_attrs.position);
    const bool to_included = to_attrs.contains(from_attrs.position);
    if (!to_included && !from_included) {
      continue;
    }

    if (!from_included || to_attrs.area() > from_attrs.area()) {
      // Merge from -> to
      result.push_back({edge_key.k1, edge_key.k2});
      merged.insert(edge_key.k1);
    } else {
      result.push_back({edge_key.k2, edge_key.k1});
      merged.insert(edge_key.k2);
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
    const DynamicSceneGraph& dsg, const TravNodeAttributes& from_attrs) const {
  std::vector<NodeId> connections;
  // NOTE(lschmid): Radius search doesn't work right, brute force for now.
  for (const auto& [to_id, to_node] : dsg.getLayer(config.layer).nodes()) {
    const auto& to_attrs = to_node->attributes<TravNodeAttributes>();
    if (hasActiveOverlap(from_attrs, to_attrs)) {
      continue;
    }
    if (from_attrs.intersects(to_attrs)) {
      connections.emplace_back(to_id);
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
