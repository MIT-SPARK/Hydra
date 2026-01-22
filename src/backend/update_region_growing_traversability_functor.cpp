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
  // Cache the max radius in the current layer for search.
  radius_ = 0.0f;
  const auto& layer = unmerged.getLayer(config.layer);
  for (const auto& [id, node] : layer.nodes()) {
    const auto& attrs = node->attributes<TravNodeAttributes>();
    radius_ = std::max(radius_, static_cast<float>(attrs.max_radius));
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

  // Maintain and update edges between overlapping inactive nodes.
  updateInactiveEdges(*dsg.graph);
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
  std::stringstream info;

  // Compare all active against inactive candidate nodes.
  EdgeSet active_edges;

  // TMP(lschmid): Just iterate over the entire graph until the AW tracker is fixed.
  // const auto view = active_tracker_.view(dsg.getLayer(config.layer), true);
  const auto& layer = dsg.getLayer(config.layer);
  for (const auto& [from_id, node] : layer.nodes()) {
    const auto& from_attrs = node->attributes<TravNodeAttributes>();
    if (!from_attrs.is_active) {
      continue;
    }
    info << "\nActive " << NodeSymbol(from_id) << ": [";
    for (const auto to_id : findConnections(dsg, from_attrs)) {
      // NOTE(lschmid): Weight of -1 indicates this is an AW edge.
      dsg.addOrUpdateEdge(from_id, to_id, std::make_unique<EdgeAttributes>(-1.0));
      active_edges.insert(EdgeKey(from_id, to_id));
      info << NodeSymbol(to_id) << ", ";
    }
    info << "]";
  }
  LOG(INFO) << "Active window edges:" << info.str();
  return active_edges;
}

void UpdateRegionGrowingTraversabilityFunctor::pruneActiveWindowEdges(
    DynamicSceneGraph& dsg, const EdgeSet& active_edges) const {
  for (const auto& edge_key : previous_active_edges_) {
    if (active_edges.count(edge_key) || !dsg.hasEdge(edge_key.k1, edge_key.k2)) {
      continue;
    }
    const auto& attrs_1 = dsg.getNode(edge_key.k1).attributes<TravNodeAttributes>();
    const auto& attrs_2 = dsg.getNode(edge_key.k2).attributes<TravNodeAttributes>();
    if ((!attrs_1.is_active && !attrs_2.is_active) || !attrs_1.intersects(attrs_2)) {
      dsg.removeEdge(edge_key.k1, edge_key.k2);
    }
  }
}

void UpdateRegionGrowingTraversabilityFunctor::updateInactiveEdges(
    DynamicSceneGraph& dsg) const {
  // TODO(lschmid): Consider an incremental version in the future.
  EdgeSet visited;
  for (const auto& [from_id, node] : dsg.getLayer(config.layer).nodes()) {
    const auto& from_attrs = node->attributes<TravNodeAttributes>();
    if (from_attrs.is_active) {
      continue;
    }

    // Find all overlapping inactive nodes.
    for (const auto to_id : findConnections(dsg, from_attrs)) {
      const EdgeKey edge_key(from_id, to_id);
      if (from_id == to_id || visited.count(edge_key)) {
        continue;
      }
      const auto& to_attrs = dsg.getNode(to_id).attributes<TravNodeAttributes>();
      if (to_attrs.is_active) {
        continue;
      }

      visited.insert(edge_key);
      const bool should_connect = from_attrs.intersects(to_attrs);
      if (should_connect) {
        // NOTE(lschmid): Weight of -2 indicates this is an inactive overlap edge.
        dsg.addOrUpdateEdge(from_id, to_id, std::make_unique<EdgeAttributes>(-2.0));
      } else {
        dsg.removeEdge(from_id, to_id);
      }
    }
  }
}

MergeList UpdateRegionGrowingTraversabilityFunctor::findNodeMerges(
    const DynamicSceneGraph& dsg, const UpdateInfo::ConstPtr& /* info */) const {
  // TODO(lschmid): Consider an incremental version in the future.
  MergeList result;
  std::set<NodeId> merged;
  // Candidates are all inactive connections, as these already overlap.
  for (const auto& [key, edge] : dsg.getLayer(config.layer).edges()) {
    if (edge.attributes<EdgeAttributes>().weight != -2.0 || merged.count(key.k1) ||
        merged.count(key.k2)) {
      continue;
    }

    const auto& from_attrs = dsg.getNode(key.k1).attributes<TravNodeAttributes>();
    const auto& to_attrs = dsg.getNode(key.k2).attributes<TravNodeAttributes>();

    // Check boundaries. Merge if the centroids are included in the other's radius. If
    // both are included, keep the larger one.
    const bool from_included = from_attrs.contains(to_attrs.position);
    const bool to_included = to_attrs.contains(from_attrs.position);

    if (!to_included && !from_included) {
      continue;
    }

    if (!from_included || to_attrs.area() > from_attrs.area()) {
      result.push_back({key.k1, key.k2});
      merged.insert(key.k1);
    } else {
      result.push_back({key.k2, key.k1});
      merged.insert(key.k2);
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
  for (const auto to_id : nn_->findRadius(from_attrs.position, radius_, true)) {
    const auto& to_attrs = dsg.getNode(to_id).attributes<TravNodeAttributes>();
    if (hasActiveOverlap(from_attrs, to_attrs)) {
      continue;
    }
    if (from_attrs.intersects(to_attrs)) {
      connections.emplace_back(to_id);
    }
  }
  return connections;
}

void UpdateRegionGrowingTraversabilityFunctor::resetNeighborFinder(
    const DynamicSceneGraph& dsg) const {
  nn_ = NearestNodeFinder::fromLayer(
      dsg.getLayer(config.layer),
      [](const SceneGraphNode& node) { return !node.attributes().is_active; });
}

bool UpdateRegionGrowingTraversabilityFunctor::hasActiveOverlap(
    const TravNodeAttributes& attrs1, const TravNodeAttributes& attrs2) const {
  // TODO(lschmid): Double check this is correct.
  if (attrs1.last_observed_ns < attrs2.first_observed_ns ||
      attrs1.first_observed_ns > attrs2.last_observed_ns) {
    return false;
  }
  return true;
}

}  // namespace hydra
