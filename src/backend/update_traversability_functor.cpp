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
#include "hydra/backend/update_traversability_functor.h"

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

void declare_config(UpdateTraversabilityFunctor::Config& config) {
  using namespace config;
  name("UpdateTraversabilityFunctor::Config");
  field(config.layer, "layer");
  field(config.min_place_size, "min_place_size", "m");
  field(config.max_place_size, "max_place_size", "m");
  field(config.tolerance, "tolerance", "m");
  field(config.use_metric_distance, "use_metric_distance");
  field(config.deformation, "deformation");

  check(config.min_place_size, GT, 0.0, "min_place_size");
  check(config.max_place_size, GT, 0.0, "max_place_size");
  check(config.tolerance, GE, 0.0, "min_place_size");
}

static const auto registration =
    config::RegistrationWithConfig<UpdateFunctor,
                                   UpdateTraversabilityFunctor,
                                   UpdateTraversabilityFunctor::Config>(
        "UpdateTraversabilityFunctor");

UpdateTraversabilityFunctor::UpdateTraversabilityFunctor(const Config& config)
    : config(config::checkValid(config)),
      radius_(2 * config.max_place_size),
      min_connectivity_(config.min_place_size - config.tolerance),
      deformation_interpolator_(config.deformation) {}

UpdateFunctor::Hooks UpdateTraversabilityFunctor::hooks() const {
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

void UpdateTraversabilityFunctor::call(const DynamicSceneGraph& unmerged,
                                       SharedDsgInfo& dsg,
                                       const UpdateInfo::ConstPtr& info) const {
  Timer timer("backend/update_traversability", info->timestamp_ns);
  if (!unmerged.hasLayer(config.layer)) {
    return;
  }

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

void UpdateTraversabilityFunctor::updateDeformation(
    const DynamicSceneGraph& unmerged,
    SharedDsgInfo& dsg,
    const UpdateInfo::ConstPtr& info) const {
  // Update global poses (deformation) of all nodes.
  const auto& places = unmerged.getLayer(config.layer);
  const auto view =
      info->loop_closure_detected ? LayerView(places) : active_tracker_.view(places);
  deformation_interpolator_.interpolateNodePositions(unmerged, *dsg.graph, info, view);
}

UpdateTraversabilityFunctor::EdgeSet UpdateTraversabilityFunctor::findActiveWindowEdges(
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
      // TODO(lschmid): Weight of 0.0 indicates this is an AW edge.
      dsg.addOrUpdateEdge(node.id, to_id, std::make_unique<EdgeAttributes>(0.0));
      active_edges.insert(EdgeKey(node.id, to_id));
    }
  }
  return active_edges;
}

void UpdateTraversabilityFunctor::pruneActiveWindowEdges(
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

void UpdateTraversabilityFunctor::updateDistances(const SceneGraphLayer& layer) const {
  // Simple initial solution: Recompute distances for all places. This is not pretty but
  // should be ok for a first test.
  if (config.use_metric_distance) {
    for (const auto& [id, node] : layer.nodes()) {
      auto& attrs = node->attributes<TraversabilityNodeAttributes>();
      NodeSet visited;
      attrs.distance =
          computeMetricDistance(layer, attrs.position.head<2>(), {id}, visited);
    }
  } else {
    computeTopologicalDistances(layer);
  }
}

MergeList UpdateTraversabilityFunctor::findNodeMerges(
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

    auto from_boundary = Boundary(from_attrs);
    for (const auto to_id : nn_->findRadius(from_attrs.position, radius_, false)) {
      if (to_id == from_node.id || merged.count(to_id)) {
        continue;
      }
      auto& to_attrs = dsg.getNode(to_id).attributes<TraversabilityNodeAttributes>();
      // Avoid merging nodes with active window (temporal) overlap.
      if (from_attrs.last_observed_ns >= to_attrs.first_observed_ns &&
          from_attrs.first_observed_ns <= to_attrs.last_observed_ns) {
        continue;
      }

      // Check boundaries.
      auto to_boundary = Boundary(to_attrs);
      if (!to_boundary.intersects(from_boundary)) {
        continue;
      }

      // Simple case: If places are completely contained.
      if (isContained(to_boundary, from_boundary)) {
        result.push_back({to_id, from_node.id});
        merged.insert(to_id);
        continue;
      }
      if (isContained(from_boundary, to_boundary)) {
        result.push_back({from_node.id, to_id});
        merged.insert(from_node.id);
        continue;
      }

      // Large overlap: No need to keep both, as transition is always possible between.
      auto intersection = from_boundary.intersection(to_boundary);
      if (intersection.height() < config.min_place_size ||
          intersection.width() < config.min_place_size) {
        // TODO(lschmid): Can consider cropping and dropping in the future.
        // Always update the classification of the boundary sides.
        overlapping_nodes_to_cleanup_.insert({from_node.id, to_id});
        continue;
      }

      // Aligned in X or Y direction: Merge by fusing boundary.
      if ((std::abs(from_boundary.min.x() - to_boundary.min.x()) < config.tolerance &&
           std::abs(from_boundary.max.x() - to_boundary.max.x()) < config.tolerance) ||
          (std::abs(from_boundary.min.y() - to_boundary.min.y()) < config.tolerance &&
           std::abs(from_boundary.max.y() - to_boundary.max.y()) < config.tolerance)) {
        merged.insert(to_id);
        result.push_back({to_id, from_node.id});
        continue;
      }

      // If not fuseable easily keep larger.
      if (from_boundary.area() >= to_boundary.area()) {
        result.push_back({to_id, from_node.id});
        merged.insert(to_id);
      } else {
        result.push_back({from_node.id, to_id});
        merged.insert(from_node.id);
      }
    }
  }

  return result;
}

NodeAttributes::Ptr UpdateTraversabilityFunctor::mergeNodes(
    const DynamicSceneGraph& dsg, const std::vector<NodeId>& merge_ids) const {
  auto result = dsg.getNode(merge_ids.front()).attributes().clone();
  auto& to_attrs = dynamic_cast<TraversabilityNodeAttributes&>(*result);
  auto to_boundary = Boundary(to_attrs);

  for (size_t i = 1; i < merge_ids.size(); ++i) {
    const auto& from_attrs =
        dsg.getNode(merge_ids[i]).attributes<TraversabilityNodeAttributes>();
    auto from_boundary = Boundary(to_attrs);

    // Always update the timestamps to keep the oldest.
    if (from_attrs.first_observed_ns < to_attrs.first_observed_ns) {
      to_attrs.first_observed_ns = from_attrs.first_observed_ns;
      to_attrs.last_observed_ns = from_attrs.last_observed_ns;
    }

    // Simple case: If places are completely contained.
    if (isContained(from_boundary, to_boundary)) {
      continue;
    }

    // Always update the classification of the boundary sides.
    from_boundary.mergeTraversabilityStates(to_boundary, config.min_place_size);
    to_boundary.mergeTraversabilityStates(from_boundary, config.min_place_size);

    // Large overlap that is aligned in X or Y direction: Merge by fusing boundary.
    if ((std::abs(from_boundary.min.x() - to_boundary.min.x()) < config.tolerance &&
         std::abs(from_boundary.max.x() - to_boundary.max.x()) < config.tolerance) ||
        (std::abs(from_boundary.min.y() - to_boundary.min.y()) < config.tolerance &&
         std::abs(from_boundary.max.y() - to_boundary.max.y()) < config.tolerance)) {
      // TODO(lschmid): Can consider more conservative merging but should be fine.
      double new_bot = std::min(from_boundary.min.y(), to_boundary.min.y());
      double new_top = std::max(from_boundary.max.y(), to_boundary.max.y());
      double new_left = std::min(from_boundary.min.x(), to_boundary.min.x());
      double new_right = std::max(from_boundary.max.x(), to_boundary.max.x());
      if (new_top - new_bot > config.max_place_size) {
        const double center = 0.5 * (new_top + new_bot);
        new_bot = center - 0.5 * config.max_place_size;
        new_top = center + 0.5 * config.max_place_size;
      }
      if (new_right - new_left > config.max_place_size) {
        const double center = 0.5 * (new_left + new_right);
        new_left = center - 0.5 * config.max_place_size;
        new_right = center + 0.5 * config.max_place_size;
      }
      to_boundary.setCoordinate(Side::BOTTOM, new_bot);
      to_boundary.setCoordinate(Side::TOP, new_top);
      to_boundary.setCoordinate(Side::LEFT, new_left);
      to_boundary.setCoordinate(Side::RIGHT, new_right);
      to_boundary.toAttributes(to_attrs);
    }
    // If not fuseable keep larger. Already handled by the merge proposal.
  }
  return result;
}

void UpdateTraversabilityFunctor::cleanup(const UpdateInfo::ConstPtr&,
                                          SharedDsgInfo* dsg) const {
  if (!dsg || !dsg->graph->hasLayer(config.layer)) {
    return;
  }

  // 1. Update the classification of all overlapping node boundaries.
  for (const auto& id_pair : overlapping_nodes_to_cleanup_) {
    if (!dsg->graph->hasNode(id_pair.k1) || !dsg->graph->hasNode(id_pair.k2)) {
      continue;
    }
    auto& from_attrs =
        dsg->graph->getNode(id_pair.k1).attributes<TraversabilityNodeAttributes>();
    auto& to_attrs =
        dsg->graph->getNode(id_pair.k2).attributes<TraversabilityNodeAttributes>();
    auto from_boundary = Boundary(from_attrs);
    auto to_boundary = Boundary(to_attrs);
    from_boundary.mergeTraversabilityStates(to_boundary, config.min_place_size);
    from_boundary.toAttributes(from_attrs);
    to_boundary.mergeTraversabilityStates(from_boundary, config.min_place_size);
    to_boundary.toAttributes(to_attrs);
  }
  overlapping_nodes_to_cleanup_.clear();

  // 2. Compute the new distances for all nodes.
  updateDistances(dsg->graph->getLayer(config.layer));
}

std::vector<NodeId> UpdateTraversabilityFunctor::findConnections(
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

bool UpdateTraversabilityFunctor::hasTraversableOverlap(
    const TraversabilityNodeAttributes& from,
    const TraversabilityNodeAttributes& to) const {
  // Simple first implementation: Check the intersecting area meets the minimum place
  // size.
  const auto intersection = Boundary(from).intersection(Boundary(to));
  if (!intersection.valid()) {
    return false;
  }
  return (intersection.width() >= min_connectivity_ &&
          intersection.height() >= min_connectivity_);
}

bool UpdateTraversabilityFunctor::isContained(const Boundary& from,
                                              const Boundary& in) const {
  return (from.min.x() >= in.min.x() - config.tolerance &&
          from.max.x() <= in.max.x() + config.tolerance &&
          from.min.y() >= in.min.y() - config.tolerance &&
          from.max.y() <= in.max.y() + config.tolerance);
}

double UpdateTraversabilityFunctor::computeMetricDistance(const SceneGraphLayer& layer,
                                                          const Eigen::Vector2d& point,
                                                          const NodeSet& to_visit,
                                                          NodeSet& visited) const {
  double min_distance = std::numeric_limits<double>::max();
  NodeSet neighbors;
  for (const auto& node_id : to_visit) {
    visited.insert(node_id);
    const auto& node = layer.getNode(node_id);
    const auto& attrs = node.attributes<TraversabilityNodeAttributes>();
    min_distance = std::min(min_distance, distanceToIntraversable(attrs, point));
    if (min_distance < std::numeric_limits<double>::max()) {
      continue;
    }
    for (const auto& to_id : node.siblings()) {
      if (!visited.count(to_id)) {
        neighbors.insert(to_id);
      }
    }
  }
  if (min_distance < std::numeric_limits<double>::max() || neighbors.empty()) {
    return min_distance;
  }
  return computeMetricDistance(layer, point, neighbors, visited);
}

double UpdateTraversabilityFunctor::distanceToIntraversable(
    const TraversabilityNodeAttributes& attrs, const Eigen::Vector2d& point) const {
  double min_distance = std::numeric_limits<double>::max();
  const Boundary boundary(attrs);
  for (const Side side : Side::ALL) {
    // Check if the side has any intraversable state.
    if (!spark_dsg::areAllTraversable(boundary.states[side], true)) {
      min_distance = std::min(min_distance, boundary.distanceToSide(side, point));
    }
  }
  return min_distance;
}

void UpdateTraversabilityFunctor::computeTopologicalDistances(
    const SceneGraphLayer& layer) const {
  std::queue<NodeId> queue;

  // Initialization.
  for (const auto& [id, node] : layer.nodes()) {
    auto& attrs = node->attributes<TraversabilityNodeAttributes>();
    attrs.distance = std::numeric_limits<double>::max();
    for (const Side side : Side::ALL) {
      if (!spark_dsg::areAllTraversable(attrs.boundary.states[side], true)) {
        attrs.distance = 0.0;
        queue.push(id);
        break;
      }
    }
  }

  // Propagation.
  while (!queue.empty()) {
    const auto& node = layer.getNode(queue.front());
    queue.pop();

    const auto& attrs = node.attributes<TraversabilityNodeAttributes>();
    for (const auto& to_id : node.siblings()) {
      auto& to_attrs = layer.getNode(to_id).attributes<TraversabilityNodeAttributes>();
      if (to_attrs.distance > attrs.distance + 1.0) {
        to_attrs.distance = attrs.distance + 1.0;
        queue.push(to_id);
      }
    }
  }

  // TMP(lschmid): Convert the distances to the edges for room detection via
  // filtration.
  for (const auto& [id, node] : layer.nodes()) {
    auto& attrs = node->attributes<TraversabilityNodeAttributes>();
    for (const auto& to_id : node->siblings()) {
      const auto& to_attrs =
          layer.getNode(to_id).attributes<TraversabilityNodeAttributes>();
      auto& edge = layer.getEdge(id, to_id);
      if (attrs.distance == std::numeric_limits<double>::max()) {
        edge.attributes().weight = to_attrs.distance;
      } else if (to_attrs.distance == std::numeric_limits<double>::max()) {
        edge.attributes().weight = attrs.distance;
      } else {
        edge.attributes().weight = 0.5 * (attrs.distance + to_attrs.distance);
      }
    }
  }
}

void UpdateTraversabilityFunctor::resetNeighborFinder(
    const DynamicSceneGraph& dsg) const {
  nn_ = NearestNodeFinder::fromLayer(
      dsg.getLayer(config.layer),
      [](const SceneGraphNode& node) { return !node.attributes().is_active; });
}

}  // namespace hydra
