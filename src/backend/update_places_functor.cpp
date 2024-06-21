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
#include "hydra/backend/update_places_functor.h"

#include <config_utilities/config.h>
#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>

#include "hydra/utils/timing_utilities.h"

namespace hydra {

using timing::ScopedTimer;
using MergeId = std::optional<NodeId>;

UpdatePlacesFunctor::UpdatePlacesFunctor(double pos_threshold,
                                         double distance_tolerance)
    : pos_threshold_m(pos_threshold), distance_tolerance_m(distance_tolerance) {}

void UpdatePlacesFunctor::updatePlace(const gtsam::Values& values,
                                      NodeId node,
                                      NodeAttributes& attrs) const {
  if (!values.exists(node)) {
    VLOG(5) << "[Hydra Backend] missing place " << NodeSymbol(node).getLabel()
            << " from places factors.";
    return;
  }

  attrs.position = values.at<gtsam::Pose3>(node).translation();
  // TODO(nathan) consider updating distance via parents + deformation graph
}

MergeId UpdatePlacesFunctor::proposeMerge(const SceneGraphLayer& layer,
                                          const SceneGraphNode& from_node) const {
  const auto& from_attrs = from_node.attributes<PlaceNodeAttributes>();
  std::list<NodeId> candidates;
  node_finder->find(from_attrs.position,
                    num_merges_to_consider,
                    !from_attrs.is_active,
                    [&candidates](NodeId place_id, size_t, double) {
                      candidates.push_back(place_id);
                    });

  for (const auto& id : candidates) {
    // TODO(nathan) reconsider this
    if (from_node.siblings().count(id)) {
      continue;  // avoid merging siblings
    }

    const auto& to_attrs = layer.getNode(id).attributes<PlaceNodeAttributes>();
    if ((from_attrs.position - to_attrs.position).norm() > pos_threshold_m) {
      continue;
    }

    if (std::abs(from_attrs.distance - to_attrs.distance) > distance_tolerance_m) {
      continue;
    }

    return id;
  }

  return std::nullopt;
}

// drops any isolated place nodes that would cause an inderminate system error
void UpdatePlacesFunctor::filterMissing(DynamicSceneGraph& graph,
                                        const std::list<NodeId> missing_nodes) const {
  if (missing_nodes.empty()) {
    return;
  }

  VLOG(5) << "[Places Layer]: could not update "
          << displayNodeSymbolContainer(missing_nodes);

  for (const auto& node_id : missing_nodes) {
    if (!graph.hasNode(node_id)) {
      continue;
    }

    const auto& node = graph.getNode(node_id);
    if (!node.attributes().is_active && !node.hasSiblings()) {
      VLOG(2) << "[Places Layer]: removing node " << NodeSymbol(node_id).getLabel();
      graph.removeNode(node_id);
    }
  }
}

MergeList UpdatePlacesFunctor::call(const DynamicSceneGraph& unmerged,
                                    SharedDsgInfo& dsg,
                                    const UpdateInfo::ConstPtr& info) const {
  ScopedTimer spin_timer("backend/update_places", info->timestamp_ns);

  if (!unmerged.hasLayer(DsgLayers::PLACES) || !info->places_values) {
    return {};
  }

  MergeList proposals;
  bool has_given_merges = false;
  auto iter = info->given_merges.find(DsgLayers::PLACES);
  if (iter != info->given_merges.end()) {
    has_given_merges = true;
    proposals = iter->second;
  }

  const auto& places = unmerged.getLayer(DsgLayers::PLACES);
  const auto& places_values = *info->places_values;
  if (places_values.size() == 0 && !info->allow_node_merging) {
    return proposals;
  }

  // node finder constructed from optimized graph
  node_finder = NearestNodeFinder::fromLayer(places, [](const SceneGraphNode& node) {
    return !node.attributes().is_active &&
           node.attributes<PlaceNodeAttributes>().real_place;
  });

  // we want to iterate over the unmerged graph
  LayerView view;
  if (info->loop_closure_detected) {
    view = LayerView(unmerged.getLayer(DsgLayers::PLACES));
  } else {
    view = active_tracker.view(unmerged.getLayer(DsgLayers::PLACES));
  }

  size_t num_changed = 0;
  std::list<NodeId> missing_nodes;
  for (const auto& node : view) {
    ++num_changed;
    auto& attrs = node.attributes<PlaceNodeAttributes>();
    if (!places_values.exists(node.id)) {
      // this happens for the GT version
      missing_nodes.push_back(node.id);
      continue;
    }

    updatePlace(places_values, node.id, attrs);
    dsg.graph->setNodeAttributes(node.id, attrs.clone());
  }

  VLOG(2) << "[Hydra Backend] Places update: " << num_changed << " nodes";
  filterMissing(*dsg.graph, missing_nodes);

  if (!has_given_merges && node_finder) {
    for (const auto& node : view) {
      const auto proposed = proposeMerge(places, node);
      if (proposed) {
        proposals.push_back({node.id, *proposed});
      }
    }
  }

  active_tracker.clear();
  return proposals;
}

}  // namespace hydra
