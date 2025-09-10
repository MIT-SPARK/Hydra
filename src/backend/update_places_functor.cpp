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
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>
#include <hydra/common/global_info.h>
#include <kimera_pgmo/deformation_graph.h>
#include <spark_dsg/printing.h>

#include "hydra/utils/timing_utilities.h"

namespace hydra {

using timing::ScopedTimer;
using MergeId = std::optional<NodeId>;

void declare_config(UpdatePlacesFunctor::Config& config) {
  using namespace config;
  name("UpdatePlacesFunctor::Config");
  field(config.pos_threshold_m, "pos_threshold_m", "m");
  field(config.distance_tolerance_m, "distance_tolerance_m", "m");
  field(config.deformation_interpolator, "deformation_interpolator");
  field(config.merge_proposer, "merge_proposer");
  field(config.layer, "layer");
}

UpdatePlacesFunctor::UpdatePlacesFunctor(const Config& config)
    : config(config::checkValid(config)),
      merge_proposer(config.merge_proposer),
      deformation_interpolator(config.deformation_interpolator) {}

UpdateFunctor::Hooks UpdatePlacesFunctor::hooks() const {
  auto my_hooks = UpdateFunctor::hooks();
  my_hooks.find_merges = [this](const auto& graph, const auto& info) {
    return findMerges(graph, info);
  };

  return my_hooks;
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
      VLOG(2) << "[Places Layer]: removing node " << NodeSymbol(node_id).str();
      graph.removeNode(node_id);
    }
  }
}

size_t UpdatePlacesFunctor::updateFromValues(const LayerView& view,
                                             SharedDsgInfo& dsg,
                                             const UpdateInfo::ConstPtr& info) const {
  if (!info->places_values) {
    return 0;
  }

  size_t num_changed = 0;
  const auto& places_values = *info->places_values;
  for (const auto& node : view) {
    if (!places_values.exists(node.id)) {
      // this happens for the GT version
      VLOG(10) << "[Hydra Backend] missing place " << NodeSymbol(node.id).str()
               << " from places factors.";
      continue;
    }

    // TODO(nathan) consider updating distance via parents + deformation graph
    ++num_changed;
    auto& attrs = node.attributes();
    attrs.position = places_values.at<gtsam::Pose3>(node.id).translation();
    dsg.graph->setNodeAttributes(node.id, attrs.clone());
  }

  // TODO(nathan) fix this
  // filterMissing(*dsg.graph, missing_nodes);
  return num_changed;
}

void UpdatePlacesFunctor::call(const DynamicSceneGraph& unmerged,
                               SharedDsgInfo& dsg,
                               const UpdateInfo::ConstPtr& info) const {
  ScopedTimer spin_timer("backend/update_places", info->timestamp_ns);

  if (!unmerged.hasLayer(config.layer)) {
    return;
  }

  const auto new_loopclosure = info->loop_closure_detected;
  const auto& places = unmerged.getLayer(config.layer);
  active_tracker.clear();  // reset from previous pass
  const auto view = new_loopclosure ? LayerView(places) : active_tracker.view(places);

  size_t num_changed = 0;
  if (!info->places_values || info->places_values->size() == 0) {
    deformation_interpolator.interpolateNodePositions(unmerged, *dsg.graph, info, view);
  } else {
    num_changed = updateFromValues(view, dsg, info);
  }

  VLOG(2) << "[Hydra Backend] Places update: " << num_changed << " nodes";
}

MergeList UpdatePlacesFunctor::findMerges(const DynamicSceneGraph& graph,
                                          const UpdateInfo::ConstPtr& info) const {
  const auto new_lcd = info->loop_closure_detected;
  const auto& places = graph.getLayer(config.layer);
  // freeze layer view to avoid messing with tracker
  const auto view = new_lcd ? LayerView(places) : active_tracker.view(places, true);

  MergeList proposals;
  merge_proposer.findMerges(
      places,
      view,
      [this](const SceneGraphNode& lhs, const SceneGraphNode& rhs) {
        const auto lhs_attrs = lhs.tryAttributes<PlaceNodeAttributes>();
        const auto rhs_attrs = rhs.tryAttributes<PlaceNodeAttributes>();
        if (!lhs_attrs || !rhs_attrs) {
          LOG(WARNING) << "Invalid place nodes: " << NodeSymbol(lhs.id).str() << ", "
                       << NodeSymbol(rhs.id).str();
          return false;
        }

        if (!lhs_attrs->real_place || !rhs_attrs->real_place) {
          return false;
        }

        const auto distance = (lhs_attrs->position - rhs_attrs->position).norm();
        if (distance > config.pos_threshold_m) {
          return false;
        }

        const auto radii_deviation =
            std::abs(lhs_attrs->distance - rhs_attrs->distance);
        return radii_deviation <= config.distance_tolerance_m;
      },
      proposals);
  return proposals;
}

}  // namespace hydra
