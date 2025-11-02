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
#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>
#include <hydra/common/global_info.h>
#include <kimera_pgmo/deformation_graph.h>
#include <spark_dsg/printing.h>

#include "hydra/backend/update_places_functor.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {
namespace {

static const auto registration_ =
    config::RegistrationWithConfig<UpdateFunctor,
                                   MstPlacesUpdateFunctor,
                                   MstPlacesUpdateFunctor::Config>(
        "MstPlacesUpdateFunctor");

}

using timing::ScopedTimer;
using MergeId = std::optional<NodeId>;

void declare_config(MstPlacesUpdateFunctor::Config& config) {
  using namespace config;
  name("MstPlacesUpdateFunctor::Config");
  field(config.pos_threshold_m, "pos_threshold_m", "m");
  field(config.distance_tolerance_m, "distance_tolerance_m", "m");
  field(config.merge_proposer, "merge_proposer");
  field(config.layer, "layer");
}

MstPlacesUpdateFunctor::MstPlacesUpdateFunctor(const Config& config)
    : config(config::checkValid(config)), merge_proposer(config.merge_proposer) {}

UpdateFunctor::Hooks MstPlacesUpdateFunctor::hooks() const {
  auto my_hooks = UpdateFunctor::hooks();
  my_hooks.find_merges = [this](const auto& graph, const auto& info) {
    return findMerges(graph, info);
  };

  return my_hooks;
}

void MstPlacesUpdateFunctor::call(const UpdateInfo& info,
                                  const DynamicSceneGraph&,
                                  DynamicSceneGraph& optimized) const {
  ScopedTimer spin_timer("backend/update_places", info.timestamp_ns);
  const auto places = optimized.findLayer(config.layer);
  if (!places) {
    MLOG(1) << "[PLACES update] No layer named '" << config.layer << "'";
    return;
  }

  if (!info.places_values) {
    MLOG(1) << "[PLACES update] No places in temp values";
    return;
  }

  const auto new_loopclosure = info.loop_closure_detected;
  active_tracker.clear();  // reset from previous pass
  const auto view = new_loopclosure ? LayerView(*places) : active_tracker.view(*places);

  size_t num_changed = 0;
  const auto& places_values = *info.places_values;
  for (const auto& node : view) {
    if (!places_values.exists(node.id)) {
      // this happens for the GT version
      MLOG(2) << "[PLACES update] missing place " << NodeSymbol(node.id).str()
              << " from places factors.";
      continue;
    }

    ++num_changed;
    auto& attrs = node.attributes();
    attrs.position = places_values.at<gtsam::Pose3>(node.id).translation();
    // TODO(nathan) consider updating distance via parents + deformation graph
  }

  MLOG(1) << "[PLACES update] Updated " << num_changed << " node(s)";
}

MergeList MstPlacesUpdateFunctor::findMerges(const DynamicSceneGraph& graph,
                                             const UpdateInfo& info) const {
  const auto new_lcd = info.loop_closure_detected;
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
