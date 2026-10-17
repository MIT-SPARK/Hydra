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
#include "hydra/backend/mst_factors.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <gtsam/slam/BetweenFactor.h>
#include <kimera_pgmo/deformation_edge_factor.h>
#include <kimera_pgmo/deformation_graph.h>
#include <kimera_pgmo/utils/common_functions.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/scene_graph.h>

#include "hydra/common/global_info.h"
#include "hydra/utils/minimum_spanning_tree.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {
namespace {

static const auto registration =
    config::RegistrationWithConfig<OptimizationHook,
                                   MstPlaceFactors,
                                   MstPlaceFactors::Config>("MstPlaceFactors");

}

using timing::ScopedTimer;
using namespace spark_dsg;

void declare_config(MstPlaceFactors::Config& config) {
  using namespace config;
  name("MstPlaceFactors::Config");
  field(config.mesh_variance, "mesh_variance");
  field(config.edge_variance, "edge_variance");
  check(config.mesh_variance, GT, 0.0, "mesh_variance");
  check(config.edge_variance, GT, 0.0, "edge_variance");
}

MstPlaceFactors::MstPlaceFactors(const Config& config)
    : config(config::checkValid(config)) {}

void MstPlaceFactors::updateProblem(
    uint64_t timestamp_ns,
    const SceneGraph& graph,
    const kimera_pgmo::DeformationGraph& deformation_graph,
    kimera_pgmo::OptimizationProblem& extras,
    const NodeRobotMap* robot_lookup) const {
  const auto& places = graph.getLayer(DsgLayers::PLACES);
  if (places.numNodes() == 0) {
    LOG(WARNING) << "Attempting to build place constraints without places";
    return;
  }

  const auto this_vertex_key = GlobalInfo::instance().getRobotPrefix().vertex_key;
  ScopedTimer timer("backend/mst_place_factors", timestamp_ns);

  MinimumSpanningTreeInfo mst_info;
  {  // start timing scope
    ScopedTimer mst_timer("backend/places_mst", timestamp_ns);
    mst_info = getMinimumSpanningEdges(places);
  }  // end timing scope

  {  // start timing scope
    ScopedTimer add_timer("backend/add_places_nodes", timestamp_ns);

    const auto noise = gtsam::noiseModel::Isotropic::Variance(3, config.mesh_variance);
    for (const auto& node : places.nodes()) {
      const auto& attrs = node.attributes<PlaceNodeAttributes>();
      if (!node.hasSiblings()) {
        continue;
      }

      const auto pose = gtsam::Pose3(gtsam::Rot3(), attrs.position);
      extras.initial.insert(node.id, pose);
      auto vertex_prefix = this_vertex_key;
      if (robot_lookup) {
        vertex_prefix = kimera_pgmo::GetVertexPrefix(robot_lookup->at(node.id));
      }

      if (!mst_info.leaves.count(node.id)) {
        continue;
      }

      for (const auto idx : attrs.deformation_connections) {
        if (idx == std::numeric_limits<size_t>::max()) {
          continue;
        }

        const auto vertex = gtsam::Symbol(vertex_prefix, idx);
        if (!deformation_graph.getValues()->exists(vertex)) {
          continue;
        }

        const auto position =
            deformation_graph.getInitialPositionVertex(vertex_prefix, idx);
        extras.factors.emplace_shared<kimera_pgmo::DeformationEdgeFactor>(
            node.id, vertex, pose, position, noise);
        extras.factors.emplace_shared<kimera_pgmo::DeformationEdgeFactor>(
            vertex,
            node.id,
            gtsam::Pose3(gtsam::Rot3(), position),
            attrs.position,
            noise);
      }
    }
  }  // end timing scope

  {  // start timing scope
    ScopedTimer between_timer("backend/add_places_between", timestamp_ns);
    gtsam::Vector6 variances;
    variances.head<3>().setConstant(1.0e-2 * config.edge_variance);
    variances.tail<3>().setConstant(config.edge_variance);
    const auto noise = gtsam::noiseModel::Diagonal::Variances(variances);
    for (const auto& [s_id, t_id, dist] : mst_info.edges) {
      if (!extras.initial.exists(s_id) || !extras.initial.exists(t_id)) {
        continue;
      }

      const auto source = extras.initial.at<gtsam::Pose3>(s_id);
      const auto target = extras.initial.at<gtsam::Pose3>(t_id);
      extras.factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          s_id, t_id, source.between(target), noise);
    }
  }  // end timing scope
}

}  // namespace hydra
