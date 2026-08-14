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
#include <pose_graph_tools/pose_graph.h>

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

void MstPlaceFactors::updateProblem(uint64_t timestamp_ns,
                                    const spark_dsg::SceneGraph& graph,
                                    kimera_pgmo::DeformationGraph& deformation_graph,
                                    const NodeRobotMap* robot_lookup) const {
  const auto& places = graph.getLayer(DsgLayers::PLACES);
  if (places.nodes().empty()) {
    LOG(WARNING) << "Attempting to add places to deformation graph without places";
    return;
  }

  const auto this_vertex_key = GlobalInfo::instance().getRobotPrefix().vertex_key;
  ScopedTimer timer("backend/mst_place_factors", timestamp_ns);
  deformation_graph.clearTemporaryStructures();

  MinimumSpanningTreeInfo mst_info;
  {  // start timing scope
    ScopedTimer mst_timer("backend/places_mst", timestamp_ns);
    mst_info = getMinimumSpanningEdges(places);
  }  // end timing scope

  {  // start timing scope
    ScopedTimer add_timer("backend/add_places_nodes", timestamp_ns);

    kimera_pgmo::NodeValenceInfoList factors;
    for (const auto& [node_id, node] : places.nodes()) {
      const auto& attrs = node->attributes<PlaceNodeAttributes>();
      if (!node->hasSiblings()) {
        continue;
      }

      auto& factor = factors.emplace_back();
      if (robot_lookup) {
      } else {
        factor.valence_prefix = this_vertex_key;
      }
      factor.key = node_id;
      factor.pose = gtsam::Pose3(gtsam::Rot3(), attrs.position);

      if (mst_info.leaves.count(node_id)) {
        for (const auto& idx : attrs.deformation_connections) {
          if (idx == std::numeric_limits<size_t>::max()) {
            continue;
          }
          factor.valence.push_back(idx);
        }
      }
    }

    deformation_graph.processNewTempNodesValences(factors, false, config.mesh_variance);
  }  // end timing scope

  {  // start timing scope
    ScopedTimer between_timer("backend/add_places_between", timestamp_ns);
    const auto robot_id = GlobalInfo::instance().getRobotPrefix().id;
    pose_graph_tools::PoseGraph mst_edges;
    for (const auto& edge : mst_info.edges) {
      gtsam::Pose3 source(gtsam::Rot3(), getNodePosition(places, edge.source));
      gtsam::Pose3 target(gtsam::Rot3(), getNodePosition(places, edge.target));
      pose_graph_tools::PoseGraphEdge mst_e;
      mst_e.key_from = edge.source;
      mst_e.key_to = edge.target;
      mst_e.robot_from = robot_id;
      mst_e.robot_to = robot_id;
      // TODO(nathan) this should technically be something else
      mst_e.type = pose_graph_tools::PoseGraphEdge::Type::MESH;
      mst_e.stamp_ns = 0;
      mst_e.pose = source.between(target).matrix();
      mst_e.covariance.setIdentity();
      mst_edges.edges.push_back(mst_e);
    }

    deformation_graph.processNewTempEdges(mst_edges, config.edge_variance);
  }  // end timing scope
}

}  // namespace hydra
