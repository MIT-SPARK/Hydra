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
#pragma once
#include <config_utilities/virtual_config.h>

#include <memory>

#include "hydra/common/common.h"
#include "hydra/common/output_sink.h"
#include "hydra/frontend/freespace_places_interface.h"
#include "hydra/places/graph_extractor_config.h"
#include "hydra/places/gvd_integrator_config.h"
#include "hydra/places/gvd_voxel.h"
#include "hydra/reconstruction/tsdf_interpolators.h"
#include "hydra/utils/log_utilities.h"

namespace hydra {

namespace places {
// forward declare to avoid include
class GvdIntegrator;
class GraphExtractorInterface;
}  // namespace places

class GvdPlaceExtractor : public FreespacePlacesInterface {
 public:
  using PositionMatrix = Eigen::Matrix<double, 3, Eigen::Dynamic>;
  using Sink = OutputSink<uint64_t,
                          const Eigen::Isometry3f&,
                          const places::GvdLayer&,
                          const places::GraphExtractorInterface*>;

  struct Config {
    places::GvdIntegratorConfig gvd;
    config::VirtualConfig<places::GraphExtractorInterface> graph;
    config::VirtualConfig<TsdfInterpolator> tsdf_interpolator;
    size_t min_component_size = 3;
    bool filter_places = true;
    bool filter_ground = false;
    double robot_height = 0.0;
    double node_tolerance = 1.0;
    double edge_tolerance = 1.0;
    bool add_freespace_edges = false;
    places::FreespaceEdgeConfig freespace_config;
    std::vector<Sink::Factory> sinks;
  } const config;

  explicit GvdPlaceExtractor(const Config& config);

  virtual ~GvdPlaceExtractor();

  void save(const LogSetup& logs) const override;

  NodeIdSet getActiveNodes() const override;

  // takes in a 3xN matrix
  std::vector<bool> inFreespace(const PositionMatrix& positions,
                                double freespace_distance_m) const override;

  void detect(const ReconstructionOutput& msg) override;

  void updateGraph(uint64_t timestamp_ns, DynamicSceneGraph& graph) override;

  void filterIsolated(DynamicSceneGraph& graph, NodeIdSet& active_neighborhood);

  void filterGround(DynamicSceneGraph& graph);

 protected:
  mutable std::mutex gvd_mutex_;
  places::GvdLayer::Ptr gvd_;
  std::shared_ptr<places::GraphExtractorInterface> graph_extractor_;
  std::unique_ptr<places::GvdIntegrator> gvd_integrator_;
  std::unique_ptr<TsdfInterpolator> tsdf_interpolator_;
  NodeIdSet active_nodes_;
  Eigen::Vector3d latest_pos_;
  Sink::List sinks_;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<FreespacePlacesInterface,
                                     GvdPlaceExtractor,
                                     Config>("gvd");
};

void declare_config(GvdPlaceExtractor::Config& config);

}  // namespace hydra
