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

#include "hydra/common/output_sink.h"
#include "hydra/frontend/surface_places_interface.h"
#include "hydra/places/traversability_clustering.h"
#include "hydra/places/traversability_estimator.h"
#include "hydra/places/traversability_postprocessing.h"

namespace hydra::places {

class TraversabilityPlaceExtractor : public SurfacePlacesInterface {
 public:
  using Sink = OutputSink<uint64_t, const Eigen::Vector3d&, const TraversabilityLayer&>;

  struct Config {
    config::VirtualConfig<TraversabilityEstimator> estimator;
    config::VirtualConfig<TraversabilityClustering> clustering;
    TraversabilityProcessors::Config postprocessing;
    std::vector<Sink::Factory> sinks;
  } const config;

  explicit TraversabilityPlaceExtractor(const Config& config);

  virtual ~TraversabilityPlaceExtractor() = default;

  NodeIdSet getActiveNodes() const override;

  void detect(const ActiveWindowOutput& msg,
              const kimera_pgmo::MeshDelta& mesh_delta,
              const DynamicSceneGraph& graph) override;

  void updateGraph(const ActiveWindowOutput& msg, DynamicSceneGraph& graph) override;

 protected:
  NodeIdSet active_nodes_;
  TraversabilityEstimator::Ptr estimator_;
  TraversabilityClustering::Ptr clustering_;
  const TraversabilityProcessors postprocessing_;
  Sink::List sinks_;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<SurfacePlacesInterface,
                                     TraversabilityPlaceExtractor,
                                     Config>("traversability");
};

void declare_config(TraversabilityPlaceExtractor::Config& config);

}  // namespace hydra::places
