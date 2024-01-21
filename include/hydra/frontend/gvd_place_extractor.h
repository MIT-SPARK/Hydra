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
#include "hydra/frontend/place_extractor_interface.h"
#include "hydra/places/gvd_integrator_config.h"
#include "hydra/places/gvd_voxel.h"
#include "hydra/utils/log_utilities.h"

namespace hydra {

namespace places {
// forward declare to avoid include
class GvdIntegrator;
}  // namespace places

class GvdPlaceExtractor : public PlaceExtractorInterface {
 public:
  using PositionMatrix = Eigen::Matrix<double, 3, Eigen::Dynamic>;
  using ExtractorConfig = config::VirtualConfig<places::GraphExtractorInterface>;

  GvdPlaceExtractor(const ExtractorConfig& graph_config,
                    const places::GvdIntegratorConfig& gvd_config,
                    size_t min_component_size,
                    bool filter_places);

  virtual ~GvdPlaceExtractor();

  void save(const LogSetup& logs) const override;

  NodeIdSet getActiveNodes() const override;

  // takes in a 3xN matrix
  std::vector<bool> inFreespace(const PositionMatrix& positions,
                                double freespace_distance_m) const override;

  void detect(const ReconstructionOutput& msg) override;

  void detect(uint64_t timestamp_ns,
              const VolumetricMap& map,
              const voxblox::BlockIndexList& archived_blocks) override;

  void updateGraph(uint64_t timestamp_ns, DynamicSceneGraph& graph) override;

 protected:
  const places::GvdIntegratorConfig gvd_config_;
  const size_t min_component_size_;
  const bool filter_places_;

  mutable std::mutex gvd_mutex_;
  voxblox::Layer<places::GvdVoxel>::Ptr gvd_;
  std::shared_ptr<places::GraphExtractorInterface> graph_extractor_;
  std::unique_ptr<places::GvdIntegrator> gvd_integrator_;
  NodeIdSet active_nodes_;
};

}  // namespace hydra
