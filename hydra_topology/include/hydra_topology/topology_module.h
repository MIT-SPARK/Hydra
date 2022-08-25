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
#include "hydra_topology/configs.h"

#include <hydra_utils/display_utils.h>
#include <kimera_semantics/semantic_integrator_base.h>
#include <kimera_semantics/semantic_voxel.h>

namespace hydra {
namespace topology {

struct TopologyModuleConfig {
  float voxel_size = 0.1;
  int voxels_per_side = 16;
  bool show_stats = true;
  bool clear_distant_blocks = true;
  double dense_representation_radius_m = 5.0;
};

using SemanticIntegratorConfig = kimera::SemanticIntegratorBase::SemanticConfig;
using TsdfIntegratorConfig = voxblox::TsdfIntegratorBase::Config;

class TopologyModule {
 public:
  TopologyModule(const TopologyModuleConfig& config,
                 const GvdIntegratorConfig& gvd_config,
                 const TsdfIntegratorConfig& tsdf_config,
                 const SemanticIntegratorConfig& semantic_config);

  void update(const voxblox::Transformation& T_G_C,
              const voxblox::Pointcloud& pointcloud,
              const voxblox::Colors& colors,
              bool full_update);

  void showStats() const;

 protected:
  TopologyModuleConfig config_;
  GvdIntegratorConfig gvd_config_;
  TsdfIntegratorConfig tsdf_config_;
  SemanticIntegratorConfig semantic_config_;

  Layer<TsdfVoxel>::Ptr tsdf_layer_;
  Layer<kimera::SemanticVoxel>::Ptr semantic_layer_;
  Layer<GvdVoxel>::Ptr gvd_layer_;
  MeshLayer::Ptr mesh_layer_;

  std::unique_ptr<voxblox::TsdfIntegratorBase> tsdf_integrator_;
  std::unique_ptr<GvdIntegrator> gvd_integrator_;
};

}  // namespace topology
}  // namespace hydra
