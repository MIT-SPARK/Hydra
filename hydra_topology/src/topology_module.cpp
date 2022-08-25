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
#include "hydra_topology/topology_module.h"

#include <kimera_semantics/semantic_tsdf_integrator_factory.h>

namespace hydra {
namespace topology {

TopologyModule::TopologyModule(const TopologyModuleConfig& config,
                               const GvdIntegratorConfig& gvd_config,
                               const TsdfIntegratorConfig& tsdf_config,
                               const SemanticIntegratorConfig& semantic_config)
    : config_(config),
      gvd_config_(gvd_config),
      tsdf_config_(tsdf_config),
      semantic_config_(semantic_config) {
  tsdf_layer_.reset(new Layer<TsdfVoxel>(config_.voxel_size, config_.voxels_per_side));
  semantic_layer_.reset(
      new Layer<kimera::SemanticVoxel>(config_.voxel_size, config_.voxels_per_side));
  gvd_layer_.reset(new Layer<GvdVoxel>(config_.voxel_size, config_.voxels_per_side));
  mesh_layer_.reset(new MeshLayer(tsdf_layer_->block_size()));

  tsdf_integrator_ = kimera::SemanticTsdfIntegratorFactory::create(
      "fast", tsdf_config_, semantic_config_, tsdf_layer_.get(), semantic_layer_.get());
  gvd_integrator_.reset(
      new GvdIntegrator(gvd_config_, tsdf_layer_.get(), gvd_layer_, mesh_layer_));
}

void TopologyModule::showStats() const {
  const std::string tsdf_memory_str =
      hydra_utils::getHumanReadableMemoryString(tsdf_layer_->getMemorySize());
  const std::string gvd_memory_str =
      hydra_utils::getHumanReadableMemoryString(gvd_layer_->getMemorySize());
  const std::string mesh_memory_str =
      hydra_utils::getHumanReadableMemoryString(mesh_layer_->getMemorySize());
  LOG(INFO) << "Memory used: [TSDF=" << tsdf_memory_str << ", GVD=" << gvd_memory_str
            << ", Mesh= " << mesh_memory_str << "]";
}

void TopologyModule::update(const voxblox::Transformation& T_G_C,
                            const voxblox::Pointcloud& pointcloud,
                            const voxblox::Colors& colors,
                            bool full_update) {
  tsdf_integrator_->integratePointCloud(T_G_C, pointcloud, colors, false);

  if (!tsdf_layer_ || tsdf_layer_->getNumberOfAllocatedBlocks() == 0) {
    return;
  }

  if (!full_update) {
    return;
  }

  // TODO(nathan) get actual timestamp
  gvd_integrator_->updateFromTsdfLayer(0, true);

  BlockIndexList archived_blocks;
  if (config_.clear_distant_blocks) {
    archived_blocks = gvd_integrator_->removeDistantBlocks(
        T_G_C.getPosition(), config_.dense_representation_radius_m);

    // this needs to be paired with publishMesh (i.e. generateVoxbloxMeshMsg)
    // to actual remove allocated blocks (instead of getting rid of the contents).
    // handled through publishMesh
    mesh_layer_->clearDistantMesh(T_G_C.getPosition(),
                                  config_.dense_representation_radius_m);
  }

  // publishMesh(timestamp, archived_blocks);

  if (config_.show_stats) {
    showStats();
  }
}

}  // namespace topology
}  // namespace hydra
