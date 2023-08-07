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
#include "hydra/reconstruction/combo_integrator.h"

namespace hydra {

using voxblox::Layer;

ComboIntegrator::ComboIntegrator(const places::GvdIntegratorConfig& gvd_config,
                                 const Layer<voxblox::TsdfVoxel>::Ptr& tsdf_layer,
                                 const Layer<places::GvdVoxel>::Ptr& gvd_layer,
                                 const SemanticMeshLayer::Ptr& mesh_layer,
                                 const MeshIntegratorConfig* mesh_config,
                                 const GraphExtractorConfig& graph_config)
    : tsdf_(tsdf_layer), mesh_(mesh_layer) {
  vertices_.reset(new Layer<places::VertexVoxel>(gvd_layer->voxel_size(),
                                                 gvd_layer->voxels_per_side()));
  mesh_integrator = std::make_unique<MeshIntegrator>(
      mesh_config ? *mesh_config : MeshIntegratorConfig(),
      tsdf_layer,
      vertices_,
      mesh_layer);
  graph_extractor = graph_config.create();
  gvd_integrator =
      std::make_unique<places::GvdIntegrator>(gvd_config, gvd_layer, graph_extractor);
}

ComboIntegrator::~ComboIntegrator() = default;

void ComboIntegrator::update(uint64_t timestamp_ns,
                             bool clear_updated_flag,
                             bool use_all_blocks) {
  mesh_integrator->generateMesh(!use_all_blocks, clear_updated_flag);
  gvd_integrator->updateFromTsdf(
      timestamp_ns, *tsdf_, *vertices_, *mesh_, clear_updated_flag, use_all_blocks);
  gvd_integrator->updateGvd(timestamp_ns);
}

}  // namespace hydra
