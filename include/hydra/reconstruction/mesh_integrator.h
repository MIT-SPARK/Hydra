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
#include <kimera_semantics/semantic_voxel.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <voxblox/integrator/integrator_utils.h>
#include <voxblox/mesh/mesh_layer.h>

#include <thread>

#include "hydra/places/vertex_voxel.h"
#include "hydra/reconstruction/mesh_integrator_config.h"
#include "hydra/reconstruction/semantic_mesh_layer.h"

namespace hydra {

class MeshIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MeshIntegrator(
      const MeshIntegratorConfig& config,
      const voxblox::Layer<voxblox::TsdfVoxel>::Ptr& sdf_layer,
      const voxblox::Layer<places::VertexVoxel>::Ptr& vertex_layer,
      const SemanticMeshLayer::Ptr& mesh_layer,
      const voxblox::Layer<kimera::SemanticVoxel>::Ptr& semantic_layer = nullptr);

  virtual ~MeshIntegrator() = default;

  virtual void generateMesh(bool only_mesh_updated_blocks, bool clear_updated_flag);

  virtual void extractMeshInsideBlock(const voxblox::Block<voxblox::TsdfVoxel>& block,
                                      const voxblox::VoxelIndex& index,
                                      const voxblox::Point& coords,
                                      voxblox::VertexIndex* next_mesh_index,
                                      voxblox::Mesh* mesh);

  virtual void extractMeshOnBorder(const voxblox::Block<voxblox::TsdfVoxel>& block,
                                   const voxblox::VoxelIndex& index,
                                   const voxblox::Point& coords,
                                   voxblox::VertexIndex* next_mesh_index,
                                   voxblox::Mesh* mesh);

  void processInterior(const voxblox::BlockIndexList& blocks,
                       voxblox::ThreadSafeIndex* index_getter);

  void processExterior(const voxblox::BlockIndexList& blocks,
                       voxblox::ThreadSafeIndex* index_getter);

  void updateBlockInterior(const voxblox::BlockIndex& block_index);

  void updateBlockExterior(const voxblox::BlockIndex& block_index);

  void launchThreads(const voxblox::BlockIndexList& blocks, bool interior_pass);

  voxblox::BlockIndex getNeighborBlockIndex(const voxblox::BlockIndex& block_idx,
                                            voxblox::VoxelIndex& corner_index);

  void updateMeshColor(const voxblox::Block<voxblox::TsdfVoxel>& block,
                       voxblox::Mesh* mesh,
                       const voxblox::BlockIndex& index);

 protected:
  MeshIntegratorConfig config_;

  voxblox::Layer<voxblox::TsdfVoxel>::Ptr sdf_layer_;
  SemanticMeshLayer::Ptr mesh_layer_;
  voxblox::Layer<places::VertexVoxel>::Ptr vertex_layer_;
  voxblox::Layer<kimera::SemanticVoxel>::Ptr semantic_layer_;

  voxblox::FloatingPoint voxel_size_;
  size_t voxels_per_side_;
  voxblox::FloatingPoint block_size_;

  Eigen::Matrix<int, 3, 8> cube_index_offsets_;
  Eigen::Matrix<voxblox::FloatingPoint, 3, 8> cube_coord_offsets_;
};

}  // namespace hydra
