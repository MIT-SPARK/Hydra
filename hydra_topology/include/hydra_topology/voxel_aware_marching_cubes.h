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
#include "hydra_topology/gvd_voxel.h"
#include "hydra_topology/voxblox_types.h"

#include <voxblox/mesh/marching_cubes.h>

namespace hydra {
namespace topology {

using PointMatrix = Eigen::Matrix<FloatingPoint, 3, 8>;
using SdfMatrix = Eigen::Matrix<FloatingPoint, 8, 1>;
using EdgeIndexMatrix = Eigen::Matrix<FloatingPoint, 3, 12>;

void interpolateEdges(const PointMatrix& vertex_coords,
                      const SdfMatrix& vertex_sdf,
                      EdgeIndexMatrix& edge_coords,
                      std::vector<uint8_t>& edge_status,
                      const std::vector<GvdVoxel*>& gvd_voxels);

/**
 * Performs the marching cubes algorithm to generate a mesh layer from a TSDF.
 * Implementation taken from Open Chisel
 * https://github.com/personalrobotics/OpenChisel
 */
class VoxelAwareMarchingCubes : voxblox::MarchingCubes {
 public:
  VoxelAwareMarchingCubes();

  virtual ~VoxelAwareMarchingCubes() = default;

  static void meshCube(const BlockIndex& block,
                       const PointMatrix& vertex_coords,
                       const SdfMatrix& vertex_sdf,
                       VertexIndex* next_index,
                       Mesh* mesh,
                       const std::vector<GvdVoxel*>& gvd_voxels,
                       const std::vector<bool>& voxels_in_block);
};

}  // namespace topology
}  // namespace hydra
