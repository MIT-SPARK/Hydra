// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// TODO(nathan) figure out ETHZ license

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
