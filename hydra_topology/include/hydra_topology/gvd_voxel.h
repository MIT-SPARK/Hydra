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
#include "hydra_topology/voxblox_types.h"

#include <iostream>

namespace hydra {
namespace topology {

// TODO(nathan) packed?
struct GvdVoxel {
  float distance;
  bool observed = false;
  bool fixed = false;
  bool in_queue = false;

  bool has_parent = false;
  GlobalIndex::Scalar parent[3];
  // required for removing blocks (parents leave a dangling reference otherwise)
  voxblox::Point::Scalar parent_pos[3];

  uint8_t num_extra_basis = 0;

  bool on_surface = false;
  // TODO(nathan) leave this unitialized
  size_t block_vertex_index = 123456789;
  int32_t mesh_block[3];

  bool is_voronoi_parent = false;
  GlobalIndex::Scalar nearest_voronoi[3];
  GlobalIndex::Scalar nearest_voronoi_distance;
};

std::ostream& operator<<(std::ostream& out, const GvdVoxel& voxel);

struct GvdVertexInfo {
  size_t vertex;
  double pos[3];
  int32_t block[3];
  size_t ref_count = 0;
};

using GvdParentMap = voxblox::LongIndexHashMapType<voxblox::LongIndexSet>::type;
using GvdVertexMap = voxblox::LongIndexHashMapType<GvdVertexInfo>::type;
using GvdNeighborhood = Neighborhood<voxblox::Connectivity::kTwentySix>;

template <typename Scalar = double>
inline Eigen::Matrix<Scalar, 3, 1> getVoxelPosition(const Layer<GvdVoxel>& layer,
                                                    const GlobalIndex& index) {
  BlockIndex block_idx;
  VoxelIndex voxel_idx;
  voxblox::getBlockAndVoxelIndexFromGlobalVoxelIndex(
      index, layer.voxels_per_side(), &block_idx, &voxel_idx);

  CHECK(layer.hasBlock(block_idx))
      << "Attempting to look up coordinates for " << index.transpose()
      << ", which is outside of the allocated blocks";

  return layer.getBlockByIndex(block_idx)
      .computeCoordinatesFromVoxelIndex(voxel_idx)
      .cast<Scalar>();
}

}  // namespace topology
}  // namespace hydra
