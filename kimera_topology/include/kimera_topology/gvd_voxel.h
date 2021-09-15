#pragma once
#include "kimera_topology/voxblox_types.h"

#include <iostream>

namespace kimera {
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

  bool is_voronoi_parent = false;
  GlobalIndex::Scalar nearest_voronoi[3];
  GlobalIndex::Scalar nearest_voronoi_distance;
};

std::ostream& operator<<(std::ostream& out, const GvdVoxel& voxel);

struct GvdVertexInfo {
  size_t vertex;
  double pos[3];
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
}  // namespace kimera
