#pragma once
#include "kimera_topology/voxblox_types.h"

#include <iostream>

namespace kimera {
namespace topology {

// TODO(nathan) packed?
struct GvdVoxel {
  float distance;
  bool observed = false;
  bool on_surface = false;
  bool fixed = false;
  bool in_queue = false;

  bool has_parent = false;
  GlobalIndex::Scalar parent[3];

  uint8_t num_extra_basis = 0;
  // TODO(nathan) on GVD boundary flag

  bool is_voronoi_parent = false;
  GlobalIndex::Scalar nearest_voronoi[3];
  GlobalIndex::Scalar nearest_voronoi_distance;
};

std::ostream& operator<<(std::ostream& out, const GvdVoxel& voxel);

using GvdParentMap = voxblox::LongIndexHashMapType<voxblox::LongIndexSet>::type;
using GvdNeighborhood = Neighborhood<voxblox::Connectivity::kTwentySix>;

}  // namespace topology
}  // namespace kimera
