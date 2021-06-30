#pragma once
#include "kimera_topology/voxblox_types.h"

#include <iostream>

namespace kimera {
namespace topology {

// TODO(nathan) packed?
// TODO(nathan) consider parent pointers
struct GvdVoxel {
  float distance;
  bool observed = false;
  bool on_surface = false;
  bool fixed = false;
  bool in_queue = false;

  bool has_parent = false;
  GlobalIndex::Scalar parent[3];

  bool is_voronoi = false;
  bool is_voronoi_parent = false;
  uint8_t num_basis = 0;
  GlobalIndex::Scalar nearest_voronoi[3];
  float nearest_voronoi_distance;
};

std::ostream& operator<<(std::ostream& out, const GvdVoxel& voxel);

}  // namespace topology
}  // namespace kimera
