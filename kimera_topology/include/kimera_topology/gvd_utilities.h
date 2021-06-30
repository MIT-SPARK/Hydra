#pragma once
#include "kimera_topology/gvd_voxel.h"
#include "kimera_topology/voxblox_types.h"

namespace kimera {
namespace topology {

inline void setGvdParent(GvdVoxel& voxel,
                         const GvdVoxel& ancestor,
                         const GlobalIndex& ancestor_index) {
  voxel.has_parent = true;
  if (ancestor.has_parent) {
    std::memcpy(voxel.parent, ancestor.parent, sizeof(ancestor.parent));
  } else {
    Eigen::Map<GlobalIndex>(voxel.parent) = ancestor_index;
  }
}

inline void resetGvdParent(GvdVoxel& voxel) { voxel.has_parent = false; }

inline void setGvdSurfaceVoxel(GvdVoxel& voxel) {
  voxel.on_surface = true;
  resetGvdParent(voxel);
}

struct DistancePotential {
  bool is_lower;
  double distance;
};

DistancePotential getLowerDistance(FloatingPoint v_dist,
                                   FloatingPoint n_dist,
                                   FloatingPoint distance,
                                   FloatingPoint min_diff_m = 0.0f);

struct VoronoiCondition {
  bool neighbor_is_voronoi{false};
  bool current_is_voronoi{false};
};

VoronoiCondition checkVoronoi(const GvdVoxel& current,
                              const GlobalIndex& current_idx,
                              const GvdVoxel& neighbor,
                              const GlobalIndex& neighbor_idx,
                              double gvd_min_distance,
                              double parent_min_separation = 3.0);

}  // namespace topology
}  // namespace kimera
