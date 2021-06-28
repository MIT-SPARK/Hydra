#pragma once
#include "kimera_topology/voxblox_types.h"

namespace kimera {
namespace topology {

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
                              double parent_min_separation = 1.0);

}  // namespace topology
}  // namespace kimera
