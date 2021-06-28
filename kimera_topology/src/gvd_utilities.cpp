#include "kimera_topology/gvd_utilities.h"

namespace kimera {
namespace topology {

DistancePotential getLowerDistance(FloatingPoint v_dist,
                                   FloatingPoint n_dist,
                                   FloatingPoint distance,
                                   FloatingPoint min_diff_m) {
  DistancePotential to_return;
  if (n_dist == 0.0) {
    // this is incorrect for when n and v are both 0.0, but we're not
    // going to run into that case
    to_return.distance = v_dist - std::copysign(distance, v_dist);
  } else {
    to_return.distance = v_dist + std::copysign(distance, n_dist);
  }

  if (std::signbit(v_dist) == std::signbit(n_dist)) {
    to_return.is_lower = std::abs(to_return.distance) + min_diff_m < std::abs(n_dist);
  } else {
    to_return.is_lower = std::abs(to_return.distance - n_dist) > distance;
  }

  if (to_return.is_lower and std::signbit(to_return.distance) != std::signbit(n_dist)) {
    // TODO(nathan) this is weird, but clip to boundary?
    to_return.distance = (n_dist == 0.0 ? 0.0 : std::copysign(distance, n_dist));
  }

  return to_return;
}

VoronoiCondition checkVoronoi(const GvdVoxel& current,
                              const GlobalIndex& current_idx,
                              const GvdVoxel& neighbor,
                              const GlobalIndex& neighbor_idx,
                              double gvd_min_distance,
                              double parent_min_separation) {
  Eigen::Map<const GlobalIndex> neighbor_parent(neighbor.parent);
  Eigen::Map<const GlobalIndex> current_parent(current.parent);

  VoronoiCondition voxel_conditions;
  if (neighbor_parent == current_parent) {
    return voxel_conditions;
  }

  if ((neighbor_parent - current_parent).lpNorm<1>() < parent_min_separation) {
    return voxel_conditions;
  }
  // TODO(nathan) parent adjacency checks

  // Algorithm 4: 51-52 of Lau et al. 2013
  double dist_c_pn = (current_idx - neighbor_parent).norm();
  double dist_n_pc = (neighbor_idx - current_parent).norm();

  VLOG(5) << "  d_c_pn: " << dist_c_pn << " d_n_pc: " << dist_n_pc;
  // rather than stopping the check early, we use the fixed flag to determine if
  // the voronoi condition is truly met for a voxel
  voxel_conditions.current_is_voronoi = (dist_c_pn <= dist_n_pc) && !current.fixed;
  voxel_conditions.neighbor_is_voronoi = (dist_n_pc <= dist_c_pn) && !neighbor.fixed;
  return voxel_conditions;
}

}  // namespace topology
}  // namespace kimera
