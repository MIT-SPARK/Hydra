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

VoronoiCondition checkVoronoi(const VoronoiCheckConfig& cfg,
                              const GvdVoxel& current,
                              const GlobalIndex& current_idx,
                              const GvdVoxel& neighbor,
                              const GlobalIndex& neighbor_idx) {
  VoronoiCondition result;

  // if only one voxel fails this, we still want to reject the candidate
  // as a successful check for the passing voxel would mean that it would
  // be closer than the min distance to the other parent
  if (current.distance <= cfg.min_distance_m || neighbor.distance <= cfg.min_distance_m) {
    return result;
  }

  Eigen::Map<const GlobalIndex> neighbor_parent(neighbor.parent);
  Eigen::Map<const GlobalIndex> current_parent(current.parent);

  if (!isParentUnique(cfg, current_idx, current_parent, neighbor_parent)) {
    return result;
  }

  // Algorithm 4: 51-52 of Lau et al. 2013
  const GlobalIndex c_pn = current_idx - neighbor_parent;
  const GlobalIndex n_pc = neighbor_idx - current_parent;
  const GlobalIndex::Scalar dist_c_pn = c_pn.dot(c_pn);
  const GlobalIndex::Scalar dist_n_pc = n_pc.dot(n_pc);

  VLOG(10) << "  d_c_pn: " << dist_c_pn << " d_n_pc: " << dist_n_pc;
  result.current_is_voronoi = (dist_c_pn <= dist_n_pc);
  result.neighbor_is_voronoi = (dist_n_pc <= dist_c_pn);
  return result;
}

}  // namespace topology
}  // namespace kimera
