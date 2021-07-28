#pragma once
#include "kimera_topology/gvd_voxel.h"
#include "kimera_topology/voxblox_types.h"

namespace kimera {
namespace topology {

inline void resetVoronoi(GvdVoxel& voxel) { voxel.num_extra_basis = 0; }

inline bool isVoronoi(const GvdVoxel& voxel) { return voxel.num_extra_basis != 0; }

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

enum class ParentUniquenessMode {
  ANGLE,
  L1_DISTANCE,
  L1_THEN_ANGLE,
};

struct VoronoiCheckConfig {
  ParentUniquenessMode mode = ParentUniquenessMode::L1_THEN_ANGLE;
  double min_distance_m = 0.2;
  double parent_l1_separation = 3.0;
  double parent_cos_angle_separation = 0.5;
};

VoronoiCondition checkVoronoi(const VoronoiCheckConfig& config,
                              const GvdVoxel& current,
                              const GlobalIndex& current_idx,
                              const GvdVoxel& neighbor,
                              const GlobalIndex& neighbor_idx);

template <typename ParentIndex>
inline bool isParentUniqueL1(const VoronoiCheckConfig& config,
                             const GlobalIndex& /* current_index */,
                             const ParentIndex& current_parent,
                             const ParentIndex& neighbor_parent) {
  const GlobalIndex parent_diff = current_parent - neighbor_parent;
  return parent_diff.lpNorm<1>() > config.parent_l1_separation;
}

template <typename ParentIndex>
inline bool isParentUniqueAngle(const VoronoiCheckConfig& config,
                                const GlobalIndex& current_index,
                                const ParentIndex& current_parent,
                                const ParentIndex& neighbor_parent) {
  GlobalIndex curr_vec = (current_parent - current_index);
  GlobalIndex neighbor_vec = (neighbor_parent - current_index);
  double cos_angle =
      curr_vec.cast<float>().normalized().dot(neighbor_vec.cast<float>().normalized());
  return cos_angle < config.parent_cos_angle_separation;
}

template <typename ParentIndex>
inline bool isParentUnique(const VoronoiCheckConfig& config,
                           const GlobalIndex& current_idx,
                           const ParentIndex& current_parent,
                           const ParentIndex& neighbor_parent) {
  switch (config.mode) {
    case ParentUniquenessMode::ANGLE:
      return isParentUniqueAngle(config, current_idx, current_parent, neighbor_parent);
    case ParentUniquenessMode::L1_DISTANCE:
      return isParentUniqueL1(config, current_idx, current_parent, neighbor_parent);
    case ParentUniquenessMode::L1_THEN_ANGLE:
    default:
      return isParentUniqueL1(config, current_idx, current_parent, neighbor_parent) ||
             isParentUniqueAngle(config, current_idx, current_parent, neighbor_parent);
  }
}

}  // namespace topology
}  // namespace kimera
