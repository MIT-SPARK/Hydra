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
#include "hydra/places/gvd_utilities.h"

namespace hydra::places {

DistancePotential getLowerDistance(float v_dist,
                                   float n_dist,
                                   float distance,
                                   float min_diff_m) {
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
  if (current.distance <= cfg.min_distance_m ||
      neighbor.distance <= cfg.min_distance_m) {
    return result;
  }

  if (!isParentUnique(cfg, current_idx, current.parent, neighbor.parent)) {
    return result;
  }

  // Algorithm 4: 51-52 of Lau et al. 2013
  const GlobalIndex c_pn = current_idx - neighbor.parent;
  const GlobalIndex n_pc = neighbor_idx - current.parent;
  const GlobalIndex::Scalar dist_c_pn = c_pn.dot(c_pn);
  const GlobalIndex::Scalar dist_n_pc = n_pc.dot(n_pc);

  VLOG(15) << "  d_c_pn: " << dist_c_pn << " d_n_pc: " << dist_n_pc;
  result.current_is_voronoi = (dist_c_pn <= dist_n_pc);
  result.neighbor_is_voronoi = (dist_n_pc <= dist_c_pn);
  return result;
}

bool isParentUniqueL1(const VoronoiCheckConfig& config,
                      const GlobalIndex& /* current_index */,
                      const GlobalIndex& current_parent,
                      const GlobalIndex& neighbor_parent) {
  const GlobalIndex parent_diff = current_parent - neighbor_parent;
  return parent_diff.lpNorm<1>() > config.parent_l1_separation;
}

bool isParentUniqueAngle(const VoronoiCheckConfig& config,
                         const GlobalIndex& current_index,
                         const GlobalIndex& current_parent,
                         const GlobalIndex& neighbor_parent) {
  GlobalIndex curr_vec = (current_parent - current_index);
  GlobalIndex neighbor_vec = (neighbor_parent - current_index);
  double cos_angle =
      curr_vec.cast<float>().normalized().dot(neighbor_vec.cast<float>().normalized());
  return cos_angle < config.parent_cos_angle_separation;
}

bool isParentUnique(const VoronoiCheckConfig& config,
                    const GlobalIndex& current_idx,
                    const GlobalIndex& current_parent,
                    const GlobalIndex& neighbor_parent) {
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

}  // namespace hydra::places
