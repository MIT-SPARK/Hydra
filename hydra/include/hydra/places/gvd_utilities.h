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
#include "hydra/places/gvd_integrator_config.h"
#include "hydra/places/gvd_voxel.h"
#include "hydra/places/voxblox_types.h"

namespace hydra {
namespace places {

struct DistancePotential {
  bool is_lower;
  double distance;
};

struct VoronoiCondition {
  bool neighbor_is_voronoi{false};
  bool current_is_voronoi{false};
};

DistancePotential getLowerDistance(FloatingPoint v_dist,
                                   FloatingPoint n_dist,
                                   FloatingPoint distance,
                                   FloatingPoint min_diff_m = 0.0f);

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

}  // namespace places
}  // namespace hydra
