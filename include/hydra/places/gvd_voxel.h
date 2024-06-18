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
#include <iostream>
#include <optional>

#include "hydra/reconstruction/voxel_types.h"

namespace hydra::places {

// TODO(nathan) packed?
struct GvdVoxel {
  float distance;
  // TODO(nathan) consider bitset instead
  bool observed = false;
  bool fixed = false;
  bool in_queue = false;
  bool to_raise = false;
  bool is_negative = false;
  bool on_surface = false;
  bool has_parent = false;

  uint8_t num_extra_basis = 0;

  GlobalIndex parent;
  // required for removing blocks (parents leave a dangling reference otherwise)
  Point parent_pos;
};

using GvdBlock = spatial_hash::VoxelBlock<GvdVoxel>;
using GvdLayer = spatial_hash::VoxelLayer<GvdBlock>;

std::ostream& operator<<(std::ostream& out, const GvdVoxel& voxel);

inline void resetVoronoi(GvdVoxel& voxel) { voxel.num_extra_basis = 0; }

inline bool isVoronoi(const GvdVoxel& voxel) { return voxel.num_extra_basis != 0; }

inline void setSdfParent(GvdVoxel& voxel,
                         const GvdVoxel& ancestor,
                         const GlobalIndex& ancestor_index,
                         const Point& ancestor_pos) {
  voxel.has_parent = true;
  if (ancestor.has_parent) {
    voxel.parent = ancestor.parent;
    voxel.parent_pos = ancestor.parent_pos;
  } else {
    voxel.parent = ancestor_index;
    voxel.parent_pos = ancestor_pos;
  }
}

// TODO(nathan) should probably be resetSdfParent
inline void resetParent(GvdVoxel& voxel) { voxel.has_parent = false; }

inline void setDefaultDistance(GvdVoxel& voxel, const double default_distance) {
  // TODO(nathan) there's probably a better way to do this
  voxel.distance = std::copysign(default_distance, voxel.is_negative ? -1.0 : 1.0);
}

inline void setRaiseStatus(GvdVoxel& voxel, const double default_distance) {
  resetParent(voxel);
  voxel.to_raise = true;

  if (voxel.fixed) {
    return;
  }

  setDefaultDistance(voxel, default_distance);
}

inline void setGvdSurfaceVoxel(GvdVoxel& voxel) {
  voxel.on_surface = true;
  resetParent(voxel);
}


}  // namespace hydra::places
