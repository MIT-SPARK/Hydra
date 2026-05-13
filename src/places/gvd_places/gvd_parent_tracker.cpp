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
#include "hydra/places/gvd_places/gvd_parent_tracker.h"

#include <glog/logging.h>

#include "hydra/places/gvd_places/gvd_utilities.h"

namespace hydra::places {
namespace {

const static Eigen::IOFormat fmt(
    Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "; ", "", "", "[", "]");

}

bool GvdParentTracker::add(const GvdVoxel& voxel, const GlobalIndex& index) {
  if (!voxel.has_parent) {
    return false;
  }

  bool is_new = false;
  auto iter = parents_.find(index);
  if (iter == parents_.end()) {
    iter = parents_.emplace(index, BasisPointSet{}).first;
    is_new = true;
  }

  iter->second.insert({voxel.parent, voxel.parent_pos});
  return is_new;
}

uint8_t GvdParentTracker::add_unique(const VoronoiCheckConfig& config,
                                     const GlobalIndex& voxel,
                                     const GvdVoxel& neighbor) {
  auto& curr_parents = parents_.at(voxel);

  uint8_t curr_extra_basis = curr_parents.size();
  for (const auto& other : curr_parents) {
    const bool is_unique = isParentUnique(config, voxel, other.index, neighbor.parent);
    if (!is_unique) {
      return curr_extra_basis;
    }
  }

  // parent is unique enough
  curr_parents.insert({neighbor.parent, neighbor.parent_pos});
  return curr_extra_basis + 1;
}

void GvdParentTracker::erase(const GlobalIndex& voxel_index) {
  parents_.erase(voxel_index);
}

std::vector<Point> GvdParentTracker::parents(const GlobalIndex& index) const {
  auto iter = parents_.find(index);
  if (iter == parents_.end()) {
    return {};
  }

  std::vector<Point> to_return;
  for (const auto& basis_point : iter->second) {
    to_return.push_back(basis_point.pos);
  }

  return to_return;
}

}  // namespace hydra::places
