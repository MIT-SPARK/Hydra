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

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>

#include "hydra/common/dsg_types.h"
#include "hydra/common/shared_dsg_info.h"

namespace kimera_pgmo {
class MeshDelta;
class SparseKeyframe;
}  // namespace kimera_pgmo

namespace hydra::utils {

using KeyMap = std::unordered_map<gtsam::Key, gtsam::Key>;
using FrameMap = std::unordered_map<gtsam::Key, kimera_pgmo::SparseKeyframe>;

std::optional<uint64_t> getTimeNs(const DynamicSceneGraph& graph, gtsam::Symbol key);

void updatePlace2dMesh(Place2dNodeAttributes& attrs,
                       const kimera_pgmo::MeshDelta& mesh_update,
                       const size_t num_archived_vertices);

void updatePlace2dBoundary(Place2dNodeAttributes& attrs,
                           const kimera_pgmo::MeshDelta& mesh_update);

void updatePlaces2d(SharedDsgInfo::Ptr dsg,
                    kimera_pgmo::MeshDelta& mesh_update,
                    size_t num_archived_vertices);

template <typename T>
void mergeIndices(const T& from, T& to) {
  std::vector<typename T::value_type> from_indices(from.begin(), from.end());
  std::vector<typename T::value_type> to_indices(to.begin(), to.end());
  to.clear();

  std::sort(from_indices.begin(), from_indices.end());
  std::sort(to_indices.begin(), to_indices.end());
  std::set_union(from_indices.begin(),
                 from_indices.end(),
                 to_indices.begin(),
                 to_indices.end(),
                 std::back_inserter(to));
}

gtsam::Values getDenseFrames(const KeyMap& full_sparse_frame_map,
                             const FrameMap& sparse_frames,
                             const gtsam::Values& sparse_values);

}  // namespace hydra::utils
