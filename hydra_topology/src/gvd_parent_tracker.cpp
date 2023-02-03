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
#include "hydra_topology/gvd_parent_tracker.h"
#include "hydra_topology/gvd_utilities.h"

namespace hydra {
namespace topology {

uint8_t GvdParentTracker::updateGvdParentMap(const Layer<GvdVoxel>& layer,
                                             const VoronoiCheckConfig& config,
                                             const GlobalIndex& voxel_index,
                                             const GvdVoxel& neighbor) {
  const GlobalIndex neighbor_parent = Eigen::Map<const GlobalIndex>(neighbor.parent);
  if (!parents.count(voxel_index)) {
    parents[voxel_index] = voxblox::LongIndexSet();
  }

  uint8_t curr_extra_basis = parents[voxel_index].size();
  for (const auto& other_parent : parents[voxel_index]) {
    const bool is_unique =
        isParentUnique(config, voxel_index, other_parent, neighbor_parent);
    if (!is_unique) {
      return curr_extra_basis;
    }
  }

  // parent is unique enough
  parents[voxel_index].insert(neighbor_parent);
  markNewGvdParent(layer, neighbor_parent);
  return curr_extra_basis + 1;
}

void GvdParentTracker::markNewGvdParent(const Layer<GvdVoxel>& layer,
                                        const GlobalIndex& parent) {
  if (parent_vertices.count(parent)) {
    // make sure the parent vertex map stays alive for this gvd member
    parent_vertices[parent].ref_count++;
    return;
  }

  const GvdVoxel* parent_voxel = layer.getVoxelPtrByGlobalIndex(parent);
  if (!parent_voxel || !parent_voxel->on_surface) {
    // we can't do anything for parents that have left the active mesh before being used
    // as a GVD parent, or parents that aren't registered to the mesh
    return;
  }

  GvdVertexInfo info;
  info.ref_count = 1;
  info.vertex = parent_voxel->block_vertex_index;
  std::memcpy(info.block, parent_voxel->mesh_block, sizeof(info.block));
  std::memcpy(info.pos, parent_voxel->parent_pos, sizeof(info.pos));

  parent_vertices[parent] = info;
}

void GvdParentTracker::removeVoronoiFromGvdParentMap(const GlobalIndex& voxel_index) {
  auto voxel_parents = parents.find(voxel_index);
  if (voxel_parents != parents.end()) {
    for (const auto& parent : voxel_parents->second) {
      if (parent_vertices.count(parent)) {
        // decrement the ref count (we garbage collect later to avoid losing parents
        // due to thrashing)
        parent_vertices[parent].ref_count--;
      }
    }

    parents.erase(voxel_parents);
  }
}

void GvdParentTracker::updateVertexMapping(const Layer<GvdVoxel>& layer) {
  auto iter = parent_vertices.begin();
  while (iter != parent_vertices.end()) {
    if (!iter->second.ref_count) {
      iter = parent_vertices.erase(iter);
      continue;
    }

    const GvdVoxel* voxel = layer.getVoxelPtrByGlobalIndex(iter->first);
    if (!voxel) {
      ++iter;
      continue;
    }

    if (!voxel->on_surface) {
      iter = parent_vertices.erase(iter);
      continue;
    }

    iter->second.vertex = voxel->block_vertex_index;
    std::memcpy(iter->second.block, voxel->mesh_block, sizeof(iter->second.block));
    std::memcpy(iter->second.pos, voxel->parent_pos, sizeof(iter->second.pos));
    ++iter;
  }
}

}  // namespace topology
}  // namespace hydra
