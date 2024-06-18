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

#include "hydra/utils/pgmo_mesh_interface.h"

namespace hydra {

PgmoMeshInterface::PgmoMeshInterface(const MeshLayer& mesh) : mesh_(mesh) {
  block_indices.reserve(mesh.numBlocks());
  // TODO(lschmid): Ugly hard copy for now.
  for (const auto& block_idx : mesh.allocatedBlockIndices()) {
    block_indices.push_back(block_idx.cast<voxblox::BlockIndex::Scalar>());
  }
}

const voxblox::BlockIndexList& PgmoMeshInterface::blockIndices() const {
  return block_indices;
}

void PgmoMeshInterface::markBlockActive(const voxblox::BlockIndex& block) {
  active_mesh_ = mesh_.getBlockPtr(block.cast<BlockIndex::Scalar>());
}

size_t PgmoMeshInterface::activeBlockSize() const {
  // Assumes we mark the active block first.
  return active_mesh_->points.size();
}

pcl::PointXYZRGBA PgmoMeshInterface::getActiveVertex(size_t index) const {
  // Assumes we mark the active block first.
  pcl::PointXYZRGBA point;
  const auto& pos = active_mesh_->points[index];
  point.x = pos(0);
  point.y = pos(1);
  point.z = pos(2);
  const auto& color = active_mesh_->colors[index];
  point.r = color.r;
  point.g = color.g;
  point.b = color.b;
  point.a = color.a;
  return point;
};

std::optional<uint32_t> PgmoMeshInterface::getActiveSemantics(size_t index) const {
  // Assumes we mark the active block first.
  return active_mesh_->labels[index];
}

}  // namespace hydra
