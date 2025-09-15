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
#include "hydra/places/traversability_layer.h"

namespace hydra::places {

bool TraversabilityVoxel::operator==(const TraversabilityVoxel& other) const {
  return traversability == other.traversability && confidence == other.confidence &&
         state == other.state;
}

TraversabilityBlock::TraversabilityBlock(float block_size,
                                         const BlockIndex& index,
                                         size_t voxels_per_side)
    : spatial_hash::Block(block_size, index), voxels_per_side(voxels_per_side) {
  voxels.resize(voxels_per_side * voxels_per_side);
}

TraversabilityBlock& TraversabilityBlock::operator=(const TraversabilityBlock& other) {
  if (this != &other) {
    spatial_hash::Block::operator=(other);
    voxels = other.voxels;
    const_cast<size_t&>(voxels_per_side) = other.voxels_per_side;
  }
  return *this;
}

TraversabilityBlock& TraversabilityBlock::operator=(TraversabilityBlock&& other) {
  if (this != &other) {
    spatial_hash::Block::operator=(std::move(other));
    voxels = std::move(other.voxels);
    const_cast<size_t&>(voxels_per_side) = other.voxels_per_side;
  }
  return *this;
}

TraversabilityVoxel& TraversabilityBlock::voxel(size_t x, size_t y) {
  return voxels[linearFromIndex(x, y)];
}

const TraversabilityVoxel& TraversabilityBlock::voxel(size_t x, size_t y) const {
  return voxels[linearFromIndex(x, y)];
}

TraversabilityVoxel& TraversabilityBlock::voxel(const Index2D& index) {
  return voxels[linearFromIndex(index.x(), index.y())];
}

const TraversabilityVoxel& TraversabilityBlock::voxel(const Index2D& index) const {
  return voxels[linearFromIndex(index.x(), index.y())];
}

void TraversabilityBlock::reset() {
  voxels = std::vector<TraversabilityVoxel>(voxels_per_side * voxels_per_side);
}

Index2D TraversabilityBlock::indexFromLinear(size_t linear_index) const {
  return {linear_index % voxels_per_side, linear_index / voxels_per_side};
}

size_t TraversabilityBlock::linearFromIndex(size_t x, size_t y) const {
  return x + y * voxels_per_side;
}

size_t TraversabilityBlock::linearFromIndex(const Index2D& index) const {
  return linearFromIndex(index.x(), index.y());
}

BlockIndex TraversabilityBlock::globalFromLocalIndex(const Index2D& local_index) const {
  return {index.x() * static_cast<int>(voxels_per_side) + local_index.x(),
          index.y() * static_cast<int>(voxels_per_side) + local_index.y(),
          index.z()};
}

Index2D TraversabilityBlock::localFromGlobalIndex(
    const BlockIndex& global_index) const {
  return {global_index.x() % voxels_per_side, global_index.y() % voxels_per_side};
}

bool TraversabilityBlock::isValidIndex(const Index2D& index) const {
  return index.x() >= 0 && index.x() < static_cast<int>(voxels_per_side) &&
         index.y() >= 0 && index.y() < static_cast<int>(voxels_per_side);
}

TraversabilityVoxel* TraversabilityLayer::voxel(const BlockIndex& global_index) {
  auto block = getBlockPtr(blockIndexFromGlobal(global_index));
  if (!block) {
    return nullptr;
  }
  return &block->voxel(voxelIndexFromGlobal(global_index));
}

const TraversabilityVoxel* TraversabilityLayer::voxel(
    const BlockIndex& global_index) const {
  const auto block = getBlockPtr(blockIndexFromGlobal(global_index));
  if (!block) {
    return nullptr;
  }
  return &block->voxel(voxelIndexFromGlobal(global_index));
}

BlockIndex TraversabilityLayer::blockIndexFromGlobal(
    const BlockIndex& global_index) const {
  return BlockIndex(global_index.x() / static_cast<int>(voxels_per_side),
                    global_index.y() / static_cast<int>(voxels_per_side),
                    global_index.z());
}

Index2D TraversabilityLayer::voxelIndexFromGlobal(
    const BlockIndex& global_index) const {
  return Index2D(global_index.x() % voxels_per_side,
                 global_index.y() % voxels_per_side);
}

}  // namespace hydra::places
