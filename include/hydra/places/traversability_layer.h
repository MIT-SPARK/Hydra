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

#include <spark_dsg/node_attributes.h>

#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "hydra/reconstruction/voxel_types.h"

namespace hydra::places {

struct TraversabilityVoxel {
  //! @brief The traversability value in the range [0, 1], where 0 means not traversable
  // and 1 means fully traversable.
  float traversability = 0.0f;

  //! @brief Confidence in the traversability value in [0, 1].
  float confidence = 0.0f;

  //! @brief Discrete traversability state for of the voxel, computed as a function of
  // traversability and confidence.
  spark_dsg::TraversabilityState state = spark_dsg::TraversabilityState::UNKNOWN;

  //! @brief Arbitrary debug value that can be set for viualization.
  // TODO(lschmid): Remove this at some point.
  mutable float debug_value = 0.0f;

  bool operator==(const TraversabilityVoxel& other) const {
    return traversability == other.traversability && confidence == other.confidence &&
           state == other.state;
  }
};

// Voxel indices as pairs of (x, y) coordinates.
using Index2D = Eigen::Vector2i;

struct Index2DHash {
  inline static const auto s = Index2D(1, 1290);
  int operator()(const Index2D& index) const { return index.dot(s); }
};
using Index2DSet = std::unordered_set<Index2D,
                                      Index2DHash,
                                      std::equal_to<Index2D>,
                                      Eigen::aligned_allocator<Index2D>>;

template <typename ValueT>
using Index2DMap =
    std::unordered_map<Index2D,
                       ValueT,
                       Index2DHash,
                       std::equal_to<Index2D>,
                       Eigen::aligned_allocator<std::pair<const Index2D, ValueT>>>;

// TODO(lschmid): Update SpatialHash to support 2D blocks.
template <typename VoxelT>
struct Block2D : spatial_hash::Block {
  Block2D(float block_size, const BlockIndex& index, size_t voxels_per_side)
      : spatial_hash::Block(block_size, index), voxels_per_side(voxels_per_side) {
    voxels.resize(voxels_per_side * voxels_per_side);
  }
  virtual ~Block2D() = default;

  Block2D& operator=(const Block2D& other) {
    if (this != &other) {
      spatial_hash::Block::operator=(other);
      voxels = other.voxels;
      const_cast<size_t&>(voxels_per_side) = other.voxels_per_side;
    }
    return *this;
  }
  Block2D& operator=(Block2D&& other) {
    if (this != &other) {
      spatial_hash::Block::operator=(std::move(other));
      voxels = std::move(other.voxels);
      const_cast<size_t&>(voxels_per_side) = other.voxels_per_side;
    }
    return *this;
  }
  Block2D(const Block2D& other) = default;
  Block2D(Block2D&& other) = default;

  // Accessors for voxels.
  VoxelT& voxel(size_t x, size_t y) { return voxels[linearFromIndex(x, y)]; }

  const VoxelT& voxel(size_t x, size_t y) const {
    return voxels[linearFromIndex(x, y)];
  }

  VoxelT& voxel(const Index2D& index) {
    return voxels[linearFromIndex(index.x(), index.y())];
  }

  const VoxelT& voxel(const Index2D& index) const {
    return voxels[linearFromIndex(index.x(), index.y())];
  }

  // Operations.
  void reset() { voxels = std::vector<VoxelT>(voxels_per_side * voxels_per_side); }

  // Indexing tools.
  Index2D indexFromLinear(size_t linear_index) const {
    return {linear_index % voxels_per_side, linear_index / voxels_per_side};
  }

  size_t linearFromIndex(size_t x, size_t y) const { return x + y * voxels_per_side; }
  size_t linearFromIndex(const Index2D& index) const {
    return linearFromIndex(index.x(), index.y());
  }
  BlockIndex globalFromLocalIndex(const Index2D& local_index) const {
    return {index.x() * static_cast<int>(voxels_per_side) + local_index.x(),
            index.y() * static_cast<int>(voxels_per_side) + local_index.y(),
            index.z()};
  }
  Index2D localFromGlobalIndex(const BlockIndex& global_index) const {
    return {global_index.x() % voxels_per_side, global_index.y() % voxels_per_side};
  }
  bool isValidIndex(const Index2D& index) const {
    return index.x() >= 0 && index.x() < static_cast<int>(voxels_per_side) &&
           index.y() >= 0 && index.y() < static_cast<int>(voxels_per_side);
  }

  // Data.
  const size_t voxels_per_side;
  std::vector<VoxelT> voxels;
};

template <typename VoxelT>
struct Layer2D : public spatial_hash::BlockLayer<Block2D<VoxelT>> {
  using Ptr = std::shared_ptr<Layer2D>;
  using ConstPtr = std::shared_ptr<const Layer2D>;

  Layer2D(float voxel_size, size_t voxels_per_side)
      : spatial_hash::BlockLayer<Block2D<VoxelT>>(voxel_size * voxels_per_side),
        voxels_per_side(voxels_per_side),
        voxel_size(voxel_size) {}
  virtual ~Layer2D() = default;

  // Indexing.
  BlockIndex blockIndexFromGlobal(const BlockIndex& global_index) const {
    return BlockIndex(global_index.x() / voxels_per_side,
                      global_index.y() / voxels_per_side,
                      global_index.z());
  }
  Index2D voxelIndexFromGlobal(const BlockIndex& global_index) const {
    return Index2D(global_index.x() % voxels_per_side,
                   global_index.y() % voxels_per_side);
  }
  BlockIndex globalIndexFromPoint(const Eigen::Vector3f& position) const {
    return BlockIndex(std::floor(position.x() / static_cast<float>(voxel_size)),
                      std::floor(position.y() / static_cast<float>(voxel_size)),
                      std::floor(position.z() / static_cast<float>(voxel_size)));
  }

  // Accessors.
  VoxelT* voxel(const BlockIndex& global_index) {
    auto block = this->getBlockPtr(blockIndexFromGlobal(global_index));
    if (!block) {
      return nullptr;
    }
    return &block->voxel(voxelIndexFromGlobal(global_index));
  }
  const VoxelT* voxel(const BlockIndex& global_index) const {
    return const_cast<Layer2D*>(this)->voxel(global_index);
  }

  // Data.
  const size_t voxels_per_side;
  const float voxel_size;
};

using TraversabilityBlock = Block2D<TraversabilityVoxel>;
using TraversabilityLayer = Layer2D<TraversabilityVoxel>;

}  // namespace hydra::places
