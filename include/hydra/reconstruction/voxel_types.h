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

#include <spark_dsg/mesh.h>
#include <spatial_hash/voxel_layer.h>

#include <cstddef>
#include <cstdint>

#include "hydra/common/common_types.h"

namespace hydra {

// Geometry types.
using spatial_hash::Point;

// Mesh types.
using spark_dsg::Mesh;

// Index types.
using spatial_hash::BlockIndex;
using spatial_hash::GlobalIndex;
using spatial_hash::VoxelIndex;
using spatial_hash::VoxelKey;

// Index containers.
using spatial_hash::BlockIndices;
using spatial_hash::GlobalIndices;
using spatial_hash::VoxelIndices;
using spatial_hash::VoxelKeys;

// Index Sets.
using BlockIndexSet = spatial_hash::IndexSet;
using VoxelIndexSet = spatial_hash::IndexSet;
using GlobalIndexSet = spatial_hash::IndexSet;

// Index hash maps.
template <typename ValueT>
using VoxelIndexMap = spatial_hash::IndexHashMap<ValueT>;
template <typename ValueT>
using BlockIndexMap = spatial_hash::IndexHashMap<ValueT>;
template <typename ValueT>
using GlobalIndexMap = spatial_hash::IndexHashMap<ValueT>;

// Voxel types.
struct TsdfVoxel {
  float distance = 0.0f;
  float weight = 0.0f;
  Color color;
};

// Based on the semantic voxel from Kimera-Semantics
struct SemanticVoxel {
  //! Current MLE semantic label
  uint32_t semantic_label = 0;
  // TODO(nathan) top-K!
  //! Log-likelihood priors of each label
  Eigen::VectorXf semantic_likelihoods;
  //! Whether or not the voxel has been initialized
  bool empty = true;
};

// Voxel to track which parts of space are free with high confidence.
struct TrackingVoxel {
  // Time stamp [ns] when the voxel was last observed.
  TimeStamp last_observed;
  TimeStamp last_occupied;
  bool ever_free = false;
  bool active = false;
  bool to_remove = false;
};

// Block types.
struct TsdfBlock : public spatial_hash::VoxelBlock<TsdfVoxel> {
  using Ptr = std::shared_ptr<TsdfBlock>;
  using ConstPtr = std::shared_ptr<const TsdfBlock>;

  TsdfBlock(const float voxel_size,
            const float voxels_per_side,
            const BlockIndex& index)
      : spatial_hash::VoxelBlock<TsdfVoxel>(voxel_size, voxels_per_side, index) {}

  mutable bool esdf_updated = false;
  mutable bool mesh_updated = false;
  mutable bool tracking_updated = false;

  void setUpdated() const {
    updated = true;
    esdf_updated = true;
    mesh_updated = true;
    tracking_updated = true;
  }

  // Function to enable iterating over update blocks.
  static bool esdfUpdated(const TsdfBlock& block) { return block.esdf_updated; }
  static bool meshUpdated(const TsdfBlock& block) { return block.mesh_updated; }
  static bool trackingUpdated(const TsdfBlock& block) { return block.tracking_updated; }
};

struct MeshBlock : public Mesh, public spatial_hash::Block {
  using Ptr = std::shared_ptr<MeshBlock>;
  using ConstPtr = std::shared_ptr<const MeshBlock>;
  MeshBlock(const float block_size, const BlockIndex& index, bool has_labels = false)
      : Mesh(true, false, has_labels, false), spatial_hash::Block(block_size, index) {}
};

struct TrackingBlock : public spatial_hash::VoxelBlock<TrackingVoxel> {
  using Ptr = std::shared_ptr<TrackingBlock>;
  using ConstPtr = std::shared_ptr<const TrackingBlock>;
  TrackingBlock(const float voxel_size,
                const float voxels_per_side,
                const BlockIndex& index)
      : spatial_hash::VoxelBlock<TrackingVoxel>(voxel_size, voxels_per_side, index) {}
  bool has_active_data = false;
};

using SemanticBlock = spatial_hash::VoxelBlock<SemanticVoxel>;

// Layer types.
using TsdfLayer = spatial_hash::VoxelLayer<TsdfBlock>;
using SemanticLayer = spatial_hash::VoxelLayer<SemanticBlock>;
using MeshLayer = spatial_hash::BlockLayer<MeshBlock>;
using TrackingLayer = spatial_hash::VoxelLayer<TrackingBlock>;

}  // namespace hydra
