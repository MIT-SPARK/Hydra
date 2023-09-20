// The contents of this file are originally from Panoptic-Mapping,
// under the following license:
//
// BSD 3-Clause License
// Copyright (c) 2021, ETHZ ASL
// All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// See https://github.com/ethz-asl/panoptic_mapping for original code and paper
//
// Modifications (including work done by Lukas Schmid for Khronos) fall under the same
// license as Hydra and are subject to the following copyright and disclaimer:
//
// Copyright 2022 Massachusetts Institute of Technology.
// All Rights Reserved
//
// Research was sponsored by the United States Air Force Research Laboratory and
// the United States Air Force Artificial Intelligence Accelerator and was
// accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
// and conclusions contained in this document are those of the authors and should
// not be interpreted as representing the official policies, either expressed or
// implied, of the United States Air Force or the U.S. Government. The U.S.
// Government is authorized to reproduce and distribute reprints for Government
// purposes notwithstanding any copyright notation herein.
#pragma once

#include <config_utilities/config_utilities.h>
#include <voxblox/core/layer.h>

#include "hydra/reconstruction/semantic_mesh_layer.h"
#include "hydra/reconstruction/semantic_voxel.h"
#include "hydra/reconstruction/vertex_voxel.h"

namespace hydra {

// TODO(nathan) clean this up
template <typename Voxel>
void mergeLayer(const voxblox::Layer<Voxel>& layer_in,
                voxblox::Layer<Voxel>& layer_out,
                bool overwrite_updated = false) {
  voxblox::BlockIndexList blocks;
  layer_in.getAllAllocatedBlocks(&blocks);
  for (const auto& idx : blocks) {
    auto block = layer_in.getBlockPtrByIndex(idx);
    if (!block) {
      continue;
    }

    auto new_block = layer_out.allocateBlockPtrByIndex(idx);
    for (size_t i = 0; i < block->num_voxels(); ++i) {
      new_block->getVoxelByLinearIndex(i) = block->getVoxelByLinearIndex(i);
    }

    if (overwrite_updated) {
      new_block->updated() = block->updated();
    } else {
      new_block->updated() |= block->updated();
    }

    new_block->has_data() = block->has_data();
    // TODO(nathan) copy other block attributes...
  }
}

template <typename Voxel>
void mergeLayer(const voxblox::Layer<Voxel>& layer_in,
                typename voxblox::Layer<Voxel>::Ptr& layer_out,
                bool overwrite_updated = false) {
  if (!layer_out) {
    layer_out.reset(
        new voxblox::Layer<Voxel>(layer_in.voxel_size(), layer_in.voxels_per_side()));
  }
  mergeLayer(layer_in, *layer_out, overwrite_updated);
}

class VolumetricMap {
 public:
  using TsdfLayer = voxblox::Layer<voxblox::TsdfVoxel>;
  using SemanticLayer = voxblox::Layer<SemanticVoxel>;
  using OccupancyLayer = voxblox::Layer<VertexVoxel>;

  struct Config {
    /// Voxel size.
    float voxel_size = 0.1f;
    /// Number of voxels per block side.
    int voxels_per_side = 16;  // TODO(nathan) fix int
    /// TSDF truncation distance.
    float truncation_distance = 0.3f;
  };

  explicit VolumetricMap(const Config& config,
                         bool with_semantics = false,
                         bool with_occupancy = false);

  virtual ~VolumetricMap() = default;

  TsdfLayer& getTsdfLayer() { return tsdf_layer_; }

  const TsdfLayer& getTsdfLayer() const { return tsdf_layer_; }

  SemanticMeshLayer& getMeshLayer() { return mesh_layer_; }

  const SemanticMeshLayer& getMeshLayer() const { return mesh_layer_; }

  SemanticLayer* getSemanticLayer() { return semantic_layer_.get(); }

  const SemanticLayer* getSemanticLayer() const { return semantic_layer_.get(); }

  bool hasSemantics() const { return semantic_layer_ != nullptr; }

  OccupancyLayer* getOccupancyLayer() { return occupancy_layer_.get(); }

  const OccupancyLayer* getOccupancyLayer() const { return occupancy_layer_.get(); }

  bool hasOccupancy() const { return occupancy_layer_ != nullptr; }

  // TODO(nathan) drop these for const members or direct config access?
  float voxel_size() const { return config.voxel_size; }

  float truncation_distance() const { return config.truncation_distance; }

  int voxels_per_side() const { return config.voxels_per_side; }

  virtual void removeBlock(const voxblox::BlockIndex& block);

  template <typename BlockIndices>
  void removeBlocks(const BlockIndices& blocks) {
    for (const auto& idx : blocks) {
      removeBlock(idx);
    }
  }

  virtual std::string printStats() const;

  void save(const std::string& filepath) const;

  static std::unique_ptr<VolumetricMap> fromTsdf(const TsdfLayer& tsdf,
                                                 double truncation_distance_m,
                                                 bool with_semantics = false,
                                                 bool with_occupancy = false);

  static std::unique_ptr<VolumetricMap> load(const std::string& filepath);

 public:
  const Config config;

 private:
  TsdfLayer tsdf_layer_;
  SemanticMeshLayer mesh_layer_;
  SemanticLayer::Ptr semantic_layer_;
  OccupancyLayer::Ptr occupancy_layer_;

 public:
  const float block_size;
  const float voxel_size_inv;
};

void declare_config(VolumetricMap::Config& config);

}  // namespace hydra
