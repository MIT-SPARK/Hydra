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
#include "hydra/reconstruction/volumetric_map.h"

#include "hydra/utils/display_utilities.h"

namespace hydra {

void declare_config(VolumetricMap::Config& config) {
  using namespace config;
  name("VolumetricMap");
  field(config.voxel_size, "voxel_size", "m");
  field(config.voxels_per_side, "voxels_per_side");
  field(config.truncation_distance, "truncation_distance", "m");

  check(config.voxel_size, GT, 0, "voxel_size");
  check(config.voxels_per_side, GT, 0, "voxels_per_side");
  check(config.truncation_distance, GT, 0, "truncation_distance");
}

VolumetricMap::VolumetricMap(const Config& _config,
                             bool with_semantics,
                             bool with_occupancy)
    : config(config::checkValid(_config)),
      tsdf_layer_(config.voxel_size, config.voxels_per_side),
      mesh_layer_(tsdf_layer_.block_size()),
      block_size(tsdf_layer_.block_size()),
      voxel_size_inv(tsdf_layer_.voxel_size_inv()) {
  if (with_semantics) {
    semantic_layer_.reset(new SemanticLayer(config.voxel_size, config.voxels_per_side));
  }

  if (with_occupancy) {
    occupancy_layer_.reset(
        new OccupancyLayer(config.voxel_size, config.voxels_per_side));
  }
}

void VolumetricMap::removeBlock(const voxblox::BlockIndex& block_index) {
  tsdf_layer_.removeBlock(block_index);
  mesh_layer_.removeBlock(block_index);
  if (semantic_layer_) {
    semantic_layer_->removeBlock(block_index);
  }

  if (occupancy_layer_) {
    occupancy_layer_->removeBlock(block_index);
  }
}

std::unique_ptr<VolumetricMap> VolumetricMap::fromTsdf(const TsdfLayer& tsdf,
                                                       double truncation_distance_m,
                                                       bool with_semantics,
                                                       bool with_occupancy) {
  VolumetricMap::Config config;
  config.voxel_size = tsdf.voxel_size();
  config.voxels_per_side = tsdf.voxels_per_side();
  config.truncation_distance = truncation_distance_m;
  auto to_return =
      std::make_unique<VolumetricMap>(config, with_semantics, with_occupancy);
  mergeLayer(tsdf, to_return->getTsdfLayer());
  return to_return;
}

std::string VolumetricMap::printStats() const {
  std::stringstream ss;
  const auto tsdf_size = tsdf_layer_.getMemorySize();
  ss << "TSDF: " << getHumanReadableMemoryString(tsdf_size);

  size_t semantic_size = 0;
  if (semantic_layer_) {
    semantic_size = semantic_layer_->getMemorySize();
    ss << ", SEMANTICS: " << getHumanReadableMemoryString(semantic_size);
  }

  size_t occ_size = 0;
  if (occupancy_layer_) {
    occ_size = occupancy_layer_->getMemorySize();
    ss << ", OCCUPANCY: " << getHumanReadableMemoryString(occ_size);
  }

  const auto mesh_size = mesh_layer_.getMemorySize();
  ss << ", MESH: " << getHumanReadableMemoryString(mesh_size);

  const auto total_size = tsdf_size + semantic_size + occ_size + mesh_size;
  ss << ", TOTAL: " << getHumanReadableMemoryString(total_size);

  return ss.str();
}

}  // namespace hydra
