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
#include "hydra/reconstruction/tsdf_interpolators.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>

namespace hydra {

Eigen::Matrix<int, 3, 8> getCubeOffsets() {
  Eigen::Matrix<int, 3, 8> offsets;
  for (size_t i = 0; i < 8; ++i) {
    // x: lsb, y: second lsb, z: third lsb
    offsets.col(i) << ((i & 0x01) > 0), ((i & 0x02) > 0), ((i & 0x04) > 0);
  }

  return offsets;
}

void declare_config(DownsampleTsdfInterpolator::Config& config) {
  using namespace config;
  name("DownsampleTsdfInterpolator::Config");
  field(config.ratio, "ratio");
  field(config.tolerance, "tolerance");
  checkCondition((config.ratio % 2 == 0), "Downsample ratio must be even");
  check(config.ratio, GT, 1, "ratio");
}

DownsampleTsdfInterpolator::DownsampleTsdfInterpolator(const Config& config)
    : TsdfInterpolator(), config(config::checkValid(config)) {}

TsdfLayer::Ptr DownsampleTsdfInterpolator::interpolate(
    const TsdfLayer& input, const BlockIndices* to_use) const {
  const auto new_vs = input.voxel_size * config.ratio;
  const auto new_vps = input.voxels_per_side / config.ratio;
  if (new_vps <= 1) {
    LOG(ERROR) << "ratio invalid for current number of voxels";
    return nullptr;
  }
  auto new_layer = std::make_shared<TsdfLayer>(new_vs, new_vps);

  const auto offsets = getCubeOffsets();
  VoxelIndex center_offset = VoxelIndex::Constant(config.ratio / 2);
  const BlockIndices blocks = to_use ? *to_use : input.allocatedBlockIndices();

  for (const auto& idx : blocks) {
    auto& new_block = new_layer->allocateBlock(idx);
    const auto& old_block = input.getBlock(idx);

    for (size_t i = 0; i < new_block.numVoxels(); ++i) {
      auto& new_voxel = new_block.getVoxel(i);

      // get top-level corner of 2x2x2 cube
      const auto new_idx = new_block.getVoxelIndex(i);
      const VoxelIndex old_idx = config.ratio * new_idx + center_offset;

      // float required to avoid overflow
      Eigen::Vector4f new_color = Eigen::Vector4f::Zero();
      for (size_t i = 0; i < 8; ++i) {
        const auto& voxel = old_block.getVoxel(old_idx - offsets.col(i));

        new_voxel.weight += voxel.weight;
        new_voxel.distance += voxel.weight * voxel.distance;
        new_color(0, 0) += voxel.weight * voxel.color.r;
        new_color(1, 0) += voxel.weight * voxel.color.g;
        new_color(2, 0) += voxel.weight * voxel.color.b;
        new_color(3, 0) += voxel.weight * voxel.color.a;
      }

      if (new_voxel.weight < config.tolerance) {
        continue;  // all voxels are unobserved, skip distance and color computation
      }

      // finalize the weight average
      new_voxel.color.r = std::round(new_color(0) / new_voxel.weight);
      new_voxel.color.g = std::round(new_color(1) / new_voxel.weight);
      new_voxel.color.b = std::round(new_color(2) / new_voxel.weight);
      new_voxel.color.a = std::round(new_color(3) / new_voxel.weight);
      new_voxel.distance /= new_voxel.weight;
      new_voxel.weight /= 8;
    }
  }

  return new_layer;
}

}  // namespace hydra
