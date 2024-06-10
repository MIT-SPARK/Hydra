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
#include <gtest/gtest.h>
#include <hydra/reconstruction/tsdf_interpolators.h>

#include <optional>
#include <set>

namespace hydra {

using voxblox::Layer;
using voxblox::TsdfVoxel;

namespace {

Layer<TsdfVoxel>::Ptr makeLayer(double resolution, int voxels_per_side, double weight) {
  auto layer = std::make_shared<Layer<TsdfVoxel>>(resolution, voxels_per_side);
  auto block = layer->allocateBlockPtrByIndex(voxblox::BlockIndex(0, 0, 0));
  for (size_t i = 0; i < block->num_voxels(); ++i) {
    auto& voxel = block->getVoxelByLinearIndex(i);
    voxel.weight = weight;
    voxel.distance = i;
    voxel.color.r = 1;
    voxel.color.g = 2;
    voxel.color.b = 3;
    voxel.color.a = 4;
  }

  return layer;
}

}  // namespace

TEST(TsdfInterpolators, TrilinearCorrect) {
  const auto tsdf = makeLayer(0.1, 4, 0.1);
  TrilinearTsdfInterpolator::Config config;
  config.voxels_per_side = 2;
  TrilinearTsdfInterpolator interp(config);
  const auto new_layer = interp.interpolate(*tsdf, nullptr);

  ASSERT_TRUE(new_layer != nullptr);
  const auto new_block = new_layer->getBlockPtrByIndex(voxblox::BlockIndex(0, 0, 0));
  ASSERT_TRUE(new_block != nullptr);

  const std::vector<double> expected{10.5, 12.5, 18.5, 20.5, 42.5, 44.5, 50.5, 52.5};
  ASSERT_EQ(expected.size(), new_block->num_voxels());
  for (size_t i = 0; i < new_block->num_voxels(); ++i) {
    const auto& voxel = new_block->getVoxelByLinearIndex(i);
    EXPECT_NEAR(voxel.weight, 0.1, 1.0e-6);
    EXPECT_NEAR(voxel.distance, expected.at(i), 1.0e-6);
  }
}

TEST(TsdfInterpolators, ResampleCorrect) {
  const auto tsdf = makeLayer(0.1, 4, 0.1);
  DownsampleTsdfInterpolator::Config config;
  config.ratio = 2;
  DownsampleTsdfInterpolator interp(config);
  const auto new_layer = interp.interpolate(*tsdf, nullptr);

  ASSERT_TRUE(new_layer != nullptr);
  const auto new_block = new_layer->getBlockPtrByIndex(voxblox::BlockIndex(0, 0, 0));
  ASSERT_TRUE(new_block != nullptr);

  const std::vector<double> expected{10.5, 12.5, 18.5, 20.5, 42.5, 44.5, 50.5, 52.5};
  ASSERT_EQ(expected.size(), new_block->num_voxels());
  for (size_t i = 0; i < new_block->num_voxels(); ++i) {
    const auto& voxel = new_block->getVoxelByLinearIndex(i);
    EXPECT_NEAR(voxel.weight, 0.1, 1.0e-6);
    EXPECT_NEAR(voxel.distance, expected.at(i), 1.0e-5);
  }
}

}  // namespace hydra
