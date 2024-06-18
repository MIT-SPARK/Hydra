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
#include "hydra_test/place_fixtures.h"

#include <hydra/places/gvd_utilities.h>

namespace hydra::places::test {

void updateGvd(GvdIntegrator& integrator,
               const VolumetricMap& map,
               bool clear_updated,
               bool use_all_blocks) {
  integrator.updateFromTsdf(0, map.getTsdfLayer(), clear_updated, use_all_blocks);
  integrator.updateGvd(0);
}

void SingleBlockTestFixture::setTsdfVoxel(
    int x, int y, int z, float distance, float weight) {
  CHECK_LT(x, voxels_per_side);
  CHECK_LT(y, voxels_per_side);
  CHECK_LT(z, voxels_per_side);

  VoxelIndex v_index;
  v_index << x, y, z;

  auto& voxel = tsdf_block->getVoxel(v_index);
  voxel.distance = distance;
  voxel.weight = weight;
}

const GvdVoxel& SingleBlockTestFixture::getGvdVoxel(int x, int y, int z) {
  CHECK_LT(x, voxels_per_side);
  CHECK_LT(y, voxels_per_side);
  CHECK_LT(z, voxels_per_side);

  VoxelIndex v_index;
  v_index << x, y, z;

  return gvd_block->getVoxel(v_index);
}

const TsdfVoxel& SingleBlockTestFixture::getTsdfVoxel(int x, int y, int z) {
  CHECK_LT(x, voxels_per_side);
  CHECK_LT(y, voxels_per_side);
  CHECK_LT(z, voxels_per_side);

  VoxelIndex v_index;
  v_index << x, y, z;

  return tsdf_block->getVoxel(v_index);
}

void SingleBlockTestFixture::SetUp() {
  gvd_config.min_distance_m = truncation_distance;
  gvd_config.max_distance_m = 10.0;

  VolumetricMap::Config map_config;
  map_config.voxel_size = voxel_size;
  map_config.voxels_per_side = voxels_per_side;
  map_config.truncation_distance = truncation_distance;
  map = std::make_unique<VolumetricMap>(map_config, false);
  gvd_layer.reset(new GvdLayer(voxel_size, voxels_per_side));

  BlockIndex block_index = BlockIndex::Zero();
  tsdf_block = map->getTsdfLayer().allocateBlockPtr(block_index);
  gvd_block = gvd_layer->allocateBlockPtr(block_index);
  tsdf_block->setUpdated();

  for (int x = 0; x < voxels_per_side; ++x) {
    for (int y = 0; y < voxels_per_side; ++y) {
      for (int z = 0; z < voxels_per_side; ++z) {
        const bool is_edge = (x == 0) || (y == 0) || (z == 0);
        setTsdfVoxel(x, y, z, is_edge ? -0.05 : truncation_distance);
      }
    }
  }
}

void SingleBlockExtractionTestFixture::SetUp() {
  SingleBlockTestFixture::SetUp();
  setBlockState();

  gvd_integrator.reset(new GvdIntegrator(gvd_config, gvd_layer, nullptr));
  gvd_integrator->updateFromTsdf(0, map->getTsdfLayer(), true);
  gvd_integrator->updateGvd(0);
}

void SingleBlockExtractionTestFixture::setBlockState() {}

void LargeSingleBlockTestFixture::SetUp() {
  voxels_per_side = 8;
  SingleBlockTestFixture::SetUp();
}

void TestFixture2d::setSurfaceVoxel(int x, int y) {
  VoxelIndex v_index;
  v_index << x, y, 0;

  auto& voxel = tsdf_block->getVoxel(v_index);
  voxel.distance = 0.0;
  voxel.weight = 1.0;
}

void TestFixture2d::setTsdfVoxel(int x, int y, float distance, float weight) {
  CHECK_LT(x, voxels_per_side);
  CHECK_LT(y, voxels_per_side);

  VoxelIndex v_index;
  v_index << x, y, 0;

  auto& voxel = tsdf_block->getVoxel(v_index);
  voxel.distance = distance;
  voxel.weight = weight;
}

const GvdVoxel& TestFixture2d::getGvdVoxel(int x, int y) {
  CHECK_LT(x, voxels_per_side);
  CHECK_LT(y, voxels_per_side);

  VoxelIndex v_index;
  v_index << x, y, 0;

  return gvd_block->getVoxel(v_index);
}

void TestFixture2d::SetUp() {
  tsdf_layer.reset(new TsdfLayer(voxel_size, voxels_per_side));
  gvd_layer.reset(new GvdLayer(voxel_size, voxels_per_side));

  BlockIndex block_index = BlockIndex::Zero();
  tsdf_block = tsdf_layer->allocateBlockPtr(block_index);
  gvd_block = gvd_layer->allocateBlockPtr(block_index);
  tsdf_block->setUpdated();

  for (int x = 0; x < voxels_per_side; ++x) {
    for (int y = 0; y < voxels_per_side; ++y) {
      for (int z = 1; z < voxels_per_side; ++z) {
        VoxelIndex v_index;
        v_index << x, y, 0;

        auto& voxel = tsdf_block->getVoxel(v_index);
        voxel.weight = 0.0;
      }
    }
  }
}

}  // namespace hydra::places::test
