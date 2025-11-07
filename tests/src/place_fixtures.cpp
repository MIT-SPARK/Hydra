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
namespace {

VoxelIndex validateCoords(const GvdIntegratorData::Coords& coords,
                          size_t voxels_per_side) {
  CHECK_LT(coords[0], voxels_per_side);
  CHECK_LT(coords[1], voxels_per_side);
  CHECK_LT(coords[2], voxels_per_side);

  VoxelIndex v_index;
  v_index << coords[0], coords[1], coords[2];
  return v_index;
}

}  // namespace

void updateGvd(GvdIntegrator& integrator,
               const VolumetricMap& map,
               bool clear_updated,
               bool use_all_blocks) {
  integrator.updateFromTsdf(
      0, map.getTsdfLayer(), map.getMeshLayer(), clear_updated, use_all_blocks);
  integrator.updateGvd(0);
}

GvdIntegratorData::GvdIntegratorData(float voxel_size,
                                     size_t voxels_per_side,
                                     float truncation_distance)
    : map_config({voxel_size, voxels_per_side, truncation_distance}), map(map_config) {
  gvd_config.min_distance_m = truncation_distance;
  gvd_config.max_distance_m = 10.0;
  gvd_layer.reset(new GvdLayer(voxel_size, voxels_per_side));

  BlockIndex block_index = BlockIndex::Zero();
  tsdf_block = map.getTsdfLayer().allocateBlockPtr(block_index);
  mesh_block = map.getMeshLayer().allocateBlockPtr(block_index);
  gvd_block = gvd_layer->allocateBlockPtr(block_index);
  tsdf_block->setUpdated();
}

void GvdIntegratorData::setup(const ObservationCallback& callback) {
  for (size_t x = 0; x < map_config.voxels_per_side; ++x) {
    for (size_t y = 0; y < map_config.voxels_per_side; ++y) {
      for (size_t z = 0; z < map_config.voxels_per_side; ++z) {
        const Coords coords{x, y, z};
        const auto obs = callback(coords, map_config);
        if (!obs) {
          continue;
        }

        setTsdf(coords, obs->distance, obs->weight);
      }
    }
  }
}

void GvdIntegratorData::check(const CheckCallback& callback) {
  for (size_t x = 0; x < map_config.voxels_per_side; ++x) {
    for (size_t y = 0; y < map_config.voxels_per_side; ++y) {
      for (size_t z = 0; z < map_config.voxels_per_side; ++z) {
        const Coords coords{x, y, z};
        callback(coords, getGvd(coords), map_config);
      }
    }
  }
}

void GvdIntegratorData::setTsdf(size_t x, size_t y, float distance, float weight) {
  setTsdf({x, y, 0}, distance, weight);
}

void GvdIntegratorData::setTsdf(const Coords& coords, float distance, float weight) {
  const auto v_index = validateCoords(coords, map_config.voxels_per_side);
  auto& voxel = tsdf_block->getVoxel(v_index);
  voxel.distance = distance;
  voxel.weight = weight;
  if (distance < 0.0f && weight > 0.0f) {
    const auto pos = tsdf_block->getVoxelPosition(v_index);
    mesh_block->points.push_back(pos);
  }
}

const GvdVoxel& GvdIntegratorData::getGvd(const Coords& coords) const {
  const auto v_index = validateCoords(coords, map_config.voxels_per_side);
  return gvd_block->getVoxel(v_index);
}

const TsdfVoxel& GvdIntegratorData::getTsdf(const Coords& coords) const {
  const auto v_index = validateCoords(coords, map_config.voxels_per_side);
  return tsdf_block->getVoxel(v_index);
}

}  // namespace hydra::places::test
