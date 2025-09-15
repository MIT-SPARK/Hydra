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
#include "hydra/places/traversability_estimator.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>

namespace hydra::places {

using spark_dsg::TraversabilityState;

void declare_config(HeightTraversabilityEstimator::Config& config) {
  using namespace config;
  name("HeightTraversabilityEstimator::Config");
  field(config.height_above, "height_above", "m");
  field(config.height_below, "height_below", "m");
  field(config.min_confidence, "min_confidence");
  field(config.min_traversability, "min_traversability");
  field(config.pessimistic, "pessimistic");
  checkCondition(config.height_above >= -config.height_below,
                 "'height_above' and 'height_below' don't span any volume");
  checkInRange(config.min_confidence, 0.0f, 1.0f, "min_confidence");
  checkInRange(config.min_traversability, 0.0f, 1.0f, "min_traversability");
}

HeightTraversabilityEstimator::HeightTraversabilityEstimator(const Config& config)
    : config(config::checkValid(config)) {}

void HeightTraversabilityEstimator::updateTraversability(
    const ActiveWindowOutput& msg,
    const kimera_pgmo::MeshDelta&,
    const spark_dsg::DynamicSceneGraph&) {
  updateTsdf(msg);
  computeTraversability(msg);
}

void HeightTraversabilityEstimator::updateTsdf(const ActiveWindowOutput& msg) {
  // Initialize the TSDF if this is the first call.
  if (!tsdf_layer_) {
    const auto& map_config = msg.map().config;
    tsdf_layer_ =
        std::make_shared<TsdfLayer>(map_config.voxel_size, map_config.voxels_per_side);
    traversability_layer_ = std::make_unique<TraversabilityLayer>(
        map_config.voxel_size, map_config.voxels_per_side);
  }

  // Erase archived blocks.
  tsdf_layer_->removeBlocks(msg.archived_mesh_indices);
  const BlockIndexSet blocks_2d =
      get2DBlockIndices(tsdf_layer_->allocatedBlockIndices());
  for (const auto& block_index : traversability_layer_->allocatedBlockIndices()) {
    if (!blocks_2d.count(block_index)) {
      traversability_layer_->removeBlock(block_index);
    }
  }

  // Copy in all updated blocks. These are mutually exclusive with the updated blocks.
  for (const auto& block : msg.map().getTsdfLayer()) {
    tsdf_layer_->allocateBlock(block.index) = block;
    if (!block.updated) {
      LOG(WARNING) << "TSDF block " << block.index.transpose()
                   << " is not marked as updated, but it was copied from the map.";
    }
  }

  // Reset the updated flag.
  for (auto& block : *traversability_layer_) {
    block.updated = false;
  }
}

void HeightTraversabilityEstimator::computeTraversability(
    const ActiveWindowOutput& msg) {
  // Naive implementation: simply recompute the traversability from scratch.
  const BlockIndexSet updated_blocks_2d =
      get2DBlockIndices(msg.map().getTsdfLayer().allocatedBlockIndices());

  const VoxelKey min_height =
      tsdf_layer_->getVoxelKey(Point(0, 0, msg.world_t_body.z() - config.height_below));
  const VoxelKey max_height =
      tsdf_layer_->getVoxelKey(Point(0, 0, msg.world_t_body.z() + config.height_above));
  const int voxels_per_side = tsdf_layer_->voxels_per_side;
  const float voxel_size = tsdf_layer_->voxel_size;
  const int num_voxels =
      spatial_hash::globalIndexFromKey(max_height, voxels_per_side).z() -
      spatial_hash::globalIndexFromKey(min_height, voxels_per_side).z() + 1;

  // Iterate over all 2D blocks.
  for (auto& block_idx_2d : updated_blocks_2d) {
    // Reset the traversability.
    auto& traversability_block =
        traversability_layer_->allocateBlock(block_idx_2d, voxels_per_side);
    traversability_block.reset();
    traversability_block.updated = true;
    for (int block_z = min_height.first.z(); block_z <= max_height.first.z();
         ++block_z) {
      const auto tsdf_index = BlockIndex(block_idx_2d.x(), block_idx_2d.y(), block_z);
      const auto tsdf_block = tsdf_layer_->getBlockPtr(tsdf_index);
      if (!tsdf_block) {
        continue;
      }
      const int min_voxel_z =
          block_z == min_height.first.z() ? min_height.second.z() : 0;
      const int max_voxel_z =
          block_z == max_height.first.z() ? max_height.second.z() : voxels_per_side - 1;
      for (int x = 0; x < voxels_per_side; ++x) {
        for (int y = 0; y < voxels_per_side; ++y) {
          auto& traversability_voxel = traversability_block.voxel(x, y);
          for (int z = min_voxel_z; z <= max_voxel_z; ++z) {
            const auto& tsdf_voxel = tsdf_block->getVoxel(VoxelIndex(x, y, z));
            // Count number of observed and free voxels.
            if (tsdf_voxel.weight < 1e-6f) {
              continue;
            }
            traversability_voxel.confidence += 1.0f;
            if (tsdf_voxel.distance >= voxel_size) {
              traversability_voxel.traversability += 1.0f;
            }
          }
        }
      }
    }

    // Normalize the traversability values.
    for (auto& voxel : traversability_block.voxels) {
      if (voxel.confidence > 0.0f) {
        voxel.traversability /= voxel.confidence;
        voxel.confidence /= num_voxels;
      }
      classifyTraversabilityVoxel(voxel);
    }
  }
}

void HeightTraversabilityEstimator::classifyTraversabilityVoxel(
    TraversabilityVoxel& voxel) const {
  if (voxel.confidence <= 0.0f) {
    voxel.state = TraversabilityState::UNKNOWN;
    return;
  }
  if (voxel.confidence >= config.min_confidence) {
    if (voxel.traversability >= config.min_traversability) {
      voxel.state = TraversabilityState::TRAVERSABLE;
    } else {
      voxel.state = TraversabilityState::INTRAVERSABLE;
    }
  } else if (config.pessimistic && voxel.traversability < config.min_traversability) {
    voxel.state = TraversabilityState::INTRAVERSABLE;
  } else {
    voxel.state = TraversabilityState::UNKNOWN;
  }
}

BlockIndexSet HeightTraversabilityEstimator::get2DBlockIndices(
    const BlockIndices& blocks) const {
  BlockIndexSet block_indices;
  for (const auto& block : blocks) {
    block_indices.emplace(BlockIndex(block.x(), block.y(), 0));
  }
  return block_indices;
}
}  // namespace hydra::places
