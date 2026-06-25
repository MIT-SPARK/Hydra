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
#include <glog/logging.h>

namespace hydra::places {

namespace {
static const auto registration_ =
    config::RegistrationWithConfig<TraversabilityEstimator,
                                   HeightTraversabilityEstimator,
                                   HeightTraversabilityEstimator::Config>(
        "HeightTraversabilityEstimator");

static const auto gradient_registration_ =
    config::RegistrationWithConfig<TraversabilityEstimator,
                                   GradientTraversabilityEstimator,
                                   GradientTraversabilityEstimator::Config>(
        "GradientTraversabilityEstimator");

// Box-filter smoothing of a 2D height map: each cell becomes the average of
// itself and its observed neighbors. Removes projective TSDF radial bias
// (ripple artifact).
Index2DMap<float> smoothHeightMap(const Index2DMap<float>& height_map,
                                  const std::array<Index2D, 8>& neighbor_offsets) {
  Index2DMap<float> smoothed;
  smoothed.reserve(height_map.size());
  for (const auto& [center_idx, center_height] : height_map) {
    float height_sum = center_height;
    int count = 1;
    for (const auto& offset : neighbor_offsets) {
      auto it = height_map.find(center_idx + offset);
      if (it != height_map.end()) {
        height_sum += it->second;
        count++;
      }
    }
    smoothed[center_idx] = height_sum / count;
  }
  return smoothed;
}

BlockIndexSet get2DBlockIndices(const BlockIndices& blocks) {
  BlockIndexSet block_indices;
  for (const auto& block : blocks) {
    block_indices.emplace(BlockIndex(block.x(), block.y(), 0));
  }
  return block_indices;
}

// Build a 2D height map from a TSDF layer by scanning each vertical column of
// voxels for the highest surface voxel within the robot's vertical scan window.
// Uses block/local index access directly to avoid the signed/unsigned division
// bug in spatial_hash::blockIndexFromGlobalIndex that corrupts getVoxelPtr
// lookups for negative world coordinates.
Index2DMap<float> extractHeightMap(const TsdfLayer& tsdf_layer,
                                   const BlockIndexSet& blocks_2d,
                                   float robot_z,
                                   float height_below,
                                   float height_above,
                                   float min_weight) {
  const float voxel_size = tsdf_layer.voxel_size;
  const int vps = static_cast<int>(tsdf_layer.voxels_per_side);

  const VoxelKey min_key = tsdf_layer.getVoxelKey(Point(0, 0, robot_z - height_below));
  const VoxelKey max_key = tsdf_layer.getVoxelKey(Point(0, 0, robot_z + height_above));

  Index2DMap<float> height_map;
  for (const auto& block_idx_2d : blocks_2d) {
    for (int x = 0; x < vps; ++x) {
      for (int y = 0; y < vps; ++y) {
        // Scan the vertical column top-to-bottom and record the highest surface.
        for (int block_z = max_key.first.z(); block_z >= min_key.first.z(); --block_z) {
          const auto tsdf_block = tsdf_layer.getBlockPtr(
              BlockIndex(block_idx_2d.x(), block_idx_2d.y(), block_z));
          if (!tsdf_block) {
            continue;
          }

          const int min_voxel_z = block_z == min_key.first.z() ? min_key.second.z() : 0;
          const int max_voxel_z =
              block_z == max_key.first.z() ? max_key.second.z() : vps - 1;

          bool found = false;
          for (int z = max_voxel_z; z >= min_voxel_z; --z) {
            const auto& voxel = tsdf_block->getVoxel(VoxelIndex(x, y, z));
            if (voxel.weight < min_weight) {
              continue;
            }
            if (voxel.distance < voxel_size) {
              const VoxelKey key(
                  BlockIndex(block_idx_2d.x(), block_idx_2d.y(), block_z),
                  VoxelIndex(x, y, z));
              const Index2D map_key(block_idx_2d.x() * vps + x,
                                    block_idx_2d.y() * vps + y);
              height_map[map_key] = tsdf_layer.getVoxelPosition(key).z();
              found = true;
              break;
            }
          }
          if (found) {
            break;
          }
        }
      }
    }
  }
  return height_map;
}
// Where to put these helper functions?
}  // namespace

using spark_dsg::TraversabilityState;

void TraversabilityEstimator::classifyTraversabilityVoxel(
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

const std::array<Index2D, 8> GradientTraversabilityEstimator::kNeighborOffsets = {{
    {0, -1},   // bottom
    {-1, 0},   // left
    {0, 1},    // top
    {1, 0},    // right
    {-1, -1},  // bottom-left
    {-1, 1},   // top-left
    {1, 1},    // top-right
    {1, -1}    // bottom-right
}};

void declare_config(TraversabilityEstimator::Config& config) {
  using namespace config;
  field(config.min_confidence, "min_confidence");
  field(config.min_traversability, "min_traversability");
  field(config.pessimistic, "pessimistic");
  checkInRange(config.min_confidence, 0.0f, 1.0f, "min_confidence");
  checkInRange(config.min_traversability, 0.0f, 1.0f, "min_traversability");
}

void declare_config(HeightTraversabilityEstimator::Config& config) {
  using namespace config;
  name("HeightTraversabilityEstimator::Config");
  base<TraversabilityEstimator::Config>(config);
  field(config.height_above, "height_above", "m");
  field(config.height_below, "height_below", "m");
  checkCondition(config.height_above >= -config.height_below,
                 "'height_above' and 'height_below' don't span any volume");
}

HeightTraversabilityEstimator::HeightTraversabilityEstimator(const Config& config)
    : TraversabilityEstimator(config), config(config::checkValid(config)) {}

void HeightTraversabilityEstimator::updateTraversability(
    const ActiveWindowOutput& msg) {
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

BlockIndexSet HeightTraversabilityEstimator::get2DBlockIndices(
    const BlockIndices& blocks) const {
  BlockIndexSet block_indices;
  for (const auto& block : blocks) {
    block_indices.emplace(BlockIndex(block.x(), block.y(), 0));
  }
  return block_indices;
}

void declare_config(GradientTraversabilityEstimator::Config& config) {
  using namespace config;
  name("GradientTraversabilityEstimator::Config");
  base<TraversabilityEstimator::Config>(config);
  field(config.gradient_threshold, "gradient_threshold");
  field(config.height_above, "height_above", "m");
  field(config.height_below, "height_below", "m");
  field(config.min_weight, "min_weight");
  field(config.smoothing, "smoothing");

  checkCondition(config.gradient_threshold > 0.0f,
                 "gradient_threshold must be positive");
  checkCondition(config.height_above >= -config.height_below,
                 "'height_above' and 'height_below' don't span any volume");
}

GradientTraversabilityEstimator::GradientTraversabilityEstimator(const Config& config)
    : TraversabilityEstimator(config), config(config::checkValid(config)) {
  LOG(INFO) << "Created GradientTraversabilityEstimator with min traversability: "
            << config.min_traversability;
}

void GradientTraversabilityEstimator::updateTraversability(
    const ActiveWindowOutput& msg) {
  updateTsdf(msg);
  computeTraversability(msg);
}

void GradientTraversabilityEstimator::updateTsdf(const ActiveWindowOutput& msg) {
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

void GradientTraversabilityEstimator::computeTraversability(
    const ActiveWindowOutput& msg) {
  const auto updated_blocks_2d =
      get2DBlockIndices(msg.map().getTsdfLayer().allocatedBlockIndices());
  const float robot_z = msg.world_t_body.z();
  const int voxels_per_side = tsdf_layer_->voxels_per_side;

  // PASS 1: Extract heights from ALL currently allocated TSDF blocks (not just
  // previously-processed traversability blocks), so newly-seen areas are included.
  const auto all_blocks_2d = get2DBlockIndices(tsdf_layer_->allocatedBlockIndices());
  const Index2DMap<float> height_map = extractHeightMap(*tsdf_layer_,
                                                        all_blocks_2d,
                                                        robot_z,
                                                        config.height_below,
                                                        config.height_above,
                                                        config.min_weight);

  // Optionally smooth height map to remove projective TSDF radial bias (ripple
  // artifact). For each voxel, replace its height with the average of itself and
  // observed neighbors.
  const Index2DMap<float>& grad_height_map =
      config.smoothing ? smoothHeightMap(height_map, kNeighborOffsets) : height_map;

  // PASS 2: Update ONLY the blocks that were updated in TSDF.
  for (auto& block_idx_2d : updated_blocks_2d) {
    auto& trav_block =
        traversability_layer_->allocateBlock(block_idx_2d, voxels_per_side);
    trav_block.reset();  // Reset now, in Pass 2.
    trav_block.updated = true;

    for (int x = 0; x < voxels_per_side; ++x) {
      for (int y = 0; y < voxels_per_side; ++y) {
        auto& trav_voxel = trav_block.voxel(x, y);
        const BlockIndex global_2d = trav_block.globalFromLocalIndex(Index2D(x, y));
        const Index2D center_idx(global_2d.x(), global_2d.y());

        // Check if center has surface.
        auto center_it = grad_height_map.find(center_idx);
        if (center_it == grad_height_map.end()) {
          trav_voxel.confidence = 0.0f;
          trav_voxel.traversability = 0.0f;
          classifyTraversabilityVoxel(trav_voxel);
          continue;
        }

        const float center_height = center_it->second;

        // Compute mean gradient over observed neighbors. Skip missing neighbors —
        // the confidence field already captures partial coverage; treating missing
        // neighbors as infinite gradient makes boundary voxels intraversable even
        // on flat terrain, and prevents intermediate (yellow) traversability values.
        float gradient_sum = 0.0f;
        int num_neighbors_observed = 0;

        for (const auto& offset : kNeighborOffsets) {
          auto neighbor_it = grad_height_map.find(center_idx + offset);
          if (neighbor_it == grad_height_map.end()) {
            continue;
          }

          num_neighbors_observed++;

          const float height_diff = std::abs(neighbor_it->second - center_height);
          const float horiz_dist =
              tsdf_layer_->voxel_size * offset.cast<float>().norm();
          gradient_sum += height_diff / horiz_dist;
        }

        const float mean_gradient =
            num_neighbors_observed > 0 ? gradient_sum / num_neighbors_observed : 0.0f;
        trav_voxel.traversability = computeTraversabilityFromGradient(mean_gradient);
        trav_voxel.confidence = num_neighbors_observed / 8.0f;
        trav_voxel.height = center_height;

        classifyTraversabilityVoxel(trav_voxel);
      }
    }
  }
}

float GradientTraversabilityEstimator::computeTraversabilityFromGradient(
    float gradient) const {
  if (gradient >= config.gradient_threshold) {
    return 0.0f;  // Intraversable.
  }

  // Linear interpolation: 1.0 at gradient=0, 0.0 at gradient=threshold.
  return 1.0f - (gradient / config.gradient_threshold);
}

}  // namespace hydra::places
