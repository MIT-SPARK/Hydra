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
#include "hydra/places/robot_footprint_integrator.h"

#include <config_utilities/config.h>
#include <config_utilities/types/eigen_matrix.h>

namespace hydra {

void declare_config(RobotFootprintIntegrator::Config& config) {
  using namespace config;
  name("RobotFootprintIntegrator::Config");
  field(config.bbox_min, "bbox_min");
  field(config.bbox_max, "bbox_max");
  field(config.tsdf_weight, "tsdf_weight");
}

RobotFootprintIntegrator::RobotFootprintIntegrator(const Config& config)
    : config(config), bbox(config.bbox_min, config.bbox_max) {}

RobotFootprintIntegrator::~RobotFootprintIntegrator() = default;

// somewhat adapted from findBlocksInViewFrustum
BlockIndices getAffectedBlocks(const Eigen::Isometry3f& world_T_body,
                               const BoundingBox& bbox,
                               VolumetricMap& map) {
  const auto b_size = map.blockSize();
  const auto block_diag_half = std::sqrt(3.0f) * b_size / 2.0f;
  const auto margin = Eigen::Vector3f::Constant(block_diag_half);
  BoundingBox inflated = bbox;
  inflated.dimensions += 2.0f * margin;

  const double max_range = inflated.dimensions.maxCoeff();
  const int max_steps = std::ceil(max_range / b_size) + 1;
  const auto body_T_world = world_T_body.inverse();
  const auto pos = world_T_body.translation();
  const auto body_idx = spatial_hash::indexFromPoint<BlockIndex>(pos, 1.0f / b_size);

  BlockIndices result;
  for (int x = -max_steps; x <= max_steps; ++x) {
    for (int y = -max_steps; y <= max_steps; ++y) {
      for (int z = -max_steps; z <= max_steps; ++z) {
        const Eigen::Vector3f offset(x, y, z);
        const auto p_body = body_T_world * (pos + offset * b_size);
        if (!inflated.contains(p_body)) {
          continue;
        }

        result.push_back(body_idx + offset.cast<BlockIndex::Scalar>());
      }
    }
  }

  return result;
}

void RobotFootprintIntegrator::addFreespaceFootprint(const Eigen::Isometry3f& w_T_b,
                                                     VolumetricMap& map) const {
  const auto block_indices = getAffectedBlocks(w_T_b, bbox, map);
  const auto b_T_w = w_T_b.inverse();
  auto& tsdf = map.getTsdfLayer();
  const auto semantic_layer = map.getSemanticLayer();

  BlockIndices new_blocks;
  for (const auto& idx : block_indices) {
    TsdfBlock::Ptr block;
    if (tsdf.hasBlock(idx)) {
      block = tsdf.getBlockPtr(idx);
    } else {
      block = tsdf.allocateBlockPtr(idx);
      if (semantic_layer) {
        semantic_layer->allocateBlockPtr(idx);
      }

      new_blocks.push_back(idx);
    }

    for (size_t i = 0; i < block->numVoxels(); ++i) {
      const auto p_body = b_T_w * block->getVoxelPosition(i);
      // technically we should expand by sqrt(3) * voxel_size, but the more
      // conservative, the better
      if (!bbox.contains(p_body)) {
        continue;  // voxel doesn't intersect with free-space pattern
      }

      block->setUpdated();
      auto& voxel = block->getVoxel(i);
      voxel.distance = map.config.truncation_distance;
      voxel.weight = config.tsdf_weight;
    }
  }

  for (const auto& idx : new_blocks) {
    const auto block = map.getTsdfLayer().getBlockPtr(idx);
    if (!block || block->esdf_updated || block->mesh_updated) {
      continue;
    }

    map.getTsdfLayer().removeBlock(idx);
    if (semantic_layer) {
      semantic_layer->removeBlock(idx);
    }
  }
}

}  // namespace hydra
