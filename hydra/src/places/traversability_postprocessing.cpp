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
#include "hydra/places/traversability_postprocessing.h"

#include <config_utilities/config.h>
#include <config_utilities/types/eigen_matrix.h>
#include <config_utilities/validation.h>

#include <cmath>
#include <limits>

namespace hydra::places {

using State = spark_dsg::TraversabilityState;

namespace {
static const auto dilation_registration =
    config::RegistrationWithConfig<TransientTraversabilityProcessor,
                                   ErosionDilation,
                                   ErosionDilation::Config>("ErosionDilation");

static const auto footprint_registration =
    config::RegistrationWithConfig<PersistentTraversabilityProcessor,
                                   RobotFootprintTraversability,
                                   RobotFootprintTraversability::Config>(
        "RobotFootprintTraversability");

// Block index of a global 2D voxel index. Uses floor division to also handle negative
// indices correctly.
BlockIndex blockFromGlobal(const Index2D& index, int voxels_per_side) {
  const auto floor_div = [voxels_per_side](int i) {
    return i >= 0 ? i / voxels_per_side : (i - voxels_per_side + 1) / voxels_per_side;
  };
  return BlockIndex(floor_div(index.x()), floor_div(index.y()), 0);
}
}  // namespace

void declare_config(ErosionDilation::Config& config) {
  using namespace config;
  name("ErosionDilation::Config");
  field(config.num_dilations, "num_dilations");
  check(config.num_dilations, GT, 0, "num_dilations");
}

ErosionDilation::ErosionDilation(const Config& config)
    : config(config::checkValid(config)) {}

struct Updates : public spatial_hash::Block, Index2DSet {
  Updates(float block_size, const BlockIndex& index)
      : spatial_hash::Block(block_size, index) {}
};
using UpdatesLayer = spatial_hash::BlockLayer<Updates>;

void ErosionDilation::apply(const ActiveWindowOutput&, TraversabilityLayer& layer) {
  // NOTE(lschmid): Naive implementation for now.
  UpdatesLayer updates(layer.blockSize());
  for (const auto& block : layer) {
    updates.allocateBlock(block.index);
  }
  for (size_t i = 0; i < config.num_dilations; ++i) {
    if (i != 0) {
      for (auto& block : updates) {
        block.clear();
      }
    }
    for (const auto& block : layer) {
      auto& updates_block = updates.getBlock(block.index);
      for (size_t x = 0; x < block.voxels_per_side; ++x) {
        for (size_t y = 0; y < block.voxels_per_side; ++y) {
          const auto& voxel = block.voxel(x, y);
          if (voxel.state != State::INTRAVERSABLE) {
            continue;
          }
          for (const auto& offset : offsets_) {
            const Index2D n_index(x + offset.x(), y + offset.y());
            if (block.isValidIndex(n_index)) {
              updates_block.insert(n_index);
            } else {
              const BlockIndex global_index = block.globalFromLocalIndex(n_index);
              auto n_block =
                  updates.getBlockPtr(layer.blockIndexFromGlobal(global_index));
              if (n_block) {
                n_block->insert(layer.voxelIndexFromGlobal(global_index));
              }
            }
          }
        }
      }
    }

    // Apply updates.
    for (const auto& update_block : updates) {
      auto& block = layer.getBlock(update_block.index);
      for (const auto& index : update_block) {
        block.voxel(index).state = State::INTRAVERSABLE;
      }
      block.updated = true;
    }
  }
}

const std::array<Index2D, 4> ErosionDilation::offsets_ = {
    Index2D(1, 0), Index2D(-1, 0), Index2D(0, 1), Index2D(0, -1)};
// Index2D(1, 1),
// Index2D(-1, -1),
// Index2D(1, -1),
// Index2D(-1, 1)};

void declare_config(RobotFootprintTraversability::Config& config) {
  using namespace config;
  name("RobotFootprintTraversability::Config");
  field(config.bbox_min, "bbox_min", "m");
  field(config.bbox_max, "bbox_max", "m");
  checkCondition(config.bbox_max.x() > config.bbox_min.x(),
                 "'bbox_max.x' must be larger than 'bbox_min.x'");
  checkCondition(config.bbox_max.y() > config.bbox_min.y(),
                 "'bbox_max.y' must be larger than 'bbox_min.y'");
}

RobotFootprintTraversability::RobotFootprintTraversability(const Config& config)
    : config(config::checkValid(config)) {}

void RobotFootprintTraversability::apply(const ActiveWindowOutput& msg,
                                         TraversabilityLayer& layer) {
  addFootprint(msg.world_T_body<float>(), layer);
}

void RobotFootprintTraversability::addFootprint(const Eigen::Isometry3f& world_T_body,
                                                TraversabilityLayer& layer) const {
  const float voxel_size = layer.voxel_size;
  // Inflate the footprint by half a voxel so that every voxel intersecting the
  // footprint is included, not only those whose center lies inside it.
  const Eigen::Vector2f half_voxel = Eigen::Vector2f::Constant(voxel_size / 2.0f);
  const Eigen::Vector2f min = config.bbox_min - half_voxel;
  const Eigen::Vector2f max = config.bbox_max + half_voxel;

  // Compute the world frame xy extent of the footprint.
  Eigen::Vector2f world_min =
      Eigen::Vector2f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector2f world_max = -world_min;
  for (const auto& corner : {Eigen::Vector2f(min.x(), min.y()),
                             Eigen::Vector2f(min.x(), max.y()),
                             Eigen::Vector2f(max.x(), min.y()),
                             Eigen::Vector2f(max.x(), max.y())}) {
    const Eigen::Vector3f p_world =
        world_T_body * Eigen::Vector3f(corner.x(), corner.y(), 0.0f);
    world_min = world_min.cwiseMin(p_world.head<2>());
    world_max = world_max.cwiseMax(p_world.head<2>());
  }

  const auto body_T_world = world_T_body.inverse();
  const float z = world_T_body.translation().z();
  const int vps = static_cast<int>(layer.voxels_per_side);
  const int x_min = std::floor(world_min.x() / voxel_size);
  const int x_max = std::floor(world_max.x() / voxel_size);
  const int y_min = std::floor(world_min.y() / voxel_size);
  const int y_max = std::floor(world_max.y() / voxel_size);
  for (int x = x_min; x <= x_max; ++x) {
    for (int y = y_min; y <= y_max; ++y) {
      const Eigen::Vector3f center((x + 0.5f) * voxel_size, (y + 0.5f) * voxel_size, z);
      const Eigen::Vector2f p_body = (body_T_world * center).head<2>();
      if ((p_body.array() < min.array()).any() ||
          (p_body.array() > max.array()).any()) {
        continue;
      }

      const auto block_index = blockFromGlobal(Index2D(x, y), vps);
      auto block = layer.getBlockPtr(block_index);
      if (!block) {
        continue;
      }

      auto& voxel =
          block->voxel(Index2D(x - block_index.x() * vps, y - block_index.y() * vps));
      if (!voxel.traversed) {
        voxel.traversed = true;
        block->updated = true;
      }
    }
  }
}

}  // namespace hydra::places
