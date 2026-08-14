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
#include <config_utilities/validation.h>

namespace hydra::places {

using State = spark_dsg::TraversabilityState;

namespace {
static const auto registration =
    config::RegistrationWithConfig<TraversabilityProcessor,
                                   ErosionDilation,
                                   ErosionDilation::Config>("ErosionDilation");
}  // namespace

TraversabilityProcessors::TraversabilityProcessors(const Config& config) {
  for (const auto& processor_config : config) {
    auto processor = processor_config.create();
    if (processor) {
      processors_.emplace_back(std::move(processor));
    }
  }
}

void TraversabilityProcessors::apply(TraversabilityLayer& layer) const {
  for (const auto& processor : processors_) {
    processor->apply(layer);
  }
}

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

void ErosionDilation::apply(TraversabilityLayer& layer) {
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

}  // namespace hydra::places
