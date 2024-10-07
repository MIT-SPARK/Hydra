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

#include "hydra/active_window/volumetric_window.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/validation.h>

#include "hydra/reconstruction/volumetric_map.h"

namespace hydra {

VolumetricBlockInfo::VolumetricBlockInfo(const spatial_hash::Block& block,
                                         uint64_t update_stamp_ns)
    : index(block.index),
      block_size(block.block_size),
      update_stamp_ns(update_stamp_ns) {}

VolumetricBlockInfo::VolumetricBlockInfo(const spatial_hash::BlockIndex& index,
                                         double block_size,
                                         uint64_t update_stamp_ns)
    : index(index), block_size(block_size), update_stamp_ns(update_stamp_ns) {}

Eigen::Vector3f VolumetricBlockInfo::blockCenter() const {
  return spatial_hash::centerPointFromIndex(index, block_size);
}

BlockIndices VolumetricWindow::archiveBlocks(uint64_t timestamp_ns,
                                             const Eigen::Isometry3d& world_T_body,
                                             VolumetricMap& map,
                                             bool skip_updated) const {
  BlockIndices to_remove;
  const auto& tsdf = map.getTsdfLayer();
  for (const auto& block : tsdf) {
    // TODO(nathan|lukas) the implicit constructor zeros the block update timestamp
    if (inBounds(timestamp_ns, world_T_body, block)) {
      continue;
    }

    if (skip_updated && block.updated) {
      continue;
    }

    to_remove.push_back(block.index);
  }

  map.removeBlocks(to_remove);
  return to_remove;
}

void declare_config(SpatialWindowChecker::Config& config) {
  using namespace config;
  name("SpatialWindowChecker::Config");
  field(config.max_radius_m, "max_radius_m");
  check(config.max_radius_m, GT, 0.0, "max_radius_m");
}

namespace {
static const auto registration =
    config::RegistrationWithConfig<VolumetricWindow,
                                   SpatialWindowChecker,
                                   SpatialWindowChecker::Config>("spatial");
}

SpatialWindowChecker::SpatialWindowChecker(const Config& config)
    : config(config::checkValid(config)) {}

bool SpatialWindowChecker::inBounds(uint64_t /* timestamp_ns */,
                                    const Eigen::Isometry3d& world_T_body,
                                    const VolumetricBlockInfo& block) const {
  const Eigen::Vector3f pos = world_T_body.translation().cast<float>();
  return (pos - block.blockCenter()).norm() <= config.max_radius_m;
}

}  // namespace hydra
