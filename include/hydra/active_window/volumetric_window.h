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
#pragma once

#include <spatial_hash/block.h>
#include <spatial_hash/types.h>

#include <Eigen/Geometry>

namespace hydra {

class VolumetricMap;

struct VolumetricBlockInfo {
  const spatial_hash::BlockIndex index;
  const double block_size;
  const uint64_t update_stamp_ns;

  VolumetricBlockInfo(const spatial_hash::Block& block, uint64_t update_stamp_ns = 0);
  VolumetricBlockInfo(const spatial_hash::BlockIndex& index,
                      double block_size,
                      uint64_t update_stamp_ns = 0);
  Eigen::Vector3f blockCenter() const;
};

struct VolumetricWindow {
  virtual ~VolumetricWindow() = default;

  spatial_hash::BlockIndices archiveBlocks(uint64_t timestamp_ns,
                                           const Eigen::Isometry3d& world_T_body,
                                           VolumetricMap& map,
                                           bool skip_updated = true) const;

  bool inBounds(uint64_t timestamp_ns,
                const Eigen::Isometry3d& world_T_body,
                const VolumetricBlockInfo& block) const;

  virtual bool inBounds(uint64_t timestamp_ns,
                        const Eigen::Isometry3d& world_T_body,
                        const uint64_t last_updated_ns,
                        const Eigen::Vector3d& last_pos) const = 0;
};

struct SpatialWindowChecker : VolumetricWindow {
  struct Config {
    double max_radius_m = 8.0;
  } const config;

  explicit SpatialWindowChecker(const Config& config);
  bool inBounds(uint64_t timestamp_ns,
                const Eigen::Isometry3d& world_T_body,
                const uint64_t last_updated_ns,
                const Eigen::Vector3d& last_pos) const override;
};

void declare_config(SpatialWindowChecker::Config& config);

}  // namespace hydra
