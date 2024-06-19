// The contents of this file are originally from Panoptic-Mapping,
// under the following license:
//
// BSD 3-Clause License
// Copyright (c) 2021, ETHZ ASL
// All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// See https://github.com/ethz-asl/panoptic_mapping for original code and paper
//
// Modifications (including work done by Lukas Schmid for Khronos) fall under the same
// license as Hydra and are subject to the following copyright and disclaimer:
//
// Copyright 2022 Massachusetts Institute of Technology.
// All Rights Reserved
//
// Research was sponsored by the United States Air Force Research Laboratory and
// the United States Air Force Artificial Intelligence Accelerator and was
// accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
// and conclusions contained in this document are those of the authors and should
// not be interpreted as representing the official policies, either expressed or
// implied, of the United States Air Force or the U.S. Government. The U.S.
// Government is authorized to reproduce and distribute reprints for Government
// purposes notwithstanding any copyright notation herein.
#pragma once

#include <opencv2/core/mat.hpp>

#include "hydra/input/sensor.h"
#include "hydra/reconstruction/voxel_types.h"

namespace hydra {

/**
 * @brief Checks if a block is in the camera's view frustum. Approximate check that
 * does not check for occlusion.
 * @param block_index Index of the block to check.
 * @param T_C_B Transform from Block (B) frame to the Camera (C) frame.
 * @param block_size Size of the block in meters.
 * @param block_diag_half Half the length of the block's diagonal in meters.
 * @return True if the block can be in the camera's view frustum.
 */
bool blockIsInViewFrustum(const Sensor& sensor,
                          const BlockIndex& block_index,
                          const Eigen::Isometry3f& T_C_B,
                          float block_size,
                          float block_diag_half);

/**
 * @brief Checks if a block is in the camera's view frustum. Approximate check that
 * does not check for occlusion.
 * @param block Block to be checked.
 * @param T_C_B Transform from Block (B) frame to the Camera (C) frame.
 * @return True if the block can be in the camera's view frustum.
 */
bool blockIsInViewFrustum(const Sensor& sensor,
                          const spatial_hash::Block& block,
                          const Eigen::Isometry3f& T_C_B);
/**
 * @brief Finds the indices of all blocks that could be visible in the camera's view
 * frustum. Does not check for occlusion.
 * @param T_W_C Transform from Camera (C) frame to World (W) frame.
 * @param block_size Size of the block in meters.
 * @param min_range Optionally specify the minimum range to search for blocks in the
 * view in meters.
 * @param max_range Optionally specify the maximum range to search for blocks in the
 * view in meters.
 * @return List of block indices that could be visible in the camera's view frustum.
 */
BlockIndices findBlocksInViewFrustum(
    const Sensor& sensor,
    const Eigen::Isometry3f& T_W_C,
    float block_size,
    float min_range = 0.f,
    float max_range = std::numeric_limits<float>::max(),
    bool use_sensor_range = true);

/**
 * @brief Compute range image from pointcloud
 * @param points Pointcloud to compute range image from
 * @param min_range Optional min range to track
 * @param max_range Optional max range to track
 */
cv::Mat computeRangeImageFromPoints(const cv::Mat& points,
                                    float* min_range,
                                    float* max_range);

}  // namespace hydra
