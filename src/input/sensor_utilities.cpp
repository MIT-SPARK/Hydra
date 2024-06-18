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
#include "hydra/input/sensor_utilities.h"

namespace hydra {

bool blockIsInViewFrustum(const Sensor& sensor,
                          const BlockIndex& block_index,
                          const Eigen::Isometry3f& T_C_B,
                          float block_size,
                          float block_diag_half) {
  const auto p_B = spatial_hash::centerPointFromIndex(block_index, block_size);
  return sensor.pointIsInViewFrustum(T_C_B * p_B, block_diag_half);
}

bool blockIsInViewFrustum(const Sensor& sensor,
                          const spatial_hash::Block& block,
                          const Eigen::Isometry3f& T_C_B) {
  const auto block_diag_half = std::sqrt(3.0f) * block.block_size / 2.0f;
  return blockIsInViewFrustum(
      sensor, block.index, T_C_B, block.block_size, block_diag_half);
}

BlockIndices findBlocksInViewFrustum(const Sensor& sensor,
                                     const Eigen::Isometry3f& T_W_C,
                                     float block_size,
                                     float min_range,
                                     float max_range,
                                     bool use_sensor_range) {
  BlockIndices result;
  // Simple version to get all blocks in world frame that could be visible: Iterate
  // through all blocks that could be in range and check if they are in the view
  // frustum.

  const auto T_C_W = T_W_C.inverse();
  const auto camera_W = T_W_C.translation();  // position of camera in world frame.
  const int max_steps = std::ceil(sensor.max_range() / block_size) + 1;
  const auto block_diag_half = std::sqrt(3.0f) * block_size / 2.0f;
  const auto camera_index =
      spatial_hash::indexFromPoint<BlockIndex>(camera_W, 1.f / block_size);

  min_range = use_sensor_range ? sensor.min_range() : min_range;
  max_range = use_sensor_range ? sensor.max_range() : max_range;

  for (int x = -max_steps; x <= max_steps; ++x) {
    for (int y = -max_steps; y <= max_steps; ++y) {
      for (int z = -max_steps; z <= max_steps; ++z) {
        const Eigen::Vector3f offset(x, y, z);
        // p_C=  T_C_W * p_W
        const auto p_C = T_C_W * (camera_W + offset * block_size);
        const float distance = p_C.norm();
        if (distance < min_range - block_diag_half ||
            distance > max_range + block_diag_half) {
          continue;
        }

        if (sensor.pointIsInViewFrustum(p_C, block_diag_half)) {
          result.push_back(camera_index + offset.cast<BlockIndex::Scalar>());
        }
      }
    }
  }

  return result;
}

cv::Mat computeRangeImageFromPoints(const cv::Mat& points,
                                    float* min_range,
                                    float* max_range) {
  // Compute the range (=radial distance) from the pointcloud.
  cv::Mat range_image(points.size(), CV_32FC1);
  if (min_range) {
    *min_range = std::numeric_limits<float>::max();
  }

  if (max_range) {
    *max_range = std::numeric_limits<float>::lowest();
  }

  for (int v = 0; v < points.rows; v++) {
    for (int u = 0; u < points.cols; u++) {
      const auto& point = points.at<cv::Vec3f>(v, u);
      const float x = point[0];
      const float y = point[1];
      const float z = point[2];
      const auto range = std::sqrt(x * x + y * y + z * z);
      range_image.at<float>(v, u) = range;
      if (min_range) {
        *min_range = std::min(*min_range, range);
      }

      if (max_range) {
        *max_range = std::max(*max_range, range);
      }
    }
  }

  return range_image;
}

}  // namespace hydra
