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
#include "hydra/input/sensor_input_packet.h"

#include <glog/logging.h>

#include <opencv2/imgproc.hpp>

#include "hydra/common/global_info.h"

namespace hydra {

ImageInputPacket::ImageInputPacket(uint64_t stamp, size_t sensor_id)
    : SensorInputPacket(stamp, sensor_id) {}

bool ImageInputPacket::fillInputData(InputData& msg) const {
  if (depth.empty()) {
    LOG(ERROR) << "Missing required images: Depth image must be set.";
    return false;
  }
  if (color.empty() && labels.empty()) {
    LOG(ERROR) << "Missing required images: Color or label image must be set.";
    return false;
  }

  msg.timestamp_ns = timestamp_ns;
  msg.color_image = color;
  if (color_is_bgr) {
    cv::cvtColor(msg.color_image, msg.color_image, cv::COLOR_BGR2RGB);
  }
  msg.depth_image = depth;
  msg.label_image = labels;
  return true;
}

CloudInputPacket::CloudInputPacket(uint64_t stamp, size_t sensor_id)
    : SensorInputPacket(stamp, sensor_id) {}

bool CloudInputPacket::fillInputData(InputData& msg) const {
  if (points.empty() || (labels.empty() && colors.empty())) {
    LOG(ERROR) << "Missing required pointcloud.";
    return false;
  }

  msg.timestamp_ns = timestamp_ns;
  msg.vertex_map = points;
  msg.points_in_world_frame = in_world_frame;
  msg.color_image = colors;
  msg.label_image = labels;
  return true;
}

}  // namespace hydra
