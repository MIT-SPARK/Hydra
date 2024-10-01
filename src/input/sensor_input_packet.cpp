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
namespace {

inline std::string showImageDim(const cv::Mat& mat) {
  std::stringstream ss;
  ss << "[rows=" << mat.rows << ", cols=" << mat.cols << "]";
  return ss.str();
}

inline bool sizesMatch(const cv::Mat& lhs, const cv::Mat& rhs) {
  return lhs.rows == rhs.rows && lhs.cols == rhs.cols;
}

}  // namespace

bool SensorInputPacket::fillInputData(InputData& msg) const {
  msg.timestamp_ns = timestamp_ns;
  msg.feature = input_feature;
  return fillInputDataImpl(msg);
}

ImageInputPacket::ImageInputPacket(uint64_t stamp, const std::string& sensor_name)
    : SensorInputPacket(stamp, sensor_name) {}

bool ImageInputPacket::fillInputDataImpl(InputData& msg) const {
  if (depth.empty()) {
    LOG(ERROR) << "Missing required images: depth image must be set.";
    return false;
  }

  if (color.empty() && labels.empty()) {
    LOG(ERROR) << "Missing required images: color or label image must be set.";
    return false;
  }

  msg.color_image = color;
  if (color_is_bgr && !msg.color_image.empty()) {
    cv::cvtColor(msg.color_image, msg.color_image, cv::COLOR_BGR2RGB);
  }

  msg.depth_image = depth;
  msg.label_image = labels;
  // TODO(nathan) think about better copy
  msg.label_features = label_features;

  if (!msg.label_image.empty() && !sizesMatch(msg.depth_image, msg.label_image)) {
    LOG(ERROR) << "Label dimensions " << showImageDim(msg.label_image)
               << " do not match depth dimensions " << showImageDim(msg.depth_image);
    return false;
  }

  if (!msg.color_image.empty() && !sizesMatch(msg.depth_image, msg.color_image)) {
    LOG(ERROR) << "Color dimensions " << showImageDim(msg.color_image)
               << " do not match depth dimensions " << showImageDim(msg.depth_image);
    return false;
  }

  return true;
}

CloudInputPacket::CloudInputPacket(uint64_t stamp, const std::string& sensor_name)
    : SensorInputPacket(stamp, sensor_name) {}

bool CloudInputPacket::fillInputDataImpl(InputData& msg) const {
  if (points.empty() || (labels.empty() && colors.empty())) {
    LOG(ERROR) << "Missing required pointcloud.";
    return false;
  }

  msg.vertex_map = points;
  msg.points_in_world_frame = in_world_frame;
  msg.color_image = colors;
  msg.label_image = labels;

  if (!msg.label_image.empty() && !sizesMatch(msg.vertex_map, msg.label_image)) {
    LOG(ERROR) << "Label dimensions " << showImageDim(msg.label_image)
               << " do not match pointcloud dimensions "
               << showImageDim(msg.vertex_map);
    return false;
  }

  if (!msg.color_image.empty() && !sizesMatch(msg.vertex_map, msg.color_image)) {
    LOG(ERROR) << "Color dimensions " << showImageDim(msg.color_image)
               << " do not match pointcloud dimensions "
               << showImageDim(msg.vertex_map);
    return false;
  }

  return true;
}

}  // namespace hydra
