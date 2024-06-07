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
#include "hydra/reconstruction/frame_data.h"

#include <glog/logging.h>

#include <opencv2/imgproc.hpp>

#include "hydra/common/global_info.h"
#include "hydra/common/semantic_color_map.h"

namespace hydra {

std::string showTypeInfo(const cv::Mat& mat) {
  std::stringstream ss;
  ss << "{depth: " << mat.depth() << ", channels: " << mat.channels() << "}";
  return ss.str();
}

bool FrameData::hasData() const {
  // we accept data as either a pointcloud (points) or an rgbd image (depth_image)
  // labels or color can encode the labels for a depth image or pointcloud. For the
  // former case, color_image and/or label_image will share the same resolution as
  // depth image and for the latter it will share the same resolution as the
  // pointcloud...
  return (!color_image.empty() || !label_image.empty()) &&
         (!depth_image.empty() || !vertex_map.empty());
}

bool FrameData::normalizeDepth() { return convertDepth(); }

bool FrameData::normalizeData(bool normalize_labels) {
  if (!convertDepth()) {
    return false;
  }

  if (!convertColor()) {
    return false;
  }

  // must come after convertColor as it uses color image
  if (normalize_labels && !convertLabels()) {
    return false;
  }

  if (!vertex_map.empty() && vertex_map.type() != CV_32FC3) {
    LOG(ERROR) << "pointcloud must be of type CV_32FC3, not "
               << showTypeInfo(vertex_map);
    return false;
  }

  return true;
}

bool FrameData::colorToLabels(const cv::Mat& colors) {
  if (colors.empty() || colors.channels() != 3) {
    LOG(ERROR) << "color image required to decode semantic labels";
    return false;
  }

  CHECK_EQ(colors.type(), CV_8UC3);

  const auto colormap_ptr = GlobalInfo::instance().getSemanticColorMap();
  if (!colormap_ptr || !colormap_ptr->isValid()) {
    LOG(ERROR)
        << "label colormap not valid, but required for converting colors to labels!";
    return false;
  }

  cv::Mat new_label_image(colors.size(), CV_32SC1);
  for (int r = 0; r < colors.rows; ++r) {
    for (int c = 0; c < colors.cols; ++c) {
      const auto& pixel = colors.at<cv::Vec3b>(r, c);
      voxblox::Color color{pixel[0], pixel[1], pixel[2], 255};
      // this is lazy, but works out to the same invalid label we normally use
      new_label_image.at<int32_t>(r, c) =
          colormap_ptr->getLabelFromColor(color).value_or(-1);
    }
  }

  label_image = new_label_image;
  return true;
}

bool FrameData::convertLabels() {
  if (label_image.empty()) {
    return colorToLabels(color_image);
  }

  if (label_image.channels() != 1) {
    return colorToLabels(label_image);
  }

  const auto label_type = label_image.type();
  if (label_type == CV_32SC1) {
    return true;
  }

  if (label_type != CV_8UC1 && label_type != CV_16UC1 && label_type != CV_8SC1 &&
      label_type != CV_16SC1) {
    LOG(ERROR) << "label image must be integer type, not " << showTypeInfo(label_image);
    return false;
  }

  if (label_type == CV_16SC1 || label_type == CV_8SC1) {
    LOG_FIRST_N(WARNING, 5)
        << "signed to unsigned conversion of labels may not do what you want!";
  }

  cv::Mat label_converted;
  label_image.convertTo(label_converted, CV_32SC1);
  label_image = label_converted;
  return true;
}

bool FrameData::convertDepth() {
  if (depth_image.empty()) {
    return true;
  }

  if (depth_image.channels() != 1) {
    LOG(ERROR) << "depth image must be single-channel";
    return false;
  }

  if (depth_image.type() == CV_32FC1) {
    return true;  // nothing else to do
  }

  if (depth_image.type() != CV_16UC1) {
    LOG(ERROR) << "only CV_32FC1 or CV_16UC1 formats supported, not "
               << showTypeInfo(depth_image);
    return false;
  }

  cv::Mat depth_converted;
  depth_image.convertTo(depth_converted, CV_32FC1, 1.0e-3);
  depth_image = depth_converted;
  return true;
}

bool FrameData::convertColor() {
  if (color_image.empty()) {
    return true;
  }

  if (color_image.type() != CV_8UC3) {
    LOG(ERROR) << "only 3-channel rgb images supported";
    return false;
  }

  if (color_is_bgr) {
    cv::Mat rgb_image;
    cv::cvtColor(color_image, rgb_image, cv::COLOR_BGR2RGB);
  }

  return true;
}

}  // namespace hydra
