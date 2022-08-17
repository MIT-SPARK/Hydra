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
#include "hydra_utils/colormap_utils.h"

#include <algorithm>
#include <opencv2/imgproc.hpp>

namespace hydra {
namespace dsg_utils {

inline double lerp(double min, double max, double ratio) {
  return (max - min) * ratio + min;
}

std_msgs::ColorRGBA makeColorMsg(const NodeColor& color, double alpha) {
  std_msgs::ColorRGBA msg;
  msg.r = static_cast<double>(color(0)) / 255.0;
  msg.g = static_cast<double>(color(1)) / 255.0;
  msg.b = static_cast<double>(color(2)) / 255.0;
  msg.a = alpha;
  return msg;
}

NodeColor getRgbFromHls(double hue, double luminance, double saturation) {
  // make sure we clip the inputs to the expected range
  hue = std::clamp(hue, 0.0, 1.0);
  luminance = std::clamp(luminance, 0.0, 1.0);
  saturation = std::clamp(saturation, 0.0, 1.0);

  cv::Mat hls_value(1, 1, CV_32FC3);
  // hue is in degrees, not [0, 1]
  hls_value.at<float>(0) = hue * 360.0;
  hls_value.at<float>(1) = luminance;
  hls_value.at<float>(2) = saturation;

  cv::Mat bgr;
  cv::cvtColor(hls_value, bgr, cv::COLOR_HLS2BGR);

  NodeColor color;
  color(0, 0) = static_cast<uint8_t>(255 * bgr.at<float>(2));
  color(1, 0) = static_cast<uint8_t>(255 * bgr.at<float>(1));
  color(2, 0) = static_cast<uint8_t>(255 * bgr.at<float>(0));
  return color;
}

NodeColor interpolateColorMap(const ColormapConfig& config, double ratio) {
  // override ratio input to be in [0, 1]
  ratio = std::clamp(ratio, 0.0, 1.0);

  cv::Mat hls_value(1, 1, CV_32FC3);
  // hue is in degrees, not [0, 1]
  hls_value.at<float>(0) = lerp(config.min_hue, config.max_hue, ratio) * 360.0;
  hls_value.at<float>(1) = lerp(config.min_luminance, config.max_luminance, ratio);
  hls_value.at<float>(2) = lerp(config.min_saturation, config.max_saturation, ratio);

  cv::Mat bgr;
  cv::cvtColor(hls_value, bgr, cv::COLOR_HLS2BGR);

  NodeColor color;
  color(0, 0) = static_cast<uint8_t>(255 * bgr.at<float>(2));
  color(1, 0) = static_cast<uint8_t>(255 * bgr.at<float>(1));
  color(2, 0) = static_cast<uint8_t>(255 * bgr.at<float>(0));
  return color;
}

}  // namespace dsg_utils
}  // namespace hydra
