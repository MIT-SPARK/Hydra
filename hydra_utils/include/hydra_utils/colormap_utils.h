#pragma once
#include "hydra_utils/visualizer_types.h"

#include <std_msgs/ColorRGBA.h>

#include <algorithm>
#include <opencv2/imgproc.hpp>

namespace hydra {
namespace dsg_utils {

inline std_msgs::ColorRGBA makeColorMsg(const NodeColor& color, double alpha = 1.0) {
  std_msgs::ColorRGBA msg;
  msg.r = static_cast<double>(color(0)) / 255.0;
  msg.g = static_cast<double>(color(1)) / 255.0;
  msg.b = static_cast<double>(color(2)) / 255.0;
  msg.a = alpha;
  return msg;
}

inline double lerp(double min, double max, double ratio) {
  return (max - min) * ratio + min;
}

inline NodeColor getRgbFromHls(double hue, double luminance, double saturation) {
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

inline NodeColor interpolateColorMap(const ColormapConfig& config, double ratio) {
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
