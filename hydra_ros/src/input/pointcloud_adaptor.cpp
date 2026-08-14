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
#include "hydra_ros/input/pointcloud_adaptor.h"

#include <glog/logging.h>

namespace hydra {

template <typename T>
void parseField(const uint8_t* point_ptr, const uint32_t offset, T& value) {
  value = *reinterpret_cast<const T*>(point_ptr + offset);
}

#define DECLARE_INT_PARSER(type)                                                  \
  uint32_t parseIntFrom_##type(const uint8_t* point_ptr, const uint32_t offset) { \
    type value;                                                                   \
    parseField(point_ptr, offset, value);                                         \
    return static_cast<uint32_t>(value);                                          \
  }

#define DECLARE_FLOAT_PARSER(type)                                                \
  double parseFloatFrom_##type(const uint8_t* point_ptr, const uint32_t offset) { \
    type value;                                                                   \
    parseField(point_ptr, offset, value);                                         \
    return static_cast<double>(value);                                            \
  }

#define DECLARE_COLOR_PARSER(type)                                       \
  std::array<uint8_t, 4> parseColorFrom_##type(const uint8_t* point_ptr, \
                                               const uint32_t offset) {  \
    type value;                                                          \
    parseField(point_ptr, offset, value);                                \
                                                                         \
    uint8_t color[4];                                                    \
    std::memcpy(color, &value, sizeof(color));                           \
    return {color[2], color[1], color[0], color[3]};                     \
  }

DECLARE_INT_PARSER(int8_t)
DECLARE_INT_PARSER(uint8_t)
DECLARE_INT_PARSER(int16_t)
DECLARE_INT_PARSER(uint16_t)
DECLARE_INT_PARSER(int32_t)
DECLARE_INT_PARSER(uint32_t)
DECLARE_COLOR_PARSER(float)
DECLARE_COLOR_PARSER(uint32_t)
DECLARE_FLOAT_PARSER(float)
DECLARE_FLOAT_PARSER(double)

#undef DECLARE_INT_PARSER
#undef DECLARE_FLOAT_PARSER
#undef DECLARE_COLOR_PARSER

using sensor_msgs::msg::PointField;

std::function<double(const uint8_t*)> initFloatParser(const PointField& field) {
  using namespace std::placeholders;
  if (field.datatype == PointField::INT8) {
    LOG(ERROR) << "cannot parse float from " << sensor_msgs::msg::to_yaml(field);
    return {};
  }
  if (field.datatype == PointField::UINT8) {
    LOG(ERROR) << "cannot parse float from " << sensor_msgs::msg::to_yaml(field);
    return {};
  }
  if (field.datatype == PointField::INT16) {
    LOG(ERROR) << "cannot parse float from " << sensor_msgs::msg::to_yaml(field);
    return {};
  }
  if (field.datatype == PointField::UINT16) {
    LOG(ERROR) << "cannot parse float from " << sensor_msgs::msg::to_yaml(field);
    return {};
  }
  if (field.datatype == PointField::INT32) {
    LOG(ERROR) << "cannot parse float from " << sensor_msgs::msg::to_yaml(field);
    return {};
  }
  if (field.datatype == PointField::UINT32) {
    LOG(ERROR) << "cannot parse float from " << sensor_msgs::msg::to_yaml(field);
    return {};
  }

  if (field.datatype == PointField::FLOAT32) {
    return std::bind(&parseFloatFrom_float, _1, field.offset);
  }
  if (field.datatype == PointField::FLOAT64) {
    return std::bind(&parseFloatFrom_double, _1, field.offset);
  }

  LOG(ERROR) << "unknown datatype: " << sensor_msgs::msg::to_yaml(field);
  return {};
}

std::function<double(const uint8_t*)> initIntParser(const PointField& field) {
  using namespace std::placeholders;
  if (field.datatype == PointField::INT8) {
    return std::bind(&parseIntFrom_int8_t, _1, field.offset);
  }
  if (field.datatype == PointField::UINT8) {
    return std::bind(&parseIntFrom_uint8_t, _1, field.offset);
  }
  if (field.datatype == PointField::INT16) {
    return std::bind(&parseIntFrom_int16_t, _1, field.offset);
  }
  if (field.datatype == PointField::UINT16) {
    return std::bind(&parseIntFrom_uint16_t, _1, field.offset);
  }
  if (field.datatype == PointField::INT32) {
    return std::bind(&parseIntFrom_int32_t, _1, field.offset);
  }
  if (field.datatype == PointField::UINT32) {
    return std::bind(&parseIntFrom_uint32_t, _1, field.offset);
  }

  if (field.datatype == PointField::FLOAT32) {
    LOG(ERROR) << "cannot parse int from " << sensor_msgs::msg::to_yaml(field);
    return {};
  }
  if (field.datatype == PointField::FLOAT64) {
    LOG(ERROR) << "cannot parse int from " << sensor_msgs::msg::to_yaml(field);
    return {};
  }

  LOG(ERROR) << "unknown datatype: " << sensor_msgs::msg::to_yaml(field);
  return {};
}

std::function<std::array<uint8_t, 4>(const uint8_t*)> initColorParser(
    const PointField& field) {
  using namespace std::placeholders;
  if (field.datatype == PointField::INT8) {
    LOG(ERROR) << "cannot parse color from " << sensor_msgs::msg::to_yaml(field);
    return {};
  }
  if (field.datatype == PointField::UINT8) {
    LOG(ERROR) << "cannot parse color from " << sensor_msgs::msg::to_yaml(field);
    return {};
  }
  if (field.datatype == PointField::INT16) {
    LOG(ERROR) << "cannot parse color from " << sensor_msgs::msg::to_yaml(field);
    return {};
  }
  if (field.datatype == PointField::UINT16) {
    LOG(ERROR) << "cannot parse color from " << sensor_msgs::msg::to_yaml(field);
    return {};
  }
  if (field.datatype == PointField::INT32) {
    LOG(ERROR) << "cannot parse color from " << sensor_msgs::msg::to_yaml(field);
    return {};
  }
  if (field.datatype == PointField::FLOAT64) {
    LOG(ERROR) << "cannot parse color from " << sensor_msgs::msg::to_yaml(field);
    return {};
  }

  if (field.datatype == PointField::UINT32) {
    return std::bind(&parseColorFrom_uint32_t, _1, field.offset);
  }
  if (field.datatype == PointField::FLOAT32) {
    return std::bind(&parseColorFrom_float, _1, field.offset);
  }

  LOG(ERROR) << "unknown datatype: " << sensor_msgs::msg::to_yaml(field);
  return {};
}

PointcloudAdaptor::PointcloudAdaptor(const sensor_msgs::msg::PointCloud2& cloud) {
  for (const auto& field : cloud.fields) {
    if (field.name == "x") {
      VLOG(10) << "found x field: " << sensor_msgs::msg::to_yaml(field);
      x_parser_ = initFloatParser(field);
    } else if (field.name == "y") {
      VLOG(10) << "found y field: " << sensor_msgs::msg::to_yaml(field);
      y_parser_ = initFloatParser(field);
    } else if (field.name == "z") {
      VLOG(10) << "found z field: " << sensor_msgs::msg::to_yaml(field);
      z_parser_ = initFloatParser(field);
    } else if (field.name == "rgb" || field.name == "rgba") {
      VLOG(10) << "found color field: " << sensor_msgs::msg::to_yaml(field);
      color_parser_ = initColorParser(field);
    } else if (field.name == "label" || field.name == "ring") {
      VLOG(10) << "found label field: " << sensor_msgs::msg::to_yaml(field);
      label_parser_ = initIntParser(field);
    } else {
      VLOG(10) << "unknown field: " << sensor_msgs::msg::to_yaml(field);
    }
  }
}

bool PointcloudAdaptor::valid() const { return x_parser_ && y_parser_ && z_parser_; }

bool PointcloudAdaptor::hasLabels() const { return (label_parser_ ? true : false); }

cv::Vec3f PointcloudAdaptor::position(const uint8_t* point_ptr) const {
  return cv::Vec3f(x_parser_(point_ptr), y_parser_(point_ptr), z_parser_(point_ptr));
}

std::array<uint8_t, 4> PointcloudAdaptor::color(const uint8_t* point_ptr) const {
  if (!color_parser_) {
    return {0, 0, 0, 0};
  }

  return color_parser_(point_ptr);
}

uint32_t PointcloudAdaptor::label(const uint8_t* point_ptr) const {
  return label_parser_(point_ptr);
}

bool fillPointcloudPacket(const sensor_msgs::msg::PointCloud2& msg,
                          CloudInputPacket& packet,
                          bool instance_ids,
                          bool discard_transparent) {
  PointcloudAdaptor adaptor(msg);
  if (!adaptor.valid() || !adaptor.hasLabels()) {
    return false;
  }

  packet.points = cv::Mat(msg.height, msg.width, CV_32FC3);
  packet.colors = cv::Mat(msg.height, msg.width, CV_8UC3);
  if (discard_transparent) {
    packet.color_mask = cv::Mat(msg.height, msg.width, InputData::MaskMatType);
  }

  if (adaptor.hasLabels()) {
    packet.labels = cv::Mat(msg.height, msg.width, CV_32SC1);
    if (instance_ids) {
      packet.instances = cv::Mat(msg.height, msg.width, CV_16SC1);
    }
  }

  for (uint32_t row = 0; row < msg.height; ++row) {
    for (uint32_t col = 0; col < msg.width; ++col) {
      const auto offset = row * msg.row_step + col * msg.point_step;
      const auto point_ptr = &msg.data[offset];
      const auto color = adaptor.color(point_ptr);
      packet.points.at<cv::Vec3f>(row, col) = adaptor.position(point_ptr);
      auto& color_vec = packet.colors.at<cv::Vec3b>(row, col);
      color_vec[0] = color[0];
      color_vec[1] = color[1];
      color_vec[2] = color[2];
      if (discard_transparent) {
        packet.color_mask.at<InputData::MaskType>(row, col) = color_vec[3] > 0;
      }

      if (!adaptor.hasLabels()) {
        continue;
      }

      if (instance_ids) {
        const auto label = adaptor.label(point_ptr);
        packet.labels.at<int32_t>(row, col) = label & 0xFFFF;
        packet.instances.at<int16_t>(row, col) = label >> 16;
      } else {
        packet.labels.at<int32_t>(row, col) = adaptor.label(point_ptr);
      }
    }
  }

  return true;
}

}  // namespace hydra
