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
#include "hydra/input/input_data.h"

#include <config_utilities/config.h>
#include <config_utilities/types/eigen_matrix.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <utility>

#include "hydra/common/global_info.h"
#include "hydra/input/input_data_io.h"

namespace hydra {
namespace {

inline std::string showTypeInfo(const cv::Mat& mat) {
  std::stringstream ss;
  ss << "{depth: " << mat.depth() << ", channels: " << mat.channels() << "}";
  return ss.str();
}

bool convertDepth(InputData& data) {
  if (data.depth_image.empty()) {
    return true;
  }

  if (data.depth_image.channels() != 1) {
    LOG(ERROR) << "depth image must be single-channel";
    return false;
  }

  if (data.depth_image.type() == CV_32FC1) {
    return true;  // nothing else to do
  }

  if (data.depth_image.type() != CV_16UC1) {
    LOG(ERROR) << "only CV_32FC1 or CV_16UC1 formats supported, not "
               << showTypeInfo(data.depth_image);
    return false;
  }

  cv::Mat depth_converted;
  data.depth_image.convertTo(depth_converted, CV_32FC1, 1.0e-3);
  data.depth_image = depth_converted;
  return true;
}

bool convertColor(InputData& data) {
  if (data.color_image.empty()) {
    return true;
  }

  if (data.color_image.type() != InputData::ColorMatType) {
    LOG(ERROR) << "only 3-channel rgb images supported";
    return false;
  }

  return true;
}

bool convertLabels(InputData& data) {
  if (data.label_image.empty()) {
    return false;
  }

  // Enforcing requirement for int32_t at this point
  if (data.label_image.type() != InputData::LabelMatType) {
    cv::Mat new_label_image(data.label_image.size(), InputData::LabelMatType);
    data.label_image.convertTo(new_label_image, InputData::LabelMatType);
    data.label_image = new_label_image;
  }

  const auto remap = GlobalInfo::instance().getLabelRemapper();
  if (remap) {
    remap.remapImage(data.label_image);
  }

  return true;
}

void convertVertexMap(InputData& data, bool in_world_frame) {
  if (data.points_in_world_frame == in_world_frame) {
    return;
  }

  Eigen::Isometry3f transform = data.getSensorPose().cast<float>();  // world_T_sensor
  if (!in_world_frame) {
    transform = transform.inverse();  // Instead get sensor_T_world
  }

  for (int r = 0; r < data.vertex_map.rows; ++r) {
    for (int c = 0; c < data.vertex_map.cols; ++c) {
      auto& point = data.vertex_map.at<InputData::VertexType>(r, c);
      Eigen::Vector3f point_eigen(point[0], point[1], point[2]);
      point_eigen = transform * point_eigen;
      point[0] = point_eigen.x();
      point[1] = point_eigen.y();
      point[2] = point_eigen.z();
    }
  }

  data.points_in_world_frame = in_world_frame;
}

}  // namespace

void declare_config(InputData::SaveOptions& config) {
  using namespace config;
  name("InputData::SaveOptions");
  enum_field(config.float_compression, "float_compression", {"none", "rle", "zip"});
  field(config.png_compression, "png_compression");
  field(config.archive_compression, "archive_compression");
  checkInRange(config.png_compression, 0, 9, "png_compression");
  checkInRange(config.archive_compression, -1, 9, "archive_compression");
}

InputData::InputData(Sensor::ConstPtr sensor) : sensor_(std::move(sensor)) {}

InputData::Ptr InputData::clone() const {
  auto copy = std::make_shared<InputData>(*this);
  copy->color_image = color_image.clone();
  copy->color_mask = color_mask.clone();
  copy->depth_image = depth_image.clone();
  copy->range_image = range_image.clone();
  copy->label_image = label_image.clone();
  copy->instance_image = instance_image.clone();
  copy->vertex_map = vertex_map.clone();
  copy->traversability_image = traversability_image.clone();
  return copy;
}

const Sensor& InputData::getSensor() const {
  if (!sensor_) {
    throw std::runtime_error("InputData has no sensor");
  }
  return *sensor_;
}

Eigen::Isometry3d InputData::getSensorPose() const {
  return world_T_body * sensor_->body_T_sensor();
}

bool InputData::inRange(float range_m) const {
  return range_m >= sensor_->min_range() && range_m <= sensor_->max_range() &&
         range_m <= max_range;
}

bool InputData::finalize(bool vertices_in_world_frame, bool normalize_labels) {
  if (!convertDepth(*this)) {
    LOG(ERROR) << "[Input Conversion] Unable to normalize depth";
    return false;
  }

  if (!convertColor(*this)) {
    LOG(ERROR) << "[Input Conversion] Unable to normalize color";
    return false;
  }

  if (normalize_labels && !convertLabels(*this)) {
    LOG(ERROR) << "[Input Conversion] Unable to normalize labels";
    return false;
  }

  if (!instance_image.empty() && instance_image.type() != InputData::InstanceMatType) {
    cv::Mat instances(instance_image.size(), InputData::InstanceMatType);
    instance_image.convertTo(instances, InputData::InstanceMatType);
    instance_image = instances;
  }

  // Integer traversability images are class ids (e.g., from a traversability
  // segmentation network) and are not in the semantic labelspace, so no remapping.
  // Floating point images are continuous estimates and are left as is.
  if (!traversability_image.empty() && traversability_image.channels() == 1 &&
      traversability_image.depth() < CV_32F &&
      traversability_image.type() != InputData::LabelMatType) {
    cv::Mat traversability(traversability_image.size(), InputData::LabelMatType);
    traversability_image.convertTo(traversability, InputData::LabelMatType);
    traversability_image = traversability;
  }

  if (!vertex_map.empty() && vertex_map.type() != InputData::VertexMatType) {
    LOG(ERROR) << "[Input Conversion] pointcloud must be CV_32FC3, not "
               << showTypeInfo(vertex_map);
    return false;
  }

  if (!sensor_->finalizeRepresentations(*this)) {
    LOG(ERROR) << "[Input Conversion] Unable to compute inputs for integration";
    return false;
  }

  convertVertexMap(*this, vertices_in_world_frame);
  return true;
}

void InputData::save(const std::filesystem::path& filepath) const {
  save(filepath, SaveOptions{});
}

void InputData::save(const std::filesystem::path& filepath,
                     const SaveOptions& options) const {
  const io::ArchiveOptions archive{options.archive_compression};
  io::writeArchive(
      filepath,
      [this, &options](const auto& write) {
        input::writeInputData(*this, write, options);
      },
      archive);
}

InputData::Ptr InputData::load(const std::filesystem::path& filepath) {
  InputData::Ptr input;
  io::readArchive(filepath,
                  [&input](const auto& read) { input = input::readInputData(read); });
  return input;
}

}  // namespace hydra
