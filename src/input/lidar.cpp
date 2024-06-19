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
#include "hydra/input/lidar.h"

#include <config_utilities/config_utilities.h>
#include <glog/logging.h>

#include <opencv2/core.hpp>
#include <unordered_map>
#include <vector>

#include "hydra/input/sensor_utilities.h"

namespace hydra {

void declare_config(Lidar::Config& config) {
  using namespace config;
  name("Lidar");
  base<Sensor::Config>(config);
  field(config.horizontal_resolution, "horizontal_resolution", "points/degrees");
  field(config.vertical_resolution, "vertical_resolution", "points/degrees");
  field(config.horizontal_fov, "horizontal_fov", "degrees");
  field(config.vertical_fov, "vertical_fov", "degrees");
  field(config.is_asymmetric, "is_asymmetric");
  if (config.is_asymmetric) {
    field(config.vertical_fov_top, "vertical_fov_top", "degrees");
  }

  check(config.horizontal_resolution, GT, 0, "horizontal_resolution");
  check(config.vertical_resolution, GT, 0, "vertical_resolution");
  check(config.horizontal_fov, GT, 0, "horizontal_fov");
  check(config.vertical_fov, GT, 0, "vertical_fov");
}

Lidar::Lidar(const Config& config)
    : Sensor(config),
      config_(config::checkValid(config)),
      width_(config_.horizontal_fov / config_.horizontal_resolution),
      height_(config_.vertical_fov / config_.vertical_resolution),
      vertical_fov_rad_(config_.vertical_fov * M_PI / 180.0f),
      vertical_fov_top_rad_(config_.vertical_fov_top * M_PI / 180.0f),
      horizontal_fov_rad_(config_.horizontal_fov * M_PI / 180.0f) {
  // compute upper phi limit and associated z at unit focal distance
  const auto phi_up =
      config_.is_asymmetric ? vertical_fov_top_rad_ : vertical_fov_rad_ / 2.0f;
  const auto z_up = std::tan(phi_up);
  // left upper x right upper corner
  top_frustum_normal_ = Eigen::Vector3f(1.0f, 1.0f, z_up)
                            .cross(Eigen::Vector3f(1.0, -1.0f, z_up))
                            .normalized();

  // compute lower phi limit and associated z at unit focal distance
  const auto phi_down = config_.is_asymmetric
                            ? vertical_fov_top_rad_ - vertical_fov_rad_
                            : -vertical_fov_rad_ / 2.0f;
  const auto z_down = std::tan(phi_down);
  // right lower x left lower corner
  bottom_frustum_normal_ = Eigen::Vector3f(1.0f, -1.0f, z_down)
                               .cross(Eigen::Vector3f(1.0, 1.0f, z_down))
                               .normalized();

  const auto half_fov = horizontal_fov_rad_ / 2.0f;
  // compute left theta extent and flip if greater than 90 degrees
  const auto theta_left = half_fov >= M_PI / 2.0f ? M_PI - half_fov : half_fov;
  // flip associated unit focal length if required and compute actual coordinates
  const auto x = half_fov >= M_PI / 2.0f ? -1.0f : 1.0f;
  const auto y_left = std::tan(theta_left);
  // left lower x left upper
  left_frustum_normal_ = Eigen::Vector3f(x, y_left, -1.0f)
                             .cross(Eigen::Vector3f(x, y_left, 1.0f))
                             .normalized();
  // right upper x right lower
  right_frustum_normal_ = Eigen::Vector3f(x, -y_left, 1.0f)
                              .cross(Eigen::Vector3f(x, -y_left, -1.0f))
                              .normalized();
}

float Lidar::computeRayDensity(float voxel_size, float depth) const {
  // we want rays per meter... we can do this by computing a virtual focal length
  // compute focal lengths based on percent of spherical image inside 90 degree FOV
  // focal_length = (dim / 2) / tan(fov / 2) and tan(fov / 2) = 1
  const auto virtual_fx = (width_ * 90.0 / config_.horizontal_fov) / 2.0;
  const auto virtual_fy = (height_ * 90.0 / config_.vertical_fov) / 2.0;
  const auto voxel_density = voxel_size / depth;
  return virtual_fx * virtual_fy * voxel_density * voxel_density;
}

bool Lidar::finalizeRepresentations(InputData& input, bool force_world_frame) const {
  if (input.vertex_map.empty()) {
    LOG(ERROR) << "pointcloud required to finalize data!";
    return false;
  }

  // TODO(nathan) check that input is normalized

  if (input.vertex_map.size() != input.label_image.size()) {
    LOG(ERROR) << "label input size does not match pointcloud!";
    return false;
  }

  if (!input.color_image.empty() &&
      input.vertex_map.size() != input.color_image.size()) {
    LOG(ERROR) << "color input size does not match pointcloud!";
    return false;
  }

  // TODO(nathan) detect structured
  // TODO(nathan) think about structured points

  // TODO(nathan) test
  if (force_world_frame && !input.points_in_world_frame) {
    const auto world_T_sensor = input.getSensorPose().cast<float>();
    auto point_iter = input.vertex_map.begin<cv::Vec3f>();
    while (point_iter != input.vertex_map.end<cv::Vec3f>()) {
      auto& p = *point_iter;
      Eigen::Vector3f p_S(p[0], p[1], p[2]);
      const auto p_W = world_T_sensor * p_S;
      p[0] = p_W.x();
      p[1] = p_W.y();
      p[2] = p_W.z();
    }

    input.points_in_world_frame = true;
  }

  const auto sensor_T_world = input.getSensorPose().cast<float>().inverse();
  input.min_range = std::numeric_limits<float>::max();
  input.max_range = std::numeric_limits<float>::lowest();

  bool has_color = !input.color_image.empty();
  input.range_image = cv::Mat(height_, width_, CV_32FC1, 0.0f);
  cv::Mat labels(height_, width_, CV_32SC1, -1);
  cv::Mat color;
  if (has_color) {
    color = cv::Mat(height_, width_, CV_8UC3);
    color = 0;
  }

  auto point_iter = input.vertex_map.begin<cv::Vec3f>();
  auto label_iter = input.label_image.begin<int32_t>();
  size_t num_invalid = 0;
  size_t color_index = 0;
  while (point_iter != input.vertex_map.end<cv::Vec3f>()) {
    int u, v;
    const auto& p = *point_iter;
    Eigen::Vector3f p_C(p[0], p[1], p[2]);
    if (input.points_in_world_frame) {
      p_C = sensor_T_world * p_C;
    }

    if (!projectPointToImagePlane(p_C, u, v)) {
      ++num_invalid;
      ++point_iter;
      ++label_iter;
      ++color_index;
      continue;
    }

    const auto range_m = p_C.norm();
    input.min_range = std::min(input.min_range, range_m);
    input.max_range = std::max(input.max_range, range_m);

    input.range_image.at<float>(v, u) = p_C.norm();
    labels.at<int32_t>(v, u) = *label_iter;
    if (has_color) {
      color.at<cv::Vec3b>(v, u) = input.color_image.at<cv::Vec3b>(color_index);
    }

    ++point_iter;
    ++label_iter;
    ++color_index;
  }

  size_t total_lidar = input.vertex_map.rows * input.vertex_map.cols;
  double percent_invalid = static_cast<double>(num_invalid) / total_lidar;
  VLOG(5) << "Converted lidar points! invalid: " << num_invalid << " / " << total_lidar
          << " (percent: " << percent_invalid << ")";
  input.label_image = labels;
  input.color_image = color;
  return true;
}

// TODO(nathan) this isn't correct for all asymmetric cases
bool Lidar::projectPointToImagePlane(const Eigen::Vector3f& p_C,
                                     float& u,
                                     float& v) const {
  if (p_C.norm() <= config_.min_range) {
    return false;
  }

  // map lidar point to [0, w] x [0, h]
  // assumes forward-left-up and fov center aligned with x-axis for a spherical model
  const auto bearing = p_C.normalized();
  const auto phi = std::asin(bearing.z());
  const auto theta = std::atan2(bearing.y(), bearing.x());

  if (config_.is_asymmetric) {
    // phi is [-pi/2, pi/2], ratio is [1, 0], maps to [height, 0]
    const auto vertical_ratio = (vertical_fov_top_rad_ - phi) / vertical_fov_rad_;
    v = height_ * vertical_ratio;
  } else {
    // phi is [-pi/2, pi/2], ratio is [1, 0], maps to [height, 0]
    const auto vertical_ratio = (vertical_fov_rad_ / 2.0f - phi) / vertical_fov_rad_;
    v = height_ * vertical_ratio;
  }

  if (v < 0.0f || v > height_) {
    return false;
  }

  const auto h_ratio = (horizontal_fov_rad_ / 2.0f - theta) / horizontal_fov_rad_;
  u = width_ * h_ratio;
  if (u < 0.0f || u > width_) {
    return false;
  }

  return true;
}

bool Lidar::projectPointToImagePlane(const Eigen::Vector3f& p_C, int& u, int& v) const {
  float temp_u = -1.0f;
  float temp_v = -1.0f;
  if (!projectPointToImagePlane(p_C, temp_u, temp_v)) {
    return false;
  }

  // assumption is pixel indices point to top left corner
  u = std::floor(temp_u);
  v = std::floor(temp_v);
  if (u >= width_ || u < 0 || v >= height_ || v < 0) {
    return false;
  }

  return true;
}

bool Lidar::pointIsInViewFrustum(const Eigen::Vector3f& point_C,
                                 float inflation_distance) const {
  if (point_C.norm() > config_.max_range + inflation_distance) {
    return false;
  }

  double radius_2d = std::sqrt(point_C.x() * point_C.x() + point_C.y() * point_C.y());
  Eigen::Vector3f point_C_2d(radius_2d, 0, point_C.z());
  if (point_C_2d.dot(top_frustum_normal_) < -inflation_distance) {
    return false;
  }

  if (point_C_2d.dot(bottom_frustum_normal_) < -inflation_distance) {
    return false;
  }

  const auto left_prod = point_C.dot(left_frustum_normal_);
  const auto right_prod = point_C.dot(right_frustum_normal_);
  if (horizontal_fov_rad_ <= M_PI) {
    // normal camera or half-plane case
    return left_prod >= -inflation_distance && right_prod >= -inflation_distance;
  }

  // check to make sure that we're not in the exluded region
  return !(left_prod <= -inflation_distance && right_prod <= -inflation_distance);
}

}  // namespace hydra
