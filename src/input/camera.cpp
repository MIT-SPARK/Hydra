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
#include "hydra/input/camera.h"

#include <config_utilities/config_utilities.h>

#include <unordered_map>
#include <vector>

#include "hydra/input/sensor_utilities.h"

namespace hydra {

void declare_config(Camera::Config& config) {
  using namespace config;
  name("Camera");
  base<Sensor::Config>(config);
  field(config.width, "width", "px");
  field(config.height, "height", "px");
  field(config.cx, "cx", "px");
  field(config.cy, "cy", "px");
  field(config.fx, "fx", "px");
  field(config.fy, "fy", "px");

  check(config.width, GT, 0, "width");
  check(config.height, GT, 0, "height");
  check(config.cx, GT, 0, "cx");
  check(config.cy, GT, 0, "cy");
  check(config.fx, GT, 0, "fx");
  check(config.fy, GT, 0, "fy");
  checkCondition(config.cx <= config.width, "param 'cx' is expected <= 'width'");
  checkCondition(config.cy <= config.height, "param 'cy' is expected <= 'height'");
}

Camera::Camera(const Config& config)
    : Sensor(config), config_(config::checkValid(config)) {
  // Pre-compute the view frustum (top, right, bottom, left, plane normals).
  const auto scale_factor = config_.fx / config_.fy;
  Eigen::Vector3f p1(-config_.cx, -config_.cy * scale_factor, config_.fx);
  Eigen::Vector3f p2(
      config_.width - config_.cx, -config_.cy * scale_factor, config_.fx);
  view_frustum_.row(0) = p1.cross(p2).normalized();

  p1 = Eigen::Vector3f(config_.width - config_.cx,
                       (config_.height - config_.cy) * scale_factor,
                       config_.fx);
  view_frustum_.row(1) = p2.cross(p1).normalized();

  p2 = Eigen::Vector3f(
      -config_.cx, (config_.height - config_.cy) * scale_factor, config_.fx);
  view_frustum_.row(2) = p1.cross(p2).normalized();

  p1 = Eigen::Vector3f(-config_.cx, -config_.cy * scale_factor, config_.fx);
  view_frustum_.row(3) = p2.cross(p1).normalized();
}

float Camera::computeRayDensity(float voxel_size, float depth) const {
  return config_.fx * config_.fy * std::pow(voxel_size / depth, 2.f);
}

bool Camera::finalizeRepresentations(InputData& input, bool force_world_frame) const {
  if (!input.vertex_map.empty()) {
    input.range_image = computeRangeImageFromPoints(
        input.vertex_map, &input.min_range, &input.max_range);
    // TODO(nathan) depth image?
    return true;
  }

  if (input.depth_image.empty()) {
    return false;
  }

  const auto world_T_camera = input.getSensorPose().cast<float>();
  input.vertex_map = computeVertexMap(input.depth_image,
                                      force_world_frame ? &world_T_camera : nullptr);
  if (force_world_frame) {
    input.points_in_world_frame = true;
  }

  input.range_image =
      computeRangeImage(input.depth_image, &input.min_range, &input.max_range);
  return true;
}

bool Camera::projectPointToImagePlane(const Eigen::Vector3f& p_C,
                                      float& u,
                                      float& v) const {
  if (p_C.z() <= 0.f) {
    return false;
  }

  // all points are considered valid as long as the are contained in the image plane
  // with bounds [0, w] x [0, h]
  u = p_C.x() * config_.fx / p_C.z() + config_.cx;
  if (u > config_.width || u < 0) {
    return false;
  }

  v = p_C.y() * config_.fy / p_C.z() + config_.cy;
  if (v > config_.height || v < 0) {
    return false;
  }

  return true;
}

bool Camera::projectPointToImagePlane(const Eigen::Vector3f& p_C,
                                      int& u,
                                      int& v) const {
  float u_float = -1.0f;
  float v_float = -1.0f;
  if (!projectPointToImagePlane(p_C, u_float, v_float)) {
    return false;
  }

  u = std::floor(u_float);
  v = std::floor(v_float);
  if (u >= config_.width || u < 0 || v >= config_.height || v < 0) {
    return false;
  }

  return true;
}

bool Camera::pointIsInViewFrustum(const Eigen::Vector3f& point_C,
                                  float inflation_distance) const {
  if (point_C.z() < -inflation_distance) {
    return false;
  }

  if (point_C.norm() > config_.max_range + inflation_distance) {
    return false;
  }

  for (int i = 0; i < view_frustum_.rows(); ++i) {
    if (point_C.dot(view_frustum_.row(i)) < -inflation_distance) {
      return false;
    }
  }

  return true;
}

cv::Mat Camera::computeVertexMap(const cv::Mat& depth_image,
                                 const Eigen::Isometry3f* T_W_C) const {
  // Compute the 3D pointcloud from a depth image.
  cv::Mat vertices(depth_image.size(), CV_32FC3);
  const float fx_inv = 1.f / config_.fx;
  const float fy_inv = 1.f / config_.fy;
  for (int v = 0; v < depth_image.rows; v++) {
    for (int u = 0; u < depth_image.cols; u++) {
      const float depth = depth_image.at<float>(v, u);
      const Eigen::Vector3f p_C(depth * (static_cast<float>(u) - config_.cx) * fx_inv,
                                depth * (static_cast<float>(v) - config_.cy) * fy_inv,
                                depth);
      const auto p_W = T_W_C ? ((*T_W_C) * p_C).eval() : p_C;
      VLOG(15) << "(" << u << ", " << v << "), d=" << depth << ", fx=" << fx_inv
               << ", fy=" << fy_inv << ", cx=" << config_.cx << ", cy=" << config_.cy
               << " -> " << p_W.transpose();
      auto& vertex = vertices.at<cv::Vec3f>(v, u);
      vertex[0] = p_W.x();
      vertex[1] = p_W.y();
      vertex[2] = p_W.z();
    }
  }

  return vertices;
}

cv::Mat Camera::computeRangeImage(const cv::Mat& depth_image,
                                  float* min_range,
                                  float* max_range) const {
  // Compute the range (=radial distance) from the pointcloud.
  cv::Mat range_image(depth_image.size(), CV_32FC1);
  const float fx_inv = 1.f / config_.fx;
  const float fy_inv = 1.f / config_.fy;
  if (min_range) {
    *min_range = std::numeric_limits<float>::max();
  }

  if (max_range) {
    *max_range = std::numeric_limits<float>::lowest();
  }

  for (int v = 0; v < depth_image.rows; v++) {
    for (int u = 0; u < depth_image.cols; u++) {
      const float depth = depth_image.at<float>(v, u);
      const Eigen::Vector3f p_C(depth * (static_cast<float>(u) - config_.cx) * fx_inv,
                                depth * (static_cast<float>(v) - config_.cy) * fy_inv,
                                depth);
      const float range = p_C.norm();
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
