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
#include "hydra/reconstruction/projection_interpolators.h"

#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace hydra {


InterpolationWeights InterpolatorNearest::computeWeights(float u,
                                                         float v,
                                                         const cv::Mat& img) const {
  InterpolationWeights weights(std::round(u), std::round(v));
  if (weights.u < 0 || weights.u >= img.cols || weights.v < 0 ||
      weights.v >= img.rows) {
    return weights;
  }

  weights.valid = true;
  return weights;
}

float InterpolatorNearest::InterpolateRange(const cv::Mat& range_image,
                                            const InterpolationWeights& weights) const {
  return range_image.at<float>(weights.v, weights.u);
}

Color InterpolatorNearest::interpolateColor(const cv::Mat& color_image,
                                            const InterpolationWeights& weights) const {
  const cv::Vec3b color = color_image.at<cv::Vec3b>(weights.v, weights.u);
  return Color(color[0], color[1], color[2]);
}

int InterpolatorNearest::interpolateID(const cv::Mat& id_image,
                                       const InterpolationWeights& weights) const {
  return id_image.at<int32_t>(weights.v, weights.u);
}

InterpolationWeights InterpolatorBilinear::computeWeights(float u,
                                                          float v,
                                                          const cv::Mat& img) const {
  InterpolationWeights weights(std::floor(u), std::floor(v));
  if (weights.u < 0 || weights.v < 0 || weights.u >= img.cols - 1 ||
      weights.v >= img.rows - 1) {
    return weights;
  }

  weights.valid = true;
  weights.use_bilinear = true;
  float du = u - static_cast<float>(weights.u);
  float dv = v - static_cast<float>(weights.v);
  weights.w0 = (1.f - du) * (1.f - dv);
  weights.w1 = (1.f - du) * dv;
  weights.w2 = du * (1.f - dv);
  weights.w3 = du * dv;
  return weights;
}

float InterpolatorBilinear::InterpolateRange(
    const cv::Mat& range_image, const InterpolationWeights& weights) const {
  return range_image.at<float>(weights.v, weights.u) * weights.w0 +
         range_image.at<float>(weights.v + 1, weights.u) * weights.w1 +
         range_image.at<float>(weights.v, weights.u + 1) * weights.w2 +
         range_image.at<float>(weights.v + 1, weights.u + 1) * weights.w3;
}

Color InterpolatorBilinear::interpolateColor(
    const cv::Mat& color_image, const InterpolationWeights& weights) const {
  Eigen::Vector3f color(0, 0, 0);
  auto c1 = color_image.at<cv::Vec3b>(weights.v, weights.u);
  auto c2 = color_image.at<cv::Vec3b>(weights.v + 1, weights.u);
  auto c3 = color_image.at<cv::Vec3b>(weights.v, weights.u + 1);
  auto c4 = color_image.at<cv::Vec3b>(weights.v + 1, weights.u + 1);
  for (size_t i = 0; i < 3; ++i) {
    color[i] = c1[i] * weights.w0 + c2[i] * weights.w1 + c3[i] * weights.w2 +
               c4[i] * weights.w3;
  }
  return Color(color[0], color[1], color[2]);
}

int InterpolatorBilinear::interpolateID(const cv::Mat& id_image,
                                        const InterpolationWeights& weights) const {
  // Since IDs can not be interpolated we assign weights to all IDs in the image
  // based on the corner weights and return  the highest weights ID.
  std::unordered_map<int, float> ids;  // These are zero initialized by default.
  ids[id_image.at<int32_t>(weights.v, weights.u)] += weights.w0;
  ids[id_image.at<int32_t>(weights.v + 1, weights.u)] += weights.w1;
  ids[id_image.at<int32_t>(weights.v, weights.u + 1)] += weights.w2;
  ids[id_image.at<int32_t>(weights.v + 1, weights.u + 1)] += weights.w3;
  return std::max_element(
             std::begin(ids),
             std::end(ids),
             [](const auto& p1, const auto& p2) { return p1.second < p2.second; })
      ->first;
}

InterpolationWeights InterpolatorAdaptive::computeWeights(
    float u, float v, const cv::Mat& range_image) const {
  // NOTE(lschmid): This is currently hard coded. Should probably be a param or similar.
  constexpr float max_depth_difference = 0.2;  // m

  InterpolationWeights weights(std::floor(u), std::floor(v));
  // Check max range difference.
  float min = std::numeric_limits<float>::max();
  float max = std::numeric_limits<float>::lowest();
  for (size_t i = 0; i < 4; ++i) {
    const auto curr_v = weights.v + v_offset_[i];
    const auto curr_u = weights.u + u_offset_[i];
    if (curr_v < 0 || curr_u < 0 || curr_v >= range_image.rows ||
        curr_u >= range_image.cols) {
      return weights;
    }

    const float range = range_image.at<float>(curr_v, curr_u);
    max = std::max(range, max);
    min = std::min(range, min);
    if (max - min > max_depth_difference) {
      // interpolate nearest (manually)
      InterpolationWeights new_weights(std::round(u), std::round(v));
      new_weights.valid = true;
      return new_weights;
    }
  }

  return InterpolatorBilinear::computeWeights(u, v, range_image);
}

float InterpolatorAdaptive::InterpolateRange(
    const cv::Mat& range_image, const InterpolationWeights& weights) const {
  if (weights.use_bilinear) {
    return InterpolatorBilinear::InterpolateRange(range_image, weights);
  }
  return range_image.at<float>(weights.v, weights.u);
}

Color InterpolatorAdaptive::interpolateColor(
    const cv::Mat& color_image, const InterpolationWeights& weights) const {
  if (weights.use_bilinear) {
    return InterpolatorBilinear::interpolateColor(color_image, weights);
  }
  auto color = color_image.at<cv::Vec3b>(weights.v, weights.u);
  return Color(color[0], color[1], color[2]);
}

int InterpolatorAdaptive::interpolateID(const cv::Mat& id_image,
                                        const InterpolationWeights& weights) const {
  if (weights.use_bilinear) {
    return InterpolatorBilinear::interpolateID(id_image, weights);
  }
  return id_image.at<int32_t>(weights.v, weights.u);
}

}  // namespace hydra
