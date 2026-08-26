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

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/validation.h>

#include <Eigen/Core>
#include <cmath>
#include <limits>

#include "hydra/input/input_data.h"

namespace hydra {

using spark_dsg::Color;
using Weights = InterpolationWeights;

namespace {

static const auto nearest_registration =
    config::RegistrationWithConfig<ProjectionInterpolator,
                                   InterpolatorNearest,
                                   InterpolatorNearest::Config>("nearest");

static const auto bilinear_registration =
    config::RegistrationWithConfig<ProjectionInterpolator,
                                   InterpolatorBilinear,
                                   InterpolatorBilinear::Config>("bilinear");

static const auto adaptive_registration =
    config::RegistrationWithConfig<ProjectionInterpolator,
                                   InterpolatorAdaptive,
                                   InterpolatorAdaptive::Config>("adaptive");

inline bool pixelIsValid(int u, int v, const cv::Mat& img) {
  const auto range = img.at<InputData::RangeType>(v, u);
  return range >= 1.0e-6f && std::isfinite(range);
}

inline Weights getNearestWeights(float u, float v, const cv::Mat& img) {
  Weights weights(std::round(u), std::round(v));
  if (weights.u < 0 || weights.u >= img.cols || weights.v < 0 ||
      weights.v >= img.rows) {
    return weights;
  }

  weights.valid = pixelIsValid(weights.u, weights.v, img);
  return weights;
}

inline void updateConfidences(InputData::LabelType label,
                              float weight,
                              std::array<int, 4>& ids,
                              std::array<float, 4>& confidences,
                              size_t& allocated) {
  for (size_t i = 0; i < allocated; ++i) {
    if (ids[i] == label) {
      // increment already seen label and return
      confidences[i] += weight;
      return;
    }
  }

  // fill new slot with label and weight
  ids[allocated] = label;
  confidences[allocated] = weight;
  ++allocated;
}

}  // namespace

void declare_config(InterpolatorNearest::Config&) {
  config::name("InterpolatorNearest::Config");
}

Weights InterpolatorNearest::computeWeights(float u,
                                            float v,
                                            const cv::Mat& img) const {
  return getNearestWeights(u, v, img);
}

float InterpolatorNearest::interpolateRange(const cv::Mat& range_image,
                                            const Weights& weights) const {
  return range_image.at<InputData::RangeType>(weights.v, weights.u);
}

Color InterpolatorNearest::interpolateColor(const cv::Mat& color_image,
                                            const Weights& weights) const {
  const auto color = color_image.at<InputData::ColorType>(weights.v, weights.u);
  return Color(color[0], color[1], color[2]);
}

int InterpolatorNearest::interpolateID(const cv::Mat& id_image,
                                       const Weights& weights) const {
  return id_image.at<InputData::LabelType>(weights.v, weights.u);
}

bool InterpolatorNearest::interpolateMask(const cv::Mat& mask_image,
                                          const Weights& weights) const {
  return mask_image.at<uint8_t>(weights.v, weights.u) > 0;
}

void declare_config(InterpolatorBilinear::Config&) {
  config::name("InterpolatorBilinear::Config");
}

Weights InterpolatorBilinear::computeWeights(float u,
                                             float v,
                                             const cv::Mat& img) const {
  Weights weights(std::floor(u), std::floor(v));
  if (weights.u < 0 || weights.v < 0 || weights.u >= img.cols - 1 ||
      weights.v >= img.rows - 1) {
    return weights;
  }

  // all range values in 2x2 grid must be finite for bilinear interpolation to work
  weights.valid = pixelIsValid(weights.u, weights.v, img);
  weights.valid &= pixelIsValid(weights.u + 1, weights.v, img);
  weights.valid &= pixelIsValid(weights.u, weights.v + 1, img);
  weights.valid &= pixelIsValid(weights.u + 1, weights.v + 1, img);
  if (!weights.valid) {
    return weights;
  }

  fillBilinearWeights(u, v, weights);
  return weights;
}

float InterpolatorBilinear::interpolateRange(const cv::Mat& range_image,
                                             const Weights& weights) const {
  return range_image.at<InputData::RangeType>(weights.v, weights.u) * weights.w0 +
         range_image.at<InputData::RangeType>(weights.v + 1, weights.u) * weights.w1 +
         range_image.at<InputData::RangeType>(weights.v, weights.u + 1) * weights.w2 +
         range_image.at<InputData::RangeType>(weights.v + 1, weights.u + 1) *
             weights.w3;
}

Color InterpolatorBilinear::interpolateColor(const cv::Mat& color_image,
                                             const Weights& weights) const {
  Eigen::Vector3f color(0, 0, 0);
  const auto c1 = color_image.at<InputData::ColorType>(weights.v, weights.u);
  const auto c2 = color_image.at<InputData::ColorType>(weights.v + 1, weights.u);
  const auto c3 = color_image.at<InputData::ColorType>(weights.v, weights.u + 1);
  const auto c4 = color_image.at<InputData::ColorType>(weights.v + 1, weights.u + 1);
  for (size_t i = 0; i < 3; ++i) {
    color[i] = c1[i] * weights.w0 + c2[i] * weights.w1 + c3[i] * weights.w2 +
               c4[i] * weights.w3;
  }

  return Color(color[0], color[1], color[2]);
}

int InterpolatorBilinear::interpolateID(const cv::Mat& id_image,
                                        const Weights& weights) const {
  // Since IDs can not be interpolated we assign weights to all IDs in the image
  // based on the corner weights and return the highest weights ID.

  // first pixel will always be allocated
  size_t allocated = 1;
  std::array<int, 4> ids{
      id_image.at<InputData::LabelType>(weights.v, weights.u), 0, 0, 0};
  std::array<float, 4> confidences{weights.w0, 0.0f, 0.0f, 0.0f};

  updateConfidences(id_image.at<InputData::LabelType>(weights.v + 1, weights.u),
                    weights.w1,
                    ids,
                    confidences,
                    allocated);
  updateConfidences(id_image.at<InputData::LabelType>(weights.v, weights.u + 1),
                    weights.w2,
                    ids,
                    confidences,
                    allocated);
  updateConfidences(id_image.at<InputData::LabelType>(weights.v + 1, weights.u + 1),
                    weights.w3,
                    ids,
                    confidences,
                    allocated);

  float best_confidence = confidences[0];
  InputData::LabelType best_id = ids[0];
  for (size_t i = 1; i < allocated; ++i) {
    if (confidences[i] > best_confidence) {
      best_confidence = confidences[i];
      best_id = ids[i];
    }
  }

  return best_id;
}

bool InterpolatorBilinear::interpolateMask(const cv::Mat& img,
                                           const Weights& weights) const {
  float total = 0.0f;
  total += img.at<uint8_t>(weights.v, weights.u) > 0 ? weights.w0 : -weights.w0;
  total += img.at<uint8_t>(weights.v + 1, weights.u) > 0 ? weights.w1 : -weights.w1;
  total += img.at<uint8_t>(weights.v, weights.u + 1) > 0 ? weights.w2 : -weights.w2;
  total += img.at<uint8_t>(weights.v + 1, weights.u + 1) > 0 ? weights.w3 : -weights.w3;
  return total >= 0.0f;  // will be negative if more weight on false
}

void InterpolatorBilinear::fillBilinearWeights(float u,
                                               float v,
                                               Weights& weights) const {
  const auto du = u - static_cast<float>(weights.u);
  const auto dv = v - static_cast<float>(weights.v);

  weights.use_bilinear = true;
  weights.w0 = (1.0f - du) * (1.0f - dv);
  weights.w1 = (1.0f - du) * dv;
  weights.w2 = du * (1.0f - dv);
  weights.w3 = du * dv;
}

void declare_config(InterpolatorAdaptive::Config& config) {
  using namespace config;
  config::name("InterpolatorAdaptive::Config");
  field(config.max_depth_difference_m, "max_depth_difference_m", "m");
  check(config.max_depth_difference_m, GE, 0.0f, "max_depth_difference_m");
}

InterpolatorAdaptive::InterpolatorAdaptive(const Config& config)
    : InterpolatorBilinear(InterpolatorBilinear::Config()),
      config(config::checkValid(config)) {}

Weights InterpolatorAdaptive::computeWeights(float u,
                                             float v,
                                             const cv::Mat& ranges) const {
  const int row = std::floor(v);
  const int col = std::floor(u);

  // Check max range difference.
  bool use_nearest = false;
  float min = std::numeric_limits<float>::max();
  float max = 0.0f;
  for (size_t i = 0; i < 4; ++i) {
    const auto curr_v = row + v_offset_[i];
    const auto curr_u = col + u_offset_[i];
    if (curr_v < 0 || curr_u < 0 || curr_v >= ranges.rows || curr_u >= ranges.cols) {
      use_nearest = true;  // use nearest when at corner of imaage
      break;
    }

    const float range = ranges.at<InputData::RangeType>(curr_v, curr_u);
    if (range < 1.0e-6f || !std::isfinite(range)) {
      use_nearest = true;  // use nearest when one pixel is invalid
      break;
    }

    max = std::max(range, max);
    min = std::min(range, min);
    if (max - min > config.max_depth_difference_m) {
      use_nearest = true;  // use nearest when adaptive check is hit
      break;
    }
  }

  if (use_nearest) {
    // will handle validating pixel at [u, v]
    return getNearestWeights(u, v, ranges);
  }

  Weights weights(col, row);
  weights.valid = true;
  fillBilinearWeights(u, v, weights);
  return weights;
}

float InterpolatorAdaptive::interpolateRange(const cv::Mat& range_image,
                                             const Weights& weights) const {
  if (weights.use_bilinear) {
    return InterpolatorBilinear::interpolateRange(range_image, weights);
  }

  return range_image.at<InputData::RangeType>(weights.v, weights.u);
}

Color InterpolatorAdaptive::interpolateColor(const cv::Mat& color_image,
                                             const Weights& weights) const {
  if (weights.use_bilinear) {
    return InterpolatorBilinear::interpolateColor(color_image, weights);
  }

  auto color = color_image.at<InputData::ColorType>(weights.v, weights.u);
  return Color(color[0], color[1], color[2]);
}

int InterpolatorAdaptive::interpolateID(const cv::Mat& id_image,
                                        const Weights& weights) const {
  if (weights.use_bilinear) {
    return InterpolatorBilinear::interpolateID(id_image, weights);
  }

  return id_image.at<InputData::LabelType>(weights.v, weights.u);
}

bool InterpolatorAdaptive::interpolateMask(const cv::Mat& img,
                                           const Weights& weights) const {
  if (weights.use_bilinear) {
    return InterpolatorBilinear::interpolateMask(img, weights);
  }

  return img.at<uint8_t>(weights.v, weights.u) > 0;
}

}  // namespace hydra
