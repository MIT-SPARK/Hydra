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
#pragma once

#include <config_utilities/factory.h>

#include <memory>
#include <opencv2/core/mat.hpp>
#include <string>

#include "hydra/common/common_types.h"

namespace hydra {

struct InterpolationWeights {
  bool valid = false;
  int u;
  int v;
  float w0, w1, w2, w3;
  bool use_bilinear = false;

  InterpolationWeights() = default;
  InterpolationWeights(int _u, int _v) : u(_u), v(_v) {}
};

/**
 * @brief Interface for different ways to interpolate the values in images.
 * Use computeWeights() first, then use the other functions interpolate relevant data.
 */
class ProjectionInterpolator {
 public:
  virtual ~ProjectionInterpolator() = default;

  /**
   * @brief Computes the weight of neighboring pixels for a query point. This method
   * assumes that u and v are fully within the bounds of the range image, which is not
   * re-checked.
   * @param u Horizontal position in image space of the point to interpolate.
   * @param v Vertical position in image space of the point to interpolate.
   * @param range_image Range image as 32FC1 to compute weights.
   * @return The weights used to interpolate further quantities in other images.
   */
  virtual InterpolationWeights computeWeights(float u,
                                              float v,
                                              const cv::Mat& range_image) const = 0;

  /**
   * @brief Compute the depth based on the provided weights.
   * @param range_image Range image as 32FC1 to interpolate in.
   * @return float The interpolated range value.
   */
  virtual float InterpolateRange(const cv::Mat& range_image,
                                 const InterpolationWeights& weights) const = 0;

  /**
   * @brief Compute the color based on the provided weights.
   * @param color_image Color image as RGB8 to interpolate in.
   * @return Color The interpolated color value.
   */
  virtual Color interpolateColor(const cv::Mat& color_image,
                                 const InterpolationWeights& weights) const = 0;

  /**
   * @brief Compute the semantic id based on the provided weights.
   * @param id_image ID image as 8UC1 to interpolate in.
   * @return int The interpolated ID value.
   */
  virtual int interpolateID(const cv::Mat& id_image,
                            const InterpolationWeights& weights) const = 0;
};

/**
 * @brief Interpolates values by selecting the nearest neighbor. Use
 computeWeights() first, then use the other functions to interpolate the relevant data.
 */
class InterpolatorNearest : public ProjectionInterpolator {
 public:
  InterpolationWeights computeWeights(float u,
                                      float v,
                                      const cv::Mat& range_image) const override;

  float InterpolateRange(const cv::Mat& range_image,
                         const InterpolationWeights& weights) const override;

  Color interpolateColor(const cv::Mat& color_image,
                         const InterpolationWeights& weights) const override;

  int interpolateID(const cv::Mat& id_image,
                    const InterpolationWeights& weights) const override;

 private:
  inline static const auto registration_ =
      config::Registration<ProjectionInterpolator, InterpolatorNearest>("nearest");
};

/**
 * @brief Interpolates values using bilinear interpolation. Use computeWeights()
 first, then use the other functions to interpolate relevant data.
 */
class InterpolatorBilinear : public ProjectionInterpolator {
 public:
  InterpolationWeights computeWeights(float u,
                                      float v,
                                      const cv::Mat& range_image) const override;

  float InterpolateRange(const cv::Mat& range_image,
                         const InterpolationWeights& weights) const override;

  Color interpolateColor(const cv::Mat& color_image,
                         const InterpolationWeights& weights) const override;
  int interpolateID(const cv::Mat& id_image,
                    const InterpolationWeights& weights) const override;

 private:
  inline static const auto registration_ =
      config::Registration<ProjectionInterpolator, InterpolatorBilinear>("bilinear");
};

/**
 * @brief Use bilinear interpolation if the range values are all close,
 * otherwise use nearest neighbor lookup. This behavior should capture surface
 * discontinuities which would otherwise be interpolated. Use computeWeights()
 first, then use the other functions to interpolate the relevant data.
 */
class InterpolatorAdaptive : public InterpolatorBilinear {
 public:
  InterpolationWeights computeWeights(float u,
                                      float v,
                                      const cv::Mat& range_image) const override;

  float InterpolateRange(const cv::Mat& range_image,
                         const InterpolationWeights& weights) const override;

  Color interpolateColor(const cv::Mat& color_image,
                         const InterpolationWeights& weights) const override;

  int interpolateID(const cv::Mat& id_image,
                    const InterpolationWeights& weights) const override;

 private:
  inline static const auto registration_ =
      config::Registration<ProjectionInterpolator, InterpolatorAdaptive>("adaptive");
  const int u_offset_[4] = {0, 0, 1, 1};
  const int v_offset_[4] = {0, 1, 0, 1};
};

}  // namespace hydra
