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
#include <gtest/gtest.h>
#include <hydra/reconstruction/projection_interpolators.h>

#include <opencv2/core.hpp>

namespace hydra {
namespace {

std::optional<float> getRange(const ProjectionInterpolator& interp,
                              const cv::Mat& mat,
                              float u,
                              float v) {
  const auto weights = interp.computeWeights(u, v, mat);
  if (!weights.valid) {
    return std::nullopt;
  }

  return interp.interpolateRange(mat, weights);
}

}  // namespace

TEST(ProjectionInterpolators, NearestCorrect) {
  InterpolatorNearest interp;
  cv::Mat img = (cv::Mat_<float>(2, 2) << 1.0, 2.0, 3.0, 4.0);

  {  // outside of image -> invalid range
    const auto range = getRange(interp, img, -1.0, 2.0);
    EXPECT_FALSE(range);
  }

  {  // rounding gives us 0, 0
    const auto range = getRange(interp, img, 0.2, 0.2);
    ASSERT_TRUE(range);
    EXPECT_EQ(range.value(), 1.0);
  }

  {  // rounding gives us 1, 0
    const auto range = getRange(interp, img, 0.2, 0.8);
    ASSERT_TRUE(range);
    EXPECT_EQ(range.value(), 3.0);
  }

  {  // rounding gives us 0, 1
    const auto range = getRange(interp, img, 0.8, 0.2);
    ASSERT_TRUE(range);
    EXPECT_EQ(range.value(), 2.0);
  }
}

TEST(ProjectionInterpolators, BilinearCorrect) {
  InterpolatorBilinear interp;
  cv::Mat img = (cv::Mat_<float>(2, 2) << 1.0, 2.0, 3.0, 4.0);

  {  // outside of image -> invalid range
    const auto range = getRange(interp, img, -1.0, 2.0);
    EXPECT_FALSE(range);
  }

  {  // interp gives us average of 0, 0 and 1, 0
    const auto range = getRange(interp, img, 0.0, 0.5);
    ASSERT_TRUE(range);
    EXPECT_NEAR(range.value(), 2.0f, 1.0e-3f);
  }

  {  // interp gives us average of 0, 0 and 0, 1
    const auto range = getRange(interp, img, 0.5, 0.0);
    ASSERT_TRUE(range);
    EXPECT_EQ(range.value(), 1.5f);
  }
}

TEST(ProjectionInterpolators, AdaptiveCorrect) {
  // forces bilinear (max diff is 3)
  InterpolatorAdaptive interp_bilinear({3.5});
  // forces nearest
  InterpolatorAdaptive interp_nearest({0.2});
  cv::Mat img = (cv::Mat_<float>(2, 2) << 1.0, 2.0, 3.0, 4.0);

  {  // outside of image -> invalid range
    EXPECT_FALSE(getRange(interp_bilinear, img, -1.0, 2.0));
    EXPECT_FALSE(getRange(interp_nearest, img, -1.0, 2.0));
  }

  {  // interp gives us average of 0, 0 and 1, 0
    const auto range_bilinear = getRange(interp_bilinear, img, 0.0, 0.5);
    const auto range_nearest = getRange(interp_nearest, img, 0.0, 0.5);
    ASSERT_TRUE(range_bilinear);
    ASSERT_TRUE(range_nearest);
    EXPECT_NEAR(range_bilinear.value(), 2.0f, 1.0e-3f);
    EXPECT_NEAR(range_nearest.value(), 3.0f, 1.0e-3f);
  }

  {  // interp gives us average of 0, 0 and 1, 0
    const auto range_bilinear = getRange(interp_bilinear, img, 0.5, 0.0);
    const auto range_nearest = getRange(interp_nearest, img, 0.5, 0.0);
    ASSERT_TRUE(range_bilinear);
    ASSERT_TRUE(range_nearest);
    EXPECT_NEAR(range_bilinear.value(), 1.5f, 1.0e-3f);
    EXPECT_NEAR(range_nearest.value(), 2.0f, 1.0e-3f);
  }
}

TEST(ProjectionInterpolators, AdaptiveOutOfRange) {
  // force nearest interpolation outside of image
  InterpolatorAdaptive interp({0.5});
  cv::Mat img = (cv::Mat_<float>(2, 2) << 1.0, 2.0, 3.0, 4.0);
  const auto range = getRange(interp, img, 1.9, 0.9);
  ASSERT_FALSE(range);
}

}  // namespace hydra
