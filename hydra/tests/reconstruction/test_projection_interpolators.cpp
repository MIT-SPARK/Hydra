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

using spark_dsg::Color;

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

std::optional<Color> getColor(const ProjectionInterpolator& interp,
                              const cv::Mat& ranges,
                              const cv::Mat& colors,
                              float u,
                              float v) {
  const auto weights = interp.computeWeights(u, v, ranges);
  if (!weights.valid) {
    return std::nullopt;
  }

  return interp.interpolateColor(colors, weights);
}

std::optional<int> getId(const ProjectionInterpolator& interp,
                         const cv::Mat& ranges,
                         const cv::Mat& ids,
                         float u,
                         float v) {
  const auto weights = interp.computeWeights(u, v, ranges);
  if (!weights.valid) {
    return std::nullopt;
  }

  return interp.interpolateID(ids, weights);
}

std::optional<bool> getMask(const ProjectionInterpolator& interp,
                            const cv::Mat& ranges,
                            const cv::Mat& mask,
                            float u,
                            float v) {
  const auto weights = interp.computeWeights(u, v, ranges);
  if (!weights.valid) {
    return std::nullopt;
  }

  return interp.interpolateMask(mask, weights);
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

TEST(ProjectionInterpolators, NearestInvalidRangeCorrect) {
  InterpolatorNearest interp;
  cv::Mat img = (cv::Mat_<float>(2, 2) << 1.0, 2.0, 3.0, 4.0);

  // setup invalid range values
  img.at<float>(0, 1) = std::numeric_limits<float>::quiet_NaN();
  img.at<float>(1, 0) = 0.0f;
  img.at<float>(1, 1) = std::numeric_limits<float>::infinity();

  ASSERT_TRUE(getRange(interp, img, 0.2, 0.2));
  ASSERT_FALSE(getRange(interp, img, 1.2, 0.2));
  ASSERT_FALSE(getRange(interp, img, 0.2, 1.2));
  ASSERT_FALSE(getRange(interp, img, 1.2, 1.2));
}

TEST(ProjectionInterpolators, NearestCorrectNonRange) {
  InterpolatorNearest interp;
  cv::Mat ranges = (cv::Mat_<float>(2, 2) << 1.0, 2.0, 3.0, 4.0);
  cv::Mat colors = (cv::Mat_<cv::Vec3b>(2, 2) << cv::Vec3b(1, 2, 3),
                    cv::Vec3b(2, 3, 4),
                    cv::Vec3b(3, 4, 5),
                    cv::Vec3b(4, 5, 6));
  cv::Mat ids = (cv::Mat_<int>(2, 2) << 1, 2, 3, 1);
  cv::Mat mask = (cv::Mat_<bool>(2, 2) << false, true, false, true);

  {  // outside of image
    const auto color = getColor(interp, ranges, colors, 0.2, 0.4);
    EXPECT_EQ(color, Color(1, 2, 3));

    const auto id = getId(interp, ranges, ids, 0.2, 0.4);
    EXPECT_EQ(id, 1);

    const auto flag = getMask(interp, ranges, mask, 0.2, 0.4);
    EXPECT_EQ(flag, false);
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

TEST(ProjectionInterpolators, BilinearInvalidRangeCorrect) {
  InterpolatorBilinear interp;
  cv::Mat img = (cv::Mat_<float>(2, 2) << 1.0, 2.0, 3.0, 4.0);
  ASSERT_TRUE(getRange(interp, img, 0.0, 0.5));

  // NaNs should disable interpolation
  img.at<float>(0, 1) = std::numeric_limits<float>::quiet_NaN();
  ASSERT_FALSE(getRange(interp, img, 0.0, 0.5));

  // 0 depth should also be invalid
  img.at<float>(0, 1) = 2.0;
  img.at<float>(1, 0) = 0.0f;
  ASSERT_FALSE(getRange(interp, img, 0.0, 0.5));

  // infinite depth is also invalid
  img.at<float>(1, 0) = 3.0;
  img.at<float>(0, 0) = std::numeric_limits<float>::infinity();
  ASSERT_FALSE(getRange(interp, img, 0.0, 0.5));
}

TEST(ProjectionInterpolators, BilinearCorrectNonRange) {
  InterpolatorBilinear interp;
  cv::Mat ranges = (cv::Mat_<float>(2, 2) << 1.0, 2.0, 3.0, 4.0);
  cv::Mat colors = (cv::Mat_<cv::Vec3b>(2, 2) << cv::Vec3b(1, 2, 3),
                    cv::Vec3b(2, 3, 4),
                    cv::Vec3b(3, 4, 5),
                    cv::Vec3b(4, 5, 6));
  cv::Mat ids = (cv::Mat_<int>(2, 2) << 1, 1, 3, 1);
  cv::Mat mask = (cv::Mat_<bool>(2, 2) << false, true, false, true);

  {  // outside of image
    const auto color = getColor(interp, ranges, colors, 0.0, 0.5);
    EXPECT_EQ(color, Color(2, 3, 4));

    const auto id = getId(interp, ranges, ids, 0.5, 0.0);
    EXPECT_EQ(id, 1);

    const auto flag = getMask(interp, ranges, mask, 0.7, 0.5);
    EXPECT_EQ(flag, true);
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

TEST(ProjectionInterpolators, AdaptiveCorrectNonRange) {
  // check that bilinear values match
  cv::Mat ranges = (cv::Mat_<float>(2, 2) << 1.0, 2.0, 3.0, 4.0);
  cv::Mat colors = (cv::Mat_<cv::Vec3b>(2, 2) << cv::Vec3b(1, 2, 3),
                    cv::Vec3b(2, 3, 4),
                    cv::Vec3b(3, 4, 5),
                    cv::Vec3b(4, 5, 6));
  cv::Mat ids = (cv::Mat_<int>(2, 2) << 1, 1, 3, 1);
  cv::Mat mask = (cv::Mat_<bool>(2, 2) << false, true, false, true);

  {  // bilinear interpolation results
    InterpolatorAdaptive interp(InterpolatorAdaptive::Config{3.5});
    const auto color = getColor(interp, ranges, colors, 0.0, 0.5);
    EXPECT_EQ(color, Color(2, 3, 4));

    const auto id = getId(interp, ranges, ids, 0.5, 0.0);
    EXPECT_EQ(id, 1);

    const auto flag = getMask(interp, ranges, mask, 0.7, 0.5);
    EXPECT_EQ(flag, true);
  }

  {  // nearest interpolation results
    InterpolatorAdaptive interp(InterpolatorAdaptive::Config{0.2});

    const auto color = getColor(interp, ranges, colors, 0.0, 0.6);
    EXPECT_EQ(color, Color(3, 4, 5));

    const auto id = getId(interp, ranges, ids, 0.6, 0.0);
    EXPECT_EQ(id, 1);

    const auto flag = getMask(interp, ranges, mask, 0.7, 0.51);
    EXPECT_EQ(flag, true);
  }
}

TEST(ProjectionInterpolators, AdaptiveCorrectInvalidRange) {
  cv::Mat ranges = (cv::Mat_<float>(2, 2) << 1.0, 2.0, 3.0, 4.0);

  InterpolatorAdaptive interp(InterpolatorAdaptive::Config{3.5});
  ASSERT_TRUE(getRange(interp, ranges, 0.3, 0.4));

  ranges.at<float>(0, 0) = 0.0;
  ASSERT_FALSE(getRange(interp, ranges, 0.3, 0.4));
  ASSERT_TRUE(getRange(interp, ranges, 0.6, 0.4));

  ranges.at<float>(0, 0) = 1.0;
  ranges.at<float>(0, 1) = std::numeric_limits<float>::quiet_NaN();
  ASSERT_TRUE(getRange(interp, ranges, 0.3, 0.4));
  ASSERT_FALSE(getRange(interp, ranges, 0.6, 0.4));
}

TEST(ProjectionInterpolators, LabelVotingCorrect) {
  cv::Mat ranges = (cv::Mat_<float>(2, 2) << 1.0, 2.0, 3.0, 4.0);

  InterpolatorBilinear interp;
  cv::Mat ids = (cv::Mat_<int>(2, 2) << 2, 1, 3, 1);

  // voting should pick 2 (row 1 has no weight)
  EXPECT_EQ(getId(interp, ranges, ids, 0.4, 0.0), 2);

  // voting should pick 1 (equal weight between rows)
  EXPECT_EQ(getId(interp, ranges, ids, 0.4, 0.5), 1);

  // voting should pick 3 (row 0 has no weight)
  EXPECT_EQ(getId(interp, ranges, ids, 0.4, 0.99), 3);
}

TEST(ProjectionInterpolators, AdaptiveOutOfRange) {
  // force nearest interpolation outside of image
  InterpolatorAdaptive interp({0.5});
  cv::Mat img = (cv::Mat_<float>(2, 2) << 1.0, 2.0, 3.0, 4.0);
  const auto range = getRange(interp, img, 1.9, 0.9);
  ASSERT_FALSE(range);
}

}  // namespace hydra
