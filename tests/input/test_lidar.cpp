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
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <hydra/input/lidar.h>

#include <optional>
#include <set>

namespace hydra {

std::shared_ptr<Lidar> createLidar(double vfov,
                                   double hfov,
                                   std::pair<double, double> range,
                                   std::optional<double> vfov_top = std::nullopt) {
  Lidar::Config config;
  config.min_range = range.first;
  config.max_range = range.second;
  config.horizontal_fov = hfov;
  config.horizontal_resolution = hfov / 640.0;
  config.vertical_fov = vfov;
  config.vertical_resolution = vfov / 480.0;
  if (vfov_top) {
    config.is_asymmetric = true;
    config.vertical_fov_top = *vfov_top;
  } else {
    config.is_asymmetric = false;
  }
  config.extrinsics = ParamSensorExtrinsics::Config();
  return std::make_unique<Lidar>(config);
}

TEST(Lidar, RayDensityCorrect) {
  const auto lidar = createLidar(90.0, 180.0, {1.0, 5.0});
  // virtual fx is 160, virtual fy is 240
  EXPECT_NEAR(lidar->computeRayDensity(1.0, 1.0), 160.0 * 240.0, 0.1);
  EXPECT_NEAR(lidar->computeRayDensity(2.0, 1.0), 160.0 * 240.0 * 4.0, 0.1);
  EXPECT_NEAR(lidar->computeRayDensity(1.0, 0.5), 160.0 * 240.0 * 4.0, 0.1);
}

TEST(Lidar, FinalizeRepresentationsCorrect) {
  const auto lidar = createLidar(90.0, 180.0, {1.0, 5.0});
  InputData msg(lidar);
  // no points: no ability to make intermediate images
  EXPECT_FALSE(lidar->finalizeRepresentations(msg));

  msg.vertex_map = cv::Mat(1, 2, CV_32FC3);
  auto& v1 = msg.vertex_map.at<cv::Vec3f>(0, 0);
  v1[0] = 3.0;
  v1[1] = 4.0;
  v1[2] = 0.0;
  auto& v2 = msg.vertex_map.at<cv::Vec3f>(0, 1);
  v2[0] = 1.0;
  v2[1] = 0.0;
  v2[2] = 1.0;

  // no labels: no ability to make intermediate images
  EXPECT_FALSE(lidar->finalizeRepresentations(msg));

  // invalid label size: no ability to make intermediate images
  msg.label_image = cv::Mat(2, 1, CV_32SC1);
  EXPECT_FALSE(lidar->finalizeRepresentations(msg));

  msg.label_image = cv::Mat(1, 2, CV_32SC1);
  msg.label_image.at<int32_t>(0, 0) = 1;
  msg.label_image.at<int32_t>(0, 1) = 2;

  EXPECT_TRUE(lidar->finalizeRepresentations(msg));
  ASSERT_EQ(msg.range_image.rows, 480);
  ASSERT_EQ(msg.range_image.cols, 640);
  ASSERT_EQ(msg.label_image.rows, 480);
  ASSERT_EQ(msg.label_image.cols, 640);
  EXPECT_TRUE(msg.color_image.empty());
  EXPECT_NEAR(msg.min_range, std::sqrt(2.0), 1.0e-6);
  EXPECT_NEAR(msg.max_range, 5.0, 1.0e-6);

  std::vector<std::pair<int, int>> nonzero_points{{320, 0}, {131, 240}};
  for (int r = 0; r < msg.range_image.rows; ++r) {
    for (int c = 0; c < msg.range_image.cols; ++c) {
      bool has_data = false;
      for (const auto& point : nonzero_points) {
        if (point.first == c && point.second == r) {
          has_data = true;
          break;
        }
      }

      if (has_data) {
        continue;
      }

      SCOPED_TRACE("checking [" + std::to_string(r) + ", " + std::to_string(c) + "]");
      EXPECT_NEAR(msg.range_image.at<float>(r, c), 0.0, 1.0e-3);
      EXPECT_EQ(msg.label_image.at<int32_t>(r, c), -1);
    }
  }

  EXPECT_NEAR(msg.range_image.at<float>(0, 320), std::sqrt(2.0f), 1.0e-5f);
  EXPECT_EQ(msg.label_image.at<int32_t>(0, 320), 2);
  EXPECT_NEAR(msg.range_image.at<float>(240, 131), 5.0f, 1.0e-5f);
  EXPECT_EQ(msg.label_image.at<int32_t>(240, 131), 1);
}

TEST(Lidar, FinalizeRepresentationColor) {
  const auto lidar = createLidar(90.0, 180.0, {1.0, 5.0});
  InputData msg(lidar);
  msg.vertex_map = cv::Mat(1, 2, CV_32FC3);
  auto& v1 = msg.vertex_map.at<cv::Vec3f>(0, 0);
  v1[0] = 3.0;
  v1[1] = 4.0;
  v1[2] = 0.0;
  auto& v2 = msg.vertex_map.at<cv::Vec3f>(0, 1);
  v2[0] = 1.0;
  v2[1] = 0.0;
  v2[2] = 1.0;

  msg.label_image = cv::Mat(1, 2, CV_32SC1);
  msg.label_image.at<int32_t>(0, 0) = 1;
  msg.label_image.at<int32_t>(0, 1) = 2;

  msg.color_image = cv::Mat(1, 2, CV_8UC3);
  msg.color_image.at<cv::Vec3b>(0, 0) = {1, 2, 3};
  msg.color_image.at<cv::Vec3b>(0, 1) = {3, 4, 5};

  EXPECT_TRUE(lidar->finalizeRepresentations(msg));
  ASSERT_EQ(msg.range_image.rows, 480);
  ASSERT_EQ(msg.range_image.cols, 640);
  ASSERT_EQ(msg.label_image.rows, 480);
  ASSERT_EQ(msg.label_image.cols, 640);
  ASSERT_EQ(msg.color_image.rows, 480);
  ASSERT_EQ(msg.color_image.cols, 640);
  EXPECT_NEAR(msg.min_range, std::sqrt(2.0), 1.0e-6);
  EXPECT_NEAR(msg.max_range, 5.0, 1.0e-6);

  const cv::Vec3b color1{3, 4, 5};
  EXPECT_NEAR(msg.range_image.at<float>(0, 320), std::sqrt(2.0f), 1.0e-5f);
  EXPECT_EQ(msg.label_image.at<int32_t>(0, 320), 2);
  EXPECT_EQ(msg.color_image.at<cv::Vec3b>(0, 320), color1);

  const cv::Vec3b color2{1, 2, 3};
  EXPECT_NEAR(msg.range_image.at<float>(240, 131), 5.0f, 1.0e-5f);
  EXPECT_EQ(msg.label_image.at<int32_t>(240, 131), 1);
  EXPECT_EQ(msg.color_image.at<cv::Vec3b>(240, 131), color2);
}

TEST(Lidar, ImagePlaneProjectionCorrect) {
  // 640 x 480 image
  const auto lidar = createLidar(90.0, 270.0, {1.0, 5.0});
  float u = -1.0;
  float v = -1.0;

  // points too close to the lidar shouldn't be projected
  EXPECT_FALSE(lidar->projectPointToImagePlane(Eigen::Vector3f::Zero(), u, v));

  // points along x-axis get projected to image center
  EXPECT_TRUE(lidar->projectPointToImagePlane(Eigen::Vector3f(1.5f, 0.0f, 0.0f), u, v));
  EXPECT_NEAR(u, 320.0f, 1.0e-6);
  EXPECT_NEAR(v, 240.0f, 1.0e-6);

  // top center of image plane
  EXPECT_TRUE(
      lidar->projectPointToImagePlane(Eigen::Vector3f(1.5f, 0.0f, 1.4999999f), u, v));
  EXPECT_NEAR(u, 320.0f, 1.0e-3);
  EXPECT_NEAR(v, 0.0f, 1.0e-3);

  // bottom center of image plane
  EXPECT_TRUE(
      lidar->projectPointToImagePlane(Eigen::Vector3f(1.5f, 0.0f, -1.4999999f), u, v));
  EXPECT_NEAR(u, 320.0f, 1.0e-3);
  EXPECT_NEAR(v, 480.0f, 1.0e-3);

  // left center of image plane
  EXPECT_TRUE(
      lidar->projectPointToImagePlane(Eigen::Vector3f(-1.5f, 1.5000001f, 0.0f), u, v));
  EXPECT_NEAR(u, 0.0f, 1.0e-3);
  EXPECT_NEAR(v, 240.0f, 1.0e-3);

  // right center of image plane
  EXPECT_TRUE(
      lidar->projectPointToImagePlane(Eigen::Vector3f(-1.5f, -1.5000001f, 0.0f), u, v));
  EXPECT_NEAR(u, 640.0f, 1.0e-3);
  EXPECT_NEAR(v, 240.0f, 1.0e-3);

  // out-of-bounds horizontal
  EXPECT_FALSE(
      lidar->projectPointToImagePlane(Eigen::Vector3f(-1.5f, -1.0f, 0.0f), u, v));
  EXPECT_FALSE(
      lidar->projectPointToImagePlane(Eigen::Vector3f(-1.5f, 1.0f, 0.0f), u, v));
  // out-of-bounds vertical
  EXPECT_FALSE(
      lidar->projectPointToImagePlane(Eigen::Vector3f(-1.5f, 0.0f, 2.0f), u, v));
  EXPECT_FALSE(
      lidar->projectPointToImagePlane(Eigen::Vector3f(-1.5f, 0.0f, -2.0f), u, v));
}

TEST(Lidar, AsymmetricImagePlaneProjectionCorrect) {
  // 640 x 480 image
  const auto lidar = createLidar(90.0, 270.0, {1.0, 5.0}, 60);
  float u = -1.0;
  float v = -1.0;
  // top of image plane
  EXPECT_TRUE(lidar->projectPointToImagePlane(
      Eigen::Vector3f(1.5f, 0.0f, 1.4999999f * std::tan(M_PI / 3)), u, v));
  EXPECT_NEAR(u, 320.0f, 1.0e-3);
  EXPECT_NEAR(v, 0.0f, 1.0e-3);

  // bottom of image plane
  EXPECT_TRUE(lidar->projectPointToImagePlane(
      Eigen::Vector3f(1.5f, 0.0f, -1.4999999f * std::tan(M_PI / 6)), u, v));
  EXPECT_NEAR(u, 320.0f, 1.0e-3);
  EXPECT_NEAR(v, 480.0f, 1.0e-3);
  // out-of-bounds vertical
  EXPECT_FALSE(
      lidar->projectPointToImagePlane(Eigen::Vector3f(-1.5f, 0.0f, 2.6f), u, v));
  EXPECT_FALSE(
      lidar->projectPointToImagePlane(Eigen::Vector3f(-1.5f, 0.0f, -0.9f), u, v));
}

TEST(Lidar, IntImagePlaneProjectionCorrect) {
  // 640 x 480 image
  const auto lidar = createLidar(90.0, 270.0, {1.0, 5.0});
  int u = -1.0;
  int v = -1.0;

  // points too close to the lidar shouldn't be projected
  EXPECT_FALSE(lidar->projectPointToImagePlane(Eigen::Vector3f::Zero(), u, v));

  // points along x-axis get projected to image center
  EXPECT_TRUE(lidar->projectPointToImagePlane(Eigen::Vector3f(1.5f, 0.0f, 0.0f), u, v));
  EXPECT_EQ(u, 320);
  EXPECT_EQ(v, 240);

  // top left corner of image plane
  EXPECT_TRUE(lidar->projectPointToImagePlane(
      Eigen::Vector3f(-1.5f, 1.500001f, 1.499999f * std::sqrt(2.0f)), u, v));
  EXPECT_EQ(u, 0);
  EXPECT_EQ(v, 0);

  // bottom right of image plane
  // note: there's no easy way to get the v coordinate such that floor(v_float) == 480
  EXPECT_FALSE(lidar->projectPointToImagePlane(
      Eigen::Vector3f(-1.5f, -1.5000001f, -1.499999f * std::sqrt(2.0f)), u, v));
  EXPECT_EQ(u, 640);
  EXPECT_EQ(v, 479);

  // make sure rounding to pixel coordinates is correct
  float u_float = -1.0f;
  float v_float = -1.0f;
  Eigen::Vector3f test_point(-1.5f, -1.51f, -1.5001f * std::sqrt(2.0f));
  EXPECT_TRUE(lidar->projectPointToImagePlane(test_point, u_float, v_float));
  EXPECT_TRUE(lidar->projectPointToImagePlane(test_point, u, v));
  EXPECT_EQ(u, 639);
  EXPECT_EQ(v, 479);
  EXPECT_GT(u_float, 639.5f);
  EXPECT_LT(u_float, 640.0f);
  EXPECT_GT(v_float, 479.5f);
  EXPECT_LT(v_float, 480.0f);
}

TEST(Lidar, HalfPlaneViewFrustumCorrect) {
  const auto lidar = createLidar(20.0, 90.0, {1.0, 5.0});
  // center of focal plane: just range should matter
  EXPECT_TRUE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.5f, 0.0f, 0.0f)));
  // we don't handle min range because it would break >180 degree FOV
  EXPECT_TRUE(lidar->pointIsInViewFrustum(Eigen::Vector3f(0.5f, 0.0f, 0.0f)));
  EXPECT_FALSE(lidar->pointIsInViewFrustum(Eigen::Vector3f(6.5f, 0.0f, 0.0f)));
  // left frustum side (one just inside, one just outside)
  EXPECT_TRUE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.0f, 0.9f, 0.0f)));
  EXPECT_FALSE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.0f, 1.1f, 0.0f)));
  // right frustum side (one just inside, one just outside)
  EXPECT_TRUE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.0f, -0.9f, 0.0f)));
  EXPECT_FALSE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.0f, -1.1f, 0.0f)));
  // frustum top (one just inside, one just outside)
  EXPECT_TRUE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.0f, 0.0f, 0.15f)));
  EXPECT_FALSE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.0f, 0.0f, 0.2f)));
  // frustum bottom (one just inside, one just outside)
  EXPECT_TRUE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.0f, 0.0f, -0.15f)));
  EXPECT_FALSE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.0f, 0.0f, -0.2f)));
}

TEST(Lidar, FullPlaneViewFrustumCorrect) {
  const auto lidar = createLidar(20.0, 270.0, {1.0, 5.0});
  // center of focal plane: just range should matter
  EXPECT_TRUE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.5f, 0.0f, 0.0f)));
  // we don't handle min range because it would break >180 degree FOV
  EXPECT_TRUE(lidar->pointIsInViewFrustum(Eigen::Vector3f(0.5f, 0.0f, 0.0f)));
  EXPECT_FALSE(lidar->pointIsInViewFrustum(Eigen::Vector3f(6.5f, 0.0f, 0.0f)));
  // left frustum side (one just inside, one just outside)
  EXPECT_TRUE(lidar->pointIsInViewFrustum(Eigen::Vector3f(-1.0f, 1.1f, 0.0f)));
  EXPECT_FALSE(lidar->pointIsInViewFrustum(Eigen::Vector3f(-1.0f, 0.9f, 0.0f)));
  // right frustum side (one just inside, one just outside)
  EXPECT_TRUE(lidar->pointIsInViewFrustum(Eigen::Vector3f(-1.0f, -1.1f, 0.0f)));
  EXPECT_FALSE(lidar->pointIsInViewFrustum(Eigen::Vector3f(-1.0f, -0.9f, 0.0f)));
  // frustum top (one just inside, one just outside)
  EXPECT_TRUE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.0f, 0.0f, 0.15f)));
  EXPECT_FALSE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.0f, 0.0f, 0.2f)));
  // frustum bottom (one just inside, one just outside)
  EXPECT_TRUE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.0f, 0.0f, -0.15f)));
  EXPECT_FALSE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.0f, 0.0f, -0.2f)));
}

TEST(Lidar, AsymmetricViewFrustumCorrect) {
  const auto lidar = createLidar(90.0, 90.0, {1.0, 5.0}, 60);
  // center of focal plane: just range should matter
  EXPECT_TRUE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.5f, 0.0f, 0.0f)));
  // we don't handle min range because it would break >180 degree FOV
  EXPECT_TRUE(lidar->pointIsInViewFrustum(Eigen::Vector3f(0.5f, 0.0f, 0.0f)));
  EXPECT_FALSE(lidar->pointIsInViewFrustum(Eigen::Vector3f(6.5f, 0.0f, 0.0f)));
  // left frustum side (one just inside, one just outside)
  EXPECT_TRUE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.0f, 0.9f, 0.0f)));
  EXPECT_FALSE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.0f, 1.1f, 0.0f)));
  // right frustum side (one just inside, one just outside)
  EXPECT_TRUE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.0f, -0.9f, 0.0f)));
  EXPECT_FALSE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.0f, -1.1f, 0.0f)));
  // frustum top (one just inside, one just outside)
  EXPECT_TRUE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.0f, 0.0f, 1.7f)));
  EXPECT_FALSE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.0f, 0.0f, 1.75f)));
  // frustum bottom (one just inside, one just outside)
  EXPECT_TRUE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.0f, 0.0f, -0.55f)));
  EXPECT_FALSE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.0f, 0.0f, -0.6f)));
}

TEST(Lidar, ViewFrustumCorrectWithInflation) {
  const auto lidar = createLidar(20.0, 90.0, {1.0, 5.0});
  // center of focal plane: just range should matter
  EXPECT_TRUE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.5f, 0.0f, 0.0f)));
  // left frustum side (outside)
  EXPECT_FALSE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.0f, 1.1f, 0.0f)));
  EXPECT_TRUE(lidar->pointIsInViewFrustum(Eigen::Vector3f(1.0f, 1.1f, 0.0f), 0.2));
}

}  // namespace hydra
