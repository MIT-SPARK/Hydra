/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * all rights reserved
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
#include <config_utilities/config_utilities.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <hydra/input/camera.h>

#include <optional>
#include <set>

namespace hydra {

std::shared_ptr<Camera> createCamera(double vfov,
                                     double hfov,
                                     std::pair<double, double> range,
                                     std::pair<int, int> width_height_pair = {640,
                                                                              480}) {
  Camera::Config config;
  config.min_range = range.first;
  config.max_range = range.second;
  config.width = width_height_pair.first;
  config.height = width_height_pair.second;
  config.cx = config.width / 2.0f;
  config.cy = config.height / 2.0f;
  config.fx = config.width / (2.0 * std::tan(hfov * M_PI / 360.0));
  config.fy = config.height / (2.0 * std::tan(vfov * M_PI / 360.0));
  config.extrinsics = ParamSensorExtrinsics::Config();
  return std::make_unique<Camera>(config);
}

TEST(Camera, ProjectionCorrect) {
  // 640 x 480
  const auto camera = createCamera(60.0, 90.0, {1.0, 5.0});
  float u = -1.0;
  float v = -1.0;
  // we can't project points behind the camera
  EXPECT_FALSE(
      camera->projectPointToImagePlane(Eigen::Vector3f(0.0f, 0.0f, -1.0f), u, v));
  // top left corner
  EXPECT_TRUE(camera->projectPointToImagePlane(
      Eigen::Vector3f(-1.0f, -1.0f / std::sqrt(3.0f), 1.0f), u, v));
  EXPECT_NEAR(u, 0.0f, 1.0e-9);
  EXPECT_NEAR(v, 0.0f, 1.0e-9);
  // bottom right corner
  EXPECT_TRUE(camera->projectPointToImagePlane(
      Eigen::Vector3f(1.0f, 1.0f / std::sqrt(3.0f), 1.0f), u, v));
  EXPECT_NEAR(u, 640.0f, 1.0e-9);
  EXPECT_NEAR(v, 480.0f, 1.0e-9);
  // exceeding bounds fails projection
  EXPECT_FALSE(camera->projectPointToImagePlane(
      Eigen::Vector3f(1.1f, 1.0f / std::sqrt(3.0f), 1.0f), u, v));
  EXPECT_FALSE(camera->projectPointToImagePlane(
      Eigen::Vector3f(1.0f, 1.1f / std::sqrt(3.0f), 1.0f), u, v));
}

TEST(Camera, RayDensityCorrect) {
  const auto camera = createCamera(90.0, 90.0, {1.0, 5.0});
  EXPECT_NEAR(camera->computeRayDensity(1.0, 1.0), 320.0 * 240.0, 0.1);
  EXPECT_NEAR(camera->computeRayDensity(2.0, 1.0), 4.0 * 320.0 * 240.0, 0.1);
  EXPECT_NEAR(camera->computeRayDensity(1.0, 0.5), 4.0 * 320.0 * 240.0, 0.1);
}

TEST(Camera, IntProjectionCorrect) {
  // 640 x 480
  const auto camera = createCamera(60.0, 90.0, {1.0, 5.0});
  int u = -1;
  int v = -1;
  // we can't project points behind the camera
  EXPECT_FALSE(
      camera->projectPointToImagePlane(Eigen::Vector3f(0.0f, 0.0f, -1.0f), u, v));
  // top left corner
  EXPECT_TRUE(camera->projectPointToImagePlane(
      Eigen::Vector3f(-1.0f, -1.0f / std::sqrt(3.0f), 1.0f), u, v));
  EXPECT_EQ(u, 0);
  EXPECT_EQ(v, 0);
  // bottom right corner
  EXPECT_TRUE(camera->projectPointToImagePlane(
      Eigen::Vector3f(0.999f, 0.999f / std::sqrt(3.0f), 1.0f), u, v));
  EXPECT_EQ(u, 639);
  EXPECT_EQ(v, 479);

  // TODO(nathan) get point outside integer bounds
}

TEST(Camera, HalfPlaneViewFrustumCorrect) {
  const auto camera = createCamera(20.0, 90.0, {1.0, 5.0});
  // center of focal plane: just range should matter
  EXPECT_TRUE(camera->pointIsInViewFrustum(Eigen::Vector3f(0.0, 0.0f, 1.5f)));
  EXPECT_TRUE(camera->pointIsInViewFrustum(Eigen::Vector3f(0.0f, 0.0f, 0.5f)));
  // negative or out of range should fail
  EXPECT_FALSE(camera->pointIsInViewFrustum(Eigen::Vector3f(0.0f, 0.0f, -0.5f)));
  EXPECT_FALSE(camera->pointIsInViewFrustum(Eigen::Vector3f(0.0f, 0.0f, 6.5f)));
  // left frustum side (one just inside, one just outside)
  EXPECT_TRUE(camera->pointIsInViewFrustum(Eigen::Vector3f(-0.9f, 0.0f, 1.0f)));
  EXPECT_FALSE(camera->pointIsInViewFrustum(Eigen::Vector3f(-1.1f, 0.0f, 1.0f)));
  // right frustum side (one just inside, one just outside)
  EXPECT_TRUE(camera->pointIsInViewFrustum(Eigen::Vector3f(0.9f, 0.0f, 1.0f)));
  EXPECT_FALSE(camera->pointIsInViewFrustum(Eigen::Vector3f(1.1f, 0.0f, 1.0f)));
  // frustum top (one just inside, one just outside)
  EXPECT_TRUE(camera->pointIsInViewFrustum(Eigen::Vector3f(0.0f, -0.15f, 1.0f)));
  EXPECT_FALSE(camera->pointIsInViewFrustum(Eigen::Vector3f(0.0f, -0.2f, 1.0f)));
  // frustum bottom (one just inside, one just outside)
  EXPECT_TRUE(camera->pointIsInViewFrustum(Eigen::Vector3f(0.0f, 0.15f, 1.0f)));
  EXPECT_FALSE(camera->pointIsInViewFrustum(Eigen::Vector3f(0.0f, 0.2f, 1.0f)));
}

TEST(Camera, VertexMapAndRangeCorrect) {
  // fx: 1.5, fy: 1
  const auto camera = createCamera(90.0, 90.0, {1.0, 5.0}, {3, 2});
  const auto& config = camera->getConfig();
  cv::Mat depth(config.height, config.width, CV_32FC1);
  for (int i = 0; i < depth.rows; ++i) {
    for (int j = 0; j < depth.cols; ++j) {
      depth.at<float>(i, j) = i * depth.cols + j + 1;
    }
  }

  // x: 0 -> -1.5 / 1.5 = -1.0, x: 1 -> -0.5 / 1.5 = -1/3, x: 2 -> 0.5 / 1.5 = 1/3
  // y: 0 -> -1 / 1.0 = -1.0, y: 1 -> 0 / 1.0 = 0
  cv::Mat expected_points(depth.size(), CV_32FC3);
  cv::Mat expected_range(depth.size(), CV_32FC1);
  expected_points.at<cv::Vec3f>(0, 0) = {-1.0f, -1.0f, 1.0f};
  expected_range.at<float>(0, 0) = std::sqrt(3);
  expected_points.at<cv::Vec3f>(0, 1) = {-2.0 / 3.0f, -2.0f, 2.0f};
  expected_range.at<float>(0, 1) = 2.9059f;
  expected_points.at<cv::Vec3f>(0, 2) = {1.0f, -3.0f, 3.0f};
  expected_range.at<float>(0, 2) = 4.3589f;
  expected_points.at<cv::Vec3f>(1, 0) = {-4.0f, 0.0f, 4.0f};
  expected_range.at<float>(1, 0) = 5.6569f;
  expected_points.at<cv::Vec3f>(1, 1) = {-5.0 / 3.0f, 0.0f, 5.0f};
  expected_range.at<float>(1, 1) = 5.2605f;
  expected_points.at<cv::Vec3f>(1, 2) = {2.0f, 0.0f, 6.0f};
  expected_range.at<float>(1, 2) = 6.3245f;

  const auto result_points = camera->computeVertexMap(depth);
  float min_range = 0.0f;
  float max_range = 0.0f;
  const auto result_range = camera->computeRangeImage(depth, &min_range, &max_range);

  EXPECT_NEAR(min_range, std::sqrt(3), 1.0e-2);
  EXPECT_NEAR(max_range, 6.3245f, 1.0e-2);
  ASSERT_EQ(result_points.rows, expected_points.rows);
  ASSERT_EQ(result_points.cols, expected_points.cols);
  ASSERT_EQ(result_range.rows, expected_range.rows);
  ASSERT_EQ(result_range.cols, expected_range.cols);

  for (int i = 0; i < result_points.rows; ++i) {
    for (int j = 0; j < result_points.cols; ++j) {
      SCOPED_TRACE("checking result @ [" + std::to_string(i) + ", " +
                   std::to_string(j) + "]");
      EXPECT_NEAR(result_range.at<float>(i, j), expected_range.at<float>(i, j), 1.0e-2);
      const auto expected_point = expected_points.at<cv::Vec3f>(i, j);
      const auto result_point = result_points.at<cv::Vec3f>(i, j);
      EXPECT_NEAR(expected_point[0], result_point[0], 1.0e-5f);
      EXPECT_NEAR(expected_point[1], result_point[1], 1.0e-5f);
      EXPECT_NEAR(expected_point[2], result_point[2], 1.0e-5f);
    }
  }
}

TEST(Camera, FinalizeRepresentationsCorrect) {
  const auto camera = createCamera(90.0, 90.0, {1.0, 5.0}, {2, 1});

  InputData msg(camera);
  // depth required for range and pointcloud computation
  EXPECT_FALSE(camera->finalizeRepresentations(msg));

  // fill valid depth image and check range is correct
  msg.depth_image = cv::Mat(1, 2, CV_32FC1);
  msg.depth_image.at<float>(0, 0) = 2.0;  // [-2, 2.0, 2]
  msg.depth_image.at<float>(0, 1) = 4.0;  // [0, 4, 4]
  EXPECT_TRUE(camera->finalizeRepresentations(msg));
  EXPECT_NEAR(msg.min_range, 2.0f * std::sqrt(3.0f), 1.0e-5f);
  EXPECT_NEAR(msg.max_range, 4.0f * std::sqrt(2.0f), 1.0e-5f);
  // TODO(nathan) test pointcloud is in world frame
}

}  // namespace hydra
