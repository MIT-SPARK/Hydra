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
#include <gtest/gtest.h>
#include <hydra/utils/logging.h>

#include "hydra/frontend/feature_selector.h"
#include "hydra/input/camera.h"
#include "hydra/input/input_data.h"
#include "hydra/input/lidar.h"

namespace hydra {
namespace {

Eigen::VectorXf getOneHot(size_t i, size_t dim) {
  Eigen::VectorXf p = Eigen::VectorXf::Zero(dim);
  p(i) = 1.0;
  return p;
}

std::shared_ptr<Camera> createCamera(double vfov,
                                     double hfov,
                                     std::pair<double, double> range,
                                     std::pair<int, int> dims = {640, 480}) {
  Camera::Config config;
  config.min_range = range.first;
  config.max_range = range.second;
  config.width = dims.first;
  config.height = dims.second;
  config.cx = config.width / 2.0f;
  config.cy = config.height / 2.0f;
  config.fx = config.width / (2.0 * std::tan(hfov * M_PI / 360.0));
  config.fy = config.height / (2.0 * std::tan(vfov * M_PI / 360.0));
  config.extrinsics = ParamSensorExtrinsics::Config();
  return std::make_unique<Camera>(config, "test_camera");
}

}  // namespace

TEST(ViewSelector, ProjectionCorrect) {
  const auto camera = createCamera(60.0, 90.0, {1.0, 5.0});
  cv::Mat range_image(480, 640, CV_32FC1);
  range_image = 1.0;

  InputData data(camera);
  data.range_image = range_image;
  data.feature = getOneHot(1, 10);
  data.world_T_body = Eigen::Isometry3d::Identity();

  {  // identity pose makes test points easy
    FeatureView view(data);
    EXPECT_FALSE(view.pointInView(Eigen::Vector3d(0.0, 0.0, -1.0), 0.5));
    EXPECT_TRUE(view.pointInView(Eigen::Vector3d(0.0, 0.0, 0.3), 0.5));
    EXPECT_TRUE(view.pointInView(Eigen::Vector3d(0.0, 0.0, 1.3), 0.5));
    EXPECT_FALSE(view.pointInView(Eigen::Vector3d(0.0, 0.0, 1.9), 0.5));
  }

  data.world_T_body = Eigen::Translation<double, 3>(Eigen::Vector3d(0.0, 0.0, 1.0));

  {  // non-identity pose
    FeatureView view(data);
    EXPECT_FALSE(view.pointInView(Eigen::Vector3d(0.0, 0.0, 0.1), 0.5));
    EXPECT_TRUE(view.pointInView(Eigen::Vector3d(0.0, 0.0, 1.3), 0.5));
    EXPECT_TRUE(view.pointInView(Eigen::Vector3d(0.0, 0.0, 2.3), 0.5));
    EXPECT_FALSE(view.pointInView(Eigen::Vector3d(0.0, 0.0, 2.9), 0.5));
  }
}

TEST(ViewSelector, UsesColumnAndRowBoundsForWideImages) {
  const auto camera = createCamera(60.0, 90.0, {0.1, 5.0});
  InputData data(camera);
  data.range_image = cv::Mat(480, 640, CV_32FC1, cv::Scalar(0.0));
  data.range_image.at<float>(240, 600) = 2.0f;
  data.world_T_body = Eigen::Isometry3d::Identity();
  const FeatureView view(data);
  // Projects to (u=600, v=240): a valid column beyond the image's row count.
  EXPECT_TRUE(view.pointInView(Eigen::Vector3d(0.875, 0.0, 1.0), 0.1));
  // Nearby pixels with no range observation must not inherit that visibility.
  EXPECT_FALSE(view.pointInView(Eigen::Vector3d(0.85, 0.0, 1.0), 0.1));
  EXPECT_FALSE(view.pointInView(Eigen::Vector3d(1.0, 0.0, 1.0), 0.1));
}

TEST(ViewSelector, UsesLidarRangeImageBinning) {
  Lidar::Config config;
  config.min_range = 1.0;
  config.max_range = 5.0;
  config.horizontal_fov = 180.0;
  config.horizontal_resolution = 180.0 / 640.0;
  config.vertical_fov = 90.0;
  config.vertical_resolution = 90.0 / 480.0;
  config.extrinsics = ParamSensorExtrinsics::Config();
  const auto lidar = std::make_shared<Lidar>(config, "test_lidar");

  // Projects to (320.75, 240), so the return is stored in column 321.
  const Eigen::Vector3f point(2.0f, -0.00736f, 0.0f);
  InputData data(lidar);
  data.world_T_body = Eigen::Isometry3d::Identity();
  data.vertex_map = cv::Mat(1, 1, InputData::VertexMatType);
  data.vertex_map.at<InputData::VertexType>(0, 0) = {point.x(), point.y(), point.z()};
  ASSERT_TRUE(lidar->finalizeRepresentations(data));
  ASSERT_GT(data.range_image.at<InputData::RangeType>(240, 321), 0.0f);
  EXPECT_TRUE(FeatureView(data).pointInView(point.cast<double>(), 0.1));
}

TEST(ViewSelector, InterpolatesRangeAcrossSmoothSurfaces) {
  const auto camera = createCamera(60.0, 90.0, {0.1, 5.0});
  InputData data(camera);
  data.world_T_body = Eigen::Isometry3d::Identity();
  data.range_image = cv::Mat(480, 640, InputData::RangeMatType, cv::Scalar(0.0f));
  data.range_image(cv::Rect(320, 240, 1, 2)).setTo(1.0f);
  data.range_image(cv::Rect(321, 240, 1, 2)).setTo(1.16f);

  // At (320.25, 240), the interpolated range is 1.04 m.
  const Eigen::Vector3d point(1.05 * 0.25 / 320.0, 0.0, 1.05);
  EXPECT_TRUE(FeatureView(data).pointInView(point, 0.02f));
}

TEST(VmfFeatureSelector, NormalizesViewsBeforeFittingDistribution) {
  const auto camera = createCamera(60.0, 90.0, {0.1, 5.0});
  InputData data(camera);
  data.range_image = cv::Mat(480, 640, CV_32FC1, cv::Scalar(2.0));
  data.world_T_body = Eigen::Isometry3d::Identity();
  data.feature = 3.0f * getOneHot(0, 2);
  FeatureSelector::FeatureList views;
  views.emplace_back(data);
  InputData second(camera);
  second.range_image = data.range_image;
  second.world_T_body = data.world_T_body;
  second.feature = 7.0f * getOneHot(1, 2);
  views.emplace_back(second);

  spark_dsg::SemanticNodeAttributes attrs;
  attrs.position = Eigen::Vector3d(0.0, 0.0, 1.0);
  VMFFeatureSelector::Config config;
  config.kappa_max = 500;
  const VMFFeatureSelector selector(config);
  ASSERT_TRUE(selector.select(views, attrs));
  ASSERT_EQ(attrs.semantic_feature.size(), 2);
  EXPECT_TRUE(
      attrs.semantic_feature.isApprox(Eigen::Vector2f::Constant(std::sqrt(0.5f))));
  ASSERT_EQ(attrs.feature_concentration.size(), 1);
  // Two orthogonal unit views: Rbar = sqrt(1/2), kappa = 3/sqrt(2).
  EXPECT_NEAR(attrs.feature_concentration(0, 0), 3.0f / std::sqrt(2.0f), 1.0e-5f);

  config.kappa_max = 1;
  const VMFFeatureSelector clipped(config);
  ASSERT_TRUE(clipped.select(views, attrs));
  EXPECT_FLOAT_EQ(attrs.feature_concentration(0, 0), 1.0f);
}

}  // namespace hydra
