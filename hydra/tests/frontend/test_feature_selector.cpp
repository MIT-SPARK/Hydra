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
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "hydra/frontend/feature_selector.h"
#include "hydra/input/camera.h"
#include "hydra/input/input_data.h"

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

}  // namespace hydra
