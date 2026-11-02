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
#include <hydra/frontend/keyframe_selector.h>
#include <hydra/input/camera.h>
#include <hydra/input/sensor_extrinsics.h>

#include "hydra_test/shared_dsg_fixture.h"

namespace hydra {
namespace {

Sensor::ConstPtr makeCamera(const std::string& name = "camera") {
  Camera::Config config;
  config.width = 4;
  config.height = 3;
  config.fx = 3;
  config.fy = 4;
  config.cx = 1;
  config.cy = 2;
  config.min_range = 0.1;
  config.max_range = 10.0;
  config.extrinsics = IdentitySensorExtrinsics::Config();
  return std::make_shared<Camera>(config, name);
}

InputData::Ptr makeInput(const Sensor::ConstPtr& sensor,
                         uint64_t timestamp_ns,
                         double x = 0.0) {
  auto data = std::make_shared<InputData>(sensor);
  data->timestamp_ns = timestamp_ns;
  data->world_T_body =
      Eigen::Translation3d(x, 0.0, 0.0) * Eigen::Isometry3d::Identity();
  data->color_image = cv::Mat(3, 4, CV_8UC3, cv::Scalar(10, 20, 30));
  data->depth_image = cv::Mat(3, 4, CV_32FC1, cv::Scalar(1.5));
  data->label_image = cv::Mat(3, 4, CV_32SC1, cv::Scalar(2));
  data->vertex_map = cv::Mat(3, 4, CV_32FC3, cv::Scalar(1.0, 2.0, 3.0));
  data->feature = FeatureVector::Constant(3, 1.0);
  return data;
}

}  // namespace

TEST(DistancePolicy, MaxTimeSeparation) {
  const auto sensor = makeCamera();
  const std::list<InputData::ConstPtr> keyframes{makeInput(sensor, 0)};

  DistancePolicy::Config config;
  config.min_pose_separation = 1.0;
  config.min_time_separation_s = 0.5;
  std::string reason;
  {  // default: no motion means no new keyframe
    DistancePolicy policy(config);
    EXPECT_FALSE(
        policy.shouldAdd(makeInput(sensor, 10'000'000'000), keyframes, reason));
    EXPECT_TRUE(
        policy.shouldAdd(makeInput(sensor, 1'000'000'000, 2.0), keyframes, reason));
  }

  config.max_time_separation_s = 2.0;
  DistancePolicy policy(config);
  // min time separation still applies
  EXPECT_FALSE(
      policy.shouldAdd(makeInput(sensor, 100'000'000, 2.0), keyframes, reason));
  // not moving and not enough time passed
  EXPECT_FALSE(policy.shouldAdd(makeInput(sensor, 1'000'000'000), keyframes, reason));
  // not moving but enough time passed
  EXPECT_TRUE(policy.shouldAdd(makeInput(sensor, 2'000'000'000), keyframes, reason));
  // moving
  EXPECT_TRUE(
      policy.shouldAdd(makeInput(sensor, 1'000'000'000, 2.0), keyframes, reason));
}

}  // namespace hydra
