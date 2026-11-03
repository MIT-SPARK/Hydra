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
#include <unistd.h>

#include <filesystem>

#include "hydra_test/shared_dsg_fixture.h"

namespace hydra {
namespace {

std::filesystem::path makeTempDir() {
  auto pattern =
      (std::filesystem::temp_directory_path() / "hydra-keyframes-XXXXXX").string();
  if (!mkdtemp(pattern.data())) {
    throw std::runtime_error("could not create temporary directory");
  }

  return pattern;
}

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

TEST(KeyframeSelector, SelectFields) {
  const auto data = makeInput(makeCamera(), 5);
  const auto result = KeyframeSelector::selectFields(*data, {"depth", "labels"});
  ASSERT_TRUE(result);
  EXPECT_EQ(result->timestamp_ns, 5u);
  EXPECT_TRUE(result->color_image.empty());
  EXPECT_TRUE(result->vertex_map.empty());
  EXPECT_EQ(result->feature.size(), 0);
  ASSERT_FALSE(result->depth_image.empty());
  ASSERT_FALSE(result->label_image.empty());
  // shallow copies: the frontend never modifies its (const) inputs
  EXPECT_EQ(result->depth_image.data, data->depth_image.data);
  // original is untouched
  EXPECT_FALSE(data->color_image.empty());
  EXPECT_EQ(data->feature.size(), 3);
}

TEST(KeyframeSelector, SavesAcceptedKeyframes) {
  const auto log_path = makeTempDir() / "keyframes";
  const auto sensor = makeCamera();

  KeyframeSelector::Config config;
  DistancePolicy::Config policy;
  policy.min_pose_separation = 1.0;
  policy.min_time_separation_s = 0.0;
  config.keyframe_policy = config::VirtualConfig<KeyframePolicy>(policy);
  config.log_path = log_path;

  auto dsg = test::makeSharedDsg();
  FrontendOutput output(0, 0);
  {
    KeyframeSelector selector(config);
    ActiveWindowOutput msg(makeInput(sensor, 10, 0.0));
    msg.sensor_data.push_back(makeInput(sensor, 20, 0.5));  // dropped: not far enough
    msg.sensor_data.push_back(makeInput(sensor, 30, 2.0));
    selector.call(msg, *dsg, output, nullptr);

    ActiveWindowOutput msg2(makeInput(sensor, 40, 4.0));
    selector.call(msg2, *dsg, output, nullptr);
    selector.flush();

    EXPECT_TRUE(std::filesystem::exists(log_path / "10.zip"));
    EXPECT_FALSE(std::filesystem::exists(log_path / "20.zip"));
    EXPECT_TRUE(std::filesystem::exists(log_path / "30.zip"));
    EXPECT_TRUE(std::filesystem::exists(log_path / "40.zip"));
  }

  size_t num_files = 0;
  for (const auto& entry : std::filesystem::directory_iterator(log_path)) {
    EXPECT_EQ(entry.path().extension(), ".zip") << entry.path();
    ++num_files;
  }
  EXPECT_EQ(num_files, 3u);

  const auto loaded = InputData::load(log_path / "30.zip");
  ASSERT_TRUE(loaded);
  EXPECT_EQ(loaded->timestamp_ns, 30u);
  EXPECT_NEAR(loaded->world_T_body.translation().x(), 2.0, 1.0e-9);
  EXPECT_EQ(loaded->getSensor().name, "camera");
  ASSERT_FALSE(loaded->color_image.empty());
  ASSERT_FALSE(loaded->depth_image.empty());
  EXPECT_EQ(loaded->color_image.at<cv::Vec3b>(1, 1), cv::Vec3b(10, 20, 30));
  EXPECT_FLOAT_EQ(loaded->depth_image.at<float>(1, 1), 1.5f);
  EXPECT_TRUE(loaded->label_image.empty());
  EXPECT_TRUE(loaded->vertex_map.empty());
  EXPECT_EQ(loaded->feature.size(), 0);

  std::filesystem::remove_all(log_path.parent_path());
}

TEST(KeyframeSelector, SavesDescriptorsOnlyWhenRequested) {
  const auto root = makeTempDir();
  const auto sensor = makeCamera();
  DistancePolicy::Config policy;
  policy.min_pose_separation = 0.0;
  policy.min_time_separation_s = 0.0;

  auto dsg = test::makeSharedDsg();
  FrontendOutput output(0, 0);
  for (const bool with_features : {false, true}) {
    KeyframeSelector::Config config;
    config.keyframe_policy = config::VirtualConfig<KeyframePolicy>(policy);
    config.log_path = root / (with_features ? "on" : "off");
    if (with_features) {
      config.save_fields.push_back("features");
    }

    auto input = makeInput(sensor, 10);
    input->label_features.emplace(3, FeatureVector::Constant(2, 0.5));
    {
      KeyframeSelector selector(config);
      ActiveWindowOutput msg(input);
      selector.call(msg, *dsg, output, nullptr);
      selector.flush();
    }

    const auto loaded = InputData::load(config.log_path / "10.zip");
    ASSERT_TRUE(loaded);
    if (!with_features) {  // default: descriptors are dropped
      EXPECT_EQ(loaded->feature.size(), 0);
      EXPECT_TRUE(loaded->label_features.empty());
      continue;
    }

    ASSERT_EQ(loaded->feature.size(), 3);
    EXPECT_FLOAT_EQ(loaded->feature(2), 1.0f);
    ASSERT_EQ(loaded->label_features.size(), 1u);
    ASSERT_EQ(loaded->label_features.at(3).size(), 2);
    EXPECT_FLOAT_EQ(loaded->label_features.at(3)(1), 0.5f);
    // images are still selected as configured
    EXPECT_FALSE(loaded->color_image.empty());
    EXPECT_TRUE(loaded->label_image.empty());
  }

  std::filesystem::remove_all(root);
}

TEST(KeyframeSelector, SensorSuffixForSecondarySensors) {
  const auto log_path = makeTempDir() / "keyframes_multi";
  const auto front = makeCamera("front");
  const auto rear = makeCamera("rear cam/1");

  KeyframeSelector::Config config;
  DistancePolicy::Config policy;
  policy.min_pose_separation = 0.0;
  policy.min_time_separation_s = 0.0;
  config.keyframe_policy = config::VirtualConfig<KeyframePolicy>(policy);
  config.log_path = log_path;

  auto dsg = test::makeSharedDsg();
  FrontendOutput output(0, 0);
  {
    KeyframeSelector selector(config);
    ActiveWindowOutput msg(makeInput(front, 10));
    msg.sensor_data.push_back(makeInput(rear, 10));  // same stamp, other sensor
    msg.sensor_data.push_back(makeInput(front, 20));
    selector.call(msg, *dsg, output, nullptr);
    selector.flush();
  }

  EXPECT_TRUE(std::filesystem::exists(log_path / "10.zip"));
  EXPECT_TRUE(std::filesystem::exists(log_path / "10_rear_cam_1.zip"));
  EXPECT_TRUE(std::filesystem::exists(log_path / "20.zip"));
  const auto loaded = InputData::load(log_path / "10_rear_cam_1.zip");
  ASSERT_TRUE(loaded);
  EXPECT_EQ(loaded->getSensor().name, "rear cam/1");
  std::filesystem::remove_all(log_path.parent_path());
}

TEST(KeyframeSelector, NoLogPathNoFiles) {
  KeyframeSelector::Config config;
  EXPECT_TRUE(config.log_path.empty());
  KeyframeSelector selector(config);
  auto dsg = test::makeSharedDsg();
  FrontendOutput output(0, 0);
  ActiveWindowOutput msg(makeInput(makeCamera(), 10));
  selector.call(msg, *dsg, output, nullptr);
  selector.flush();  // no-op without a log path
}

TEST(KeyframeSelector, FlushOnDestruction) {
  const auto log_path = makeTempDir();
  KeyframeSelector::Config config;
  DistancePolicy::Config policy;
  policy.min_pose_separation = 0.0;
  policy.min_time_separation_s = 0.0;
  config.keyframe_policy = config::VirtualConfig<KeyframePolicy>(policy);
  config.log_path = log_path;
  config.max_queue_size = 0;

  const auto sensor = makeCamera();
  auto dsg = test::makeSharedDsg();
  FrontendOutput output(0, 0);
  {
    KeyframeSelector selector(config);
    ActiveWindowOutput msg(makeInput(sensor, 1));
    for (size_t i = 2; i <= 20; ++i) {
      msg.sensor_data.push_back(makeInput(sensor, i));
    }
    selector.call(msg, *dsg, output, nullptr);
  }

  size_t num_files = 0;
  for (const auto& entry : std::filesystem::directory_iterator(log_path)) {
    EXPECT_EQ(entry.path().extension(), ".zip") << entry.path();
    ++num_files;
  }
  EXPECT_EQ(num_files, 20u);
  std::filesystem::remove_all(log_path);
}

}  // namespace hydra
