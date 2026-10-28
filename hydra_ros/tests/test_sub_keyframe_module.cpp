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
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/validation.h>
#include <gtest/gtest.h>
#include <hydra/common/global_info.h>
#include <hydra/common/pipeline_queues.h>
#include <hydra/input/camera.h>
#include <hydra/input/sensor_input_packet.h>

#include <cstdlib>
#include <filesystem>

#include "hydra_ros/frontend/sub_keyframe_module.h"

namespace hydra {
namespace {

namespace fs = std::filesystem;

Camera::Config makeCameraConfig() {
  Camera::Config config;
  config.width = 8;
  config.height = 4;
  config.cx = 4.0;
  config.cy = 2.0;
  config.fx = 4.0;
  config.fy = 4.0;
  config.min_range = 0.1;
  config.max_range = 10.0;
  config.extrinsics = ParamSensorExtrinsics::Config();
  return config;
}

SubKeyframeInput makeInput(const Sensor::ConstPtr& sensor, uint64_t stamp_ns) {
  auto packet = std::make_shared<ImageInputPacket>(stamp_ns);
  packet->color = cv::Mat(4, 8, CV_8UC3, cv::Scalar(1, 2, 3));
  packet->depth = cv::Mat(4, 8, CV_32FC1, cv::Scalar(1.5f));
  return {sensor, stamp_ns, [packet] { return packet; }};
}

//! Configures the image store of the pipeline (empty disables it)
struct StoreGuard {
  explicit StoreGuard(const fs::path& path) {
    PipelineConfig config;
    config.image_store.path = path;
    GlobalInfo::init(config);
  }

  ~StoreGuard() { GlobalInfo::reset(); }
};

struct TempDirectory {
  TempDirectory() {
    auto pattern = (fs::temp_directory_path() / "hydra_ros_test-XXXXXX").string();
    if (!mkdtemp(pattern.data())) {
      throw std::runtime_error("failed to create temporary directory");
    }

    path = pattern;
  }

  ~TempDirectory() {
    std::error_code ec;
    fs::remove_all(path, ec);
  }

  fs::path path;
};

}  // namespace

TEST(SubKeyframeModule, ConfigParsesFromYaml) {
  const std::string yaml = R"yaml(
sensor_name: left_camera
gate: {min_translation_m: 0.25, min_rotation_deg: 15.0}
tf_lookup: {max_tries: 5}
queue_max_size: 30
)yaml";
  const auto config = config::fromYaml<SubKeyframeModule::Config>(YAML::Load(yaml));
  EXPECT_EQ(config.sensor_name, "left_camera");
  EXPECT_DOUBLE_EQ(config.gate.min_translation_m, 0.25);
  EXPECT_DOUBLE_EQ(config.gate.min_rotation_deg, 15.0);
  EXPECT_EQ(config.queue_max_size, 30u);
  EXPECT_TRUE(config::isValid(config));
  EXPECT_TRUE(config::isValid(SubKeyframeModule::Config()));
}

// Without an image store the module is disabled: the hand-off queue stays closed and
// nothing is written
TEST(SubKeyframeModule, NoStoreDisables) {
  const StoreGuard guard("");
  SubKeyframeModule::Config config;
  config.sensor_name = "camera";
  auto& queues = PipelineQueues::instance();
  {
    SubKeyframeModule module(config);
    EXPECT_FALSE(module.enabled());
    EXPECT_FALSE(queues.acceptsSubKeyframes("camera"));
    module.start();
    const auto sensor = std::make_shared<Camera>(makeCameraConfig(), "camera");
    EXPECT_FALSE(
        module.processInput(makeInput(sensor, 10), Eigen::Isometry3d::Identity()));
    module.stop();
  }

  EXPECT_FALSE(queues.acceptsSubKeyframes("camera"));
}

// The module enables the hand-off queue and writes gated images as unanchored frames
TEST(SubKeyframeModule, WritesUnanchoredFrames) {
  TempDirectory tmp;
  const auto output = tmp.path / "image_store";

  const StoreGuard guard(output);
  SubKeyframeModule::Config config;
  config.sensor_name = "camera";
  config.gate.min_translation_m = 0.5;
  auto& queues = PipelineQueues::instance();
  EXPECT_FALSE(queues.acceptsSubKeyframes("camera"));
  {
    SubKeyframeModule module(config);
    EXPECT_TRUE(queues.acceptsSubKeyframes("camera"));
    EXPECT_FALSE(queues.acceptsSubKeyframes("other"));

    const auto sensor = std::make_shared<Camera>(makeCameraConfig(), "camera");
    Eigen::Isometry3d world_T_body = Eigen::Isometry3d::Identity();
    EXPECT_TRUE(module.processInput(makeInput(sensor, 10), world_T_body));
    world_T_body.translation().x() = 0.1;
    EXPECT_FALSE(module.processInput(makeInput(sensor, 20), world_T_body));
    world_T_body.translation().x() = 1.0;
    EXPECT_TRUE(module.processInput(makeInput(sensor, 30), world_T_body));

    // the module shares the store instance of its directory
    const auto store = ImageStore::open(output);
    ASSERT_TRUE(store);
    store->flush();
    EXPECT_TRUE(fs::exists(output / ImageStore::kDatabaseName));
    EXPECT_TRUE(fs::exists(output / "frames/r0/10_rgb.jpg"));
    EXPECT_TRUE(fs::exists(output / "frames/r0/10_depth.png"));
    EXPECT_FALSE(fs::exists(output / "frames/r0/20_rgb.jpg"));
    EXPECT_TRUE(fs::exists(output / "frames/r0/30_rgb.jpg"));

    // frames wait for the frontend to anchor them
    const auto pending = store->unanchoredFrames(0, 100);
    ASSERT_EQ(pending.size(), 2u);
    EXPECT_EQ(pending[0].timestamp_ns, 10u);
    EXPECT_EQ(pending[1].timestamp_ns, 30u);
    EXPECT_NEAR(pending[1].world_T_body.translation().x(), 1.0, 1.0e-9);
  }

  EXPECT_FALSE(queues.acceptsSubKeyframes("camera"));
}

// Images rejected by the gate are never parsed
TEST(SubKeyframeModule, ParsesOnlyGatedImages) {
  TempDirectory tmp;
  const StoreGuard guard(tmp.path / "image_store");
  SubKeyframeModule::Config config;
  config.gate.min_translation_m = 0.5;
  SubKeyframeModule module(config);

  const auto sensor = std::make_shared<Camera>(makeCameraConfig(), "camera");
  size_t num_parsed = 0;
  auto input = makeInput(sensor, 10);
  const auto parse = input.parse;
  input.parse = [&num_parsed, parse] {
    ++num_parsed;
    return parse();
  };

  const Eigen::Isometry3d world_T_body = Eigen::Isometry3d::Identity();
  EXPECT_TRUE(module.processInput(input, world_T_body));
  EXPECT_FALSE(module.processInput(input, world_T_body));
  EXPECT_EQ(num_parsed, 1u);
}

}  // namespace hydra
