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
#include <hydra/active_window/reconstruction_module.h>
#include <hydra/input/camera.h>

#include "hydra_test/config_guard.h"

namespace hydra {
namespace {

Sensor::ConstPtr makeCamera() {
  Camera::Config config;
  config.width = 32;
  config.height = 24;
  config.fx = 20;
  config.fy = 20;
  config.cx = 16;
  config.cy = 12;
  config.min_range = 0.1;
  config.max_range = 10.0;
  config.extrinsics = IdentitySensorExtrinsics::Config();
  return std::make_shared<Camera>(config, "camera");
}

InputData::Ptr makeInput(const Sensor::ConstPtr& sensor, uint64_t timestamp_ns) {
  auto data = std::make_shared<InputData>(sensor);
  data->timestamp_ns = timestamp_ns;
  data->world_T_body = Eigen::Isometry3d::Identity();
  data->color_image = cv::Mat(24, 32, CV_8UC3, cv::Scalar(10, 20, 30));
  data->depth_image = cv::Mat(24, 32, CV_32FC1, cv::Scalar(2.0));
  return data;
}

ReconstructionModule::Config makeConfig(bool forward) {
  ReconstructionModule::Config config;
  config.volumetric_map.with_semantics = false;
  config.validate_mesh_fields = false;
  config.mesh.integrator_threads = 1;
  ProjectiveIntegrator::Config tsdf;
  tsdf.num_threads = 1;
  config.tsdf.sensors.emplace_back("", tsdf);  // single mode: any sensor
  config.full_update_separation_s = 1.0;
  config.forward_intermediate_inputs = forward;
  return config;
}

std::vector<uint64_t> stamps(const ActiveWindowOutput& output) {
  std::vector<uint64_t> result;
  for (const auto& data : output.sensor_data) {
    result.push_back(data->timestamp_ns);
  }
  return result;
}

constexpr uint64_t kSecond = 1'000'000'000;

}  // namespace

TEST(ReconstructionModule, ForwardsIntermediateInputsInOrder) {
  const test::ConfigGuard guard;
  const auto sensor = makeCamera();
  auto queue = std::make_shared<ReconstructionModule::OutputQueue>();
  ReconstructionModule module(makeConfig(true), queue, nullptr);

  // first input is always a full update
  EXPECT_TRUE(module.step(makeInput(sensor, 0)));
  // partial updates produce no output and are held back
  EXPECT_FALSE(module.step(makeInput(sensor, kSecond / 4)));
  EXPECT_FALSE(module.step(makeInput(sensor, kSecond / 2)));
  // the next full update carries them first (chronological), then its own input
  EXPECT_TRUE(module.step(makeInput(sensor, kSecond)));
  // pending at stop: emitted as a last output
  EXPECT_FALSE(module.step(makeInput(sensor, 5 * kSecond / 4)));
  EXPECT_FALSE(module.step(makeInput(sensor, 3 * kSecond / 2)));
  module.stop();

  ASSERT_EQ(queue->size(), 3u);
  const auto first = queue->pop();
  EXPECT_EQ(stamps(*first), std::vector<uint64_t>({0}));
  EXPECT_EQ(first->timestamp_ns, 0u);

  const auto second = queue->pop();
  EXPECT_EQ(stamps(*second),
            std::vector<uint64_t>({kSecond / 4, kSecond / 2, kSecond}));
  EXPECT_EQ(second->timestamp_ns, kSecond);
  EXPECT_EQ(second->world_T_body().translation().norm(), 0.0);

  const auto last = queue->pop();
  EXPECT_EQ(stamps(*last), std::vector<uint64_t>({5 * kSecond / 4, 3 * kSecond / 2}));
  EXPECT_EQ(last->timestamp_ns, 3 * kSecond / 2);

  // stopping again is a no-op
  module.stop();
  EXPECT_EQ(queue->size(), 0u);
}

TEST(ReconstructionModule, NoForwardingByDefault) {
  const test::ConfigGuard guard;
  const auto sensor = makeCamera();
  auto queue = std::make_shared<ReconstructionModule::OutputQueue>();
  ReconstructionModule module(makeConfig(false), queue, nullptr);

  EXPECT_TRUE(module.step(makeInput(sensor, 0)));
  EXPECT_FALSE(module.step(makeInput(sensor, kSecond / 2)));
  EXPECT_TRUE(module.step(makeInput(sensor, kSecond)));
  EXPECT_FALSE(module.step(makeInput(sensor, 3 * kSecond / 2)));
  module.stop();

  ASSERT_EQ(queue->size(), 2u);
  EXPECT_EQ(stamps(*queue->pop()), std::vector<uint64_t>({0}));
  EXPECT_EQ(stamps(*queue->pop()), std::vector<uint64_t>({kSecond}));
}

}  // namespace hydra
