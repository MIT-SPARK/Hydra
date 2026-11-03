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

std::vector<uint64_t> stamps(const std::vector<InputData::ConstPtr>& inputs) {
  std::vector<uint64_t> result;
  for (const auto& data : inputs) {
    result.push_back(data->timestamp_ns);
  }
  return result;
}

constexpr uint64_t kSecond = 1'000'000'000;

}  // namespace

// Steps the same inputs through a module and returns its outputs
std::vector<ActiveWindowOutput::Ptr> run(bool forward) {
  const auto sensor = makeCamera();
  auto queue = std::make_shared<ReconstructionModule::OutputQueue>();
  ReconstructionModule module(makeConfig(forward), queue, nullptr);

  // first input is always a full update
  EXPECT_TRUE(module.step(makeInput(sensor, 0)));
  // partial updates produce no output
  EXPECT_FALSE(module.step(makeInput(sensor, kSecond / 4)));
  EXPECT_FALSE(module.step(makeInput(sensor, kSecond / 2)));
  EXPECT_TRUE(module.step(makeInput(sensor, kSecond)));
  // still pending at stop: dropped
  EXPECT_FALSE(module.step(makeInput(sensor, 5 * kSecond / 4)));
  module.stop();

  std::vector<ActiveWindowOutput::Ptr> outputs;
  while (!queue->empty()) {
    outputs.push_back(queue->pop());
  }
  return outputs;
}

TEST(ReconstructionModule, ForwardsIntermediateInputsSeparately) {
  const test::ConfigGuard guard;
  const auto outputs = run(true);
  ASSERT_EQ(outputs.size(), 2u);
  EXPECT_EQ(stamps(outputs[0]->sensor_data), std::vector<uint64_t>({0}));
  EXPECT_TRUE(outputs[0]->intermediate_inputs.empty());

  // the partial updates come with the next full update, oldest first, outside of
  // sensor_data
  EXPECT_EQ(stamps(outputs[1]->sensor_data), std::vector<uint64_t>({kSecond}));
  EXPECT_EQ(stamps(outputs[1]->intermediate_inputs),
            std::vector<uint64_t>({kSecond / 4, kSecond / 2}));
  EXPECT_EQ(outputs[1]->timestamp_ns, kSecond);
}

TEST(ReconstructionModule, ForwardingKeepsSensorData) {
  const test::ConfigGuard guard;
  // everything but intermediate_inputs is the same as without forwarding
  const auto with = run(true);
  const auto without = run(false);
  ASSERT_EQ(with.size(), without.size());
  for (size_t i = 0; i < with.size(); ++i) {
    EXPECT_EQ(with[i]->timestamp_ns, without[i]->timestamp_ns);
    EXPECT_EQ(stamps(with[i]->sensor_data), stamps(without[i]->sensor_data));
    EXPECT_TRUE(without[i]->intermediate_inputs.empty());
  }
}

TEST(ActiveWindowOutput, CollatesIntermediateInputs) {
  const auto sensor = makeCamera();
  ActiveWindowOutput first(makeInput(sensor, 20));
  first.intermediate_inputs.push_back(makeInput(sensor, 10));
  ActiveWindowOutput second(makeInput(sensor, 40));
  second.intermediate_inputs.push_back(makeInput(sensor, 30));
  second.setMap(VolumetricMap(VolumetricMap::Config{}));
  first.updateFrom(std::move(second), true);
  EXPECT_EQ(stamps(first.sensor_data), std::vector<uint64_t>({20, 40}));
  EXPECT_EQ(stamps(first.intermediate_inputs), std::vector<uint64_t>({10, 30}));
}

}  // namespace hydra
