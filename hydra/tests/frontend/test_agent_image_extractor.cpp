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
#include <config_utilities/virtual_config.h>
#include <gtest/gtest.h>
#include <hydra/active_window/active_window_output.h>
#include <hydra/common/global_info.h>
#include <hydra/frontend/agent_image_extractor.h>
#include <hydra/input/camera.h>
#include <hydra/input/input_data.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>

#include <chrono>
#include <filesystem>

#include "hydra_test/shared_dsg_fixture.h"
#include "hydra_test/temp_directory.h"

namespace hydra {
namespace {

namespace fs = std::filesystem;
using spark_dsg::AgentNodeAttributes;
using spark_dsg::DsgLayers;
using spark_dsg::NodeId;
using spark_dsg::NodeSymbol;
using spark_dsg::SceneGraph;

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

// Number of cv::Mat headers sharing this Mat's pixel buffer (1 == sole owner)
int refCount(const cv::Mat& mat) { return mat.u ? mat.u->refcount : 0; }

InputData::Ptr makeData(const Sensor::ConstPtr& sensor, uint64_t stamp_ns) {
  auto data = std::make_shared<InputData>(sensor);
  data->timestamp_ns = stamp_ns;
  data->world_T_body = Eigen::Isometry3d::Identity();
  data->color_image = cv::Mat(4, 8, CV_8UC3, cv::Scalar(1, 2, 3));
  data->depth_image = cv::Mat(4, 8, CV_32FC1, cv::Scalar(1.5f));
  return data;
}

AgentImageExtractor::Config makeConfig(const fs::path& output) {
  AgentImageExtractor::Config config;
  config.image_output_path = output;
  return config;
}

// Drives the extractor the way the graph builder does
struct ExtractorHarness {
  explicit ExtractorHarness(const AgentImageExtractor::Config& config)
      : extractor(config),
        dsg(test::makeSharedDsg()),
        sensor(std::make_shared<Camera>(makeCameraConfig(), "camera")) {}

  void addFrame(uint64_t stamp_ns, const Sensor::ConstPtr& other = nullptr) {
    const auto data = makeData(other ? other : sensor, stamp_ns);
    ActiveWindowOutput msg(data);
    FrontendOutput output(stamp_ns, 0);
    extractor.call(msg, *dsg, output, nullptr);
  }

  NodeId addAgent(size_t index, uint64_t stamp_ns, double x) {
    const auto& prefix = GlobalInfo::instance().getRobotPrefix();
    const NodeSymbol node_id(prefix.key, index);
    auto attrs =
        std::make_unique<AgentNodeAttributes>(std::chrono::nanoseconds(stamp_ns),
                                              Eigen::Quaterniond::Identity(),
                                              Eigen::Vector3d(x, 0.0, 0.0),
                                              NodeSymbol('p', index));
    auto& graph = *dsg->graph;
    graph.emplaceNode(graph.getLayerKey(DsgLayers::AGENTS)->layer,
                      node_id,
                      std::move(attrs),
                      prefix.key);
    new_nodes.push_back(node_id);
    return node_id;
  }

  //! Post-update with the agent nodes added since the last update
  void update() {
    FrontendOutput output(0, 0);
    output.new_agent_nodes = new_nodes;
    new_nodes.clear();
    extractor.callPostUpdate(*dsg, output);
  }

  const std::string& folder(NodeId node) const {
    return dsg->graph->getNode(node).attributes<AgentNodeAttributes>().image_folder;
  }

  AgentImageExtractor extractor;
  SharedDsgInfo::Ptr dsg;
  Sensor::ConstPtr sensor;
  std::vector<NodeId> new_nodes;
};

}  // namespace

TEST(AgentImageExtractor, ConfigRequiresOutputPath) {
  EXPECT_FALSE(config::isValid(AgentImageExtractor::Config()));
  EXPECT_TRUE(config::isValid(makeConfig("/tmp/run/agents")));
}

// The extractor is an optional graph builder functor
TEST(AgentImageExtractor, CreatedFromYaml) {
  test::TempDirectory tmp;
  const auto output = tmp.path / "agents";
  const auto node =
      YAML::Load("{type: AgentImageExtractor, image_output_path: " + output.string() +
                 ", sensor_name: left, gate: {min_translation_m: 0.5}}");
  const auto config =
      config::fromYaml<config::VirtualConfig<GraphBuilderFunctor>>(node);
  const auto functor = config.create();
  const auto extractor = dynamic_cast<const AgentImageExtractor*>(functor.get());
  ASSERT_TRUE(extractor);
  EXPECT_EQ(extractor->config.image_output_path, output);
  EXPECT_EQ(extractor->config.sensor_name, "left");
  EXPECT_DOUBLE_EQ(extractor->config.gate.min_translation_m, 0.5);
  EXPECT_DOUBLE_EQ(extractor->config.gate.min_rotation_deg, 30.0);
  EXPECT_TRUE(fs::exists(output));
}

// Buffered frames must not share pixel buffers with the input, which shares them with
// the active window
TEST(AgentImageExtractor, BufferSharesNothingWithInput) {
  test::TempDirectory tmp;
  AgentImageExtractor extractor(makeConfig(tmp.path));
  auto dsg = test::makeSharedDsg();

  auto sensor = std::make_shared<Camera>(makeCameraConfig(), "camera");
  const auto data = makeData(sensor, 1000);
  ASSERT_EQ(refCount(data->color_image), 1);
  ASSERT_EQ(refCount(data->depth_image), 1);

  ActiveWindowOutput msg(data);
  FrontendOutput output(1000, 0);
  extractor.call(msg, *dsg, output, nullptr);
  EXPECT_EQ(refCount(data->color_image), 1);
  EXPECT_EQ(refCount(data->depth_image), 1);
}

// Keyframes pair agent nodes with the frame they were created from and respect the
// translation threshold
TEST(AgentImageExtractor, WritesPairedKeyframes) {
  test::TempDirectory tmp;
  const auto output = tmp.path / "agents";
  auto config = makeConfig(output);
  config.gate.min_translation_m = 1.0;
  ExtractorHarness harness(config);
  for (const uint64_t stamp_ns : {1000u, 2000u, 3000u}) {
    harness.addFrame(stamp_ns);
  }

  const auto a0 = harness.addAgent(0, 1000, 0.0);
  const auto a1 = harness.addAgent(1, 2000, 0.5);  // too close to the previous keyframe
  const auto a2 = harness.addAgent(2, 3000, 1.5);
  harness.update();

  // image folders are relative to the parent of the output directory
  EXPECT_EQ(harness.folder(a0), "agents/agent_1000");
  EXPECT_TRUE(harness.folder(a1).empty());
  EXPECT_EQ(harness.folder(a2), "agents/agent_3000");
  EXPECT_TRUE(fs::exists(output / "camera_calib.json"));
  for (const std::string name : {"agent_1000", "agent_3000"}) {
    EXPECT_TRUE(fs::exists(output / (name + "_rgb.jpg")));
    EXPECT_TRUE(fs::exists(output / (name + "_depth.png")));
    EXPECT_TRUE(fs::exists(output / (name + "_meta.json")));
  }

  EXPECT_FALSE(fs::exists(output / "agent_2000_meta.json"));
}

// Frames from other sensors are ignored when a sensor name is configured
TEST(AgentImageExtractor, FiltersBySensorName) {
  test::TempDirectory tmp;
  auto config = makeConfig(tmp.path / "agents");
  config.sensor_name = "camera";
  ExtractorHarness harness(config);
  const auto other = std::make_shared<Camera>(makeCameraConfig(), "other");
  harness.addFrame(1000000000, other);
  harness.addFrame(2000000000);

  const auto a0 = harness.addAgent(0, 1000000000, 0.0);
  const auto a1 = harness.addAgent(1, 2000000000, 5.0);
  harness.update();
  EXPECT_TRUE(harness.folder(a0).empty());
  EXPECT_EQ(harness.folder(a1), "agents/agent_2000000000");
}

// A node waits a bounded number of updates for its frame
TEST(AgentImageExtractor, DefersMissingFramesBounded) {
  test::TempDirectory tmp;
  auto config = makeConfig(tmp.path / "agents");
  config.gate.min_translation_m = 0.0;
  config.max_deferred_updates = 2;
  ExtractorHarness harness(config);
  harness.addFrame(1000);

  const auto a0 = harness.addAgent(0, 1000, 0.0);
  const auto a1 = harness.addAgent(1, 5000000000, 1.0);  // frame not received yet
  harness.update();
  EXPECT_FALSE(harness.folder(a0).empty());
  EXPECT_TRUE(harness.folder(a1).empty());

  // frame arrives within the deferral budget
  harness.addFrame(5000000000);
  harness.update();
  EXPECT_FALSE(harness.folder(a1).empty());

  // a node whose frame never arrives is abandoned so later nodes can proceed
  const auto a2 = harness.addAgent(2, 7000000000, 2.0);
  for (size_t i = 0; i < 3; ++i) {
    harness.update();
  }

  const auto a3 = harness.addAgent(3, 9000000000, 3.0);
  harness.addFrame(9000000000);
  harness.update();
  EXPECT_TRUE(harness.folder(a2).empty());
  EXPECT_FALSE(harness.folder(a3).empty());
}

}  // namespace hydra
