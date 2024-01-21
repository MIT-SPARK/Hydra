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
#include <hydra/loop_closure/detector.h>
#include <hydra/reconstruction/camera.h>

#include "hydra_test/config_guard.h"

namespace hydra {
namespace lcd {

struct FakeAgentFactory : public SensorDescriptorFactory {
  struct Config : public SensorDescriptorFactory::Config {};
  explicit FakeAgentFactory(const Config& config) : SensorDescriptorFactory(config) {}
  virtual ~FakeAgentFactory() = default;

 protected:
  Descriptor::Ptr describe(const Sensor&,
                           const DynamicSceneGraphNode&,
                           const FrameData&,
                           const SensorFeatures*) const override {
    return std::make_unique<Descriptor>();
  }

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<SensorDescriptorFactory, FakeAgentFactory, Config>(
          "FakeAgent");
};

void declare_config(FakeAgentFactory::Config&) {
  using namespace config;
  name("FakeAgentFactory");
}

struct LcdDetectorTests : public ::testing::Test {
  LcdDetectorTests() = default;

  virtual ~LcdDetectorTests() = default;

  virtual void SetUp() override {
    dsg.reset(new DynamicSceneGraph());

    Camera::Config cam_config;
    cam_config.min_range = 0.1;
    cam_config.max_range = 5.0;
    cam_config.width = 100;
    cam_config.height = 100;
    cam_config.fx = 1.0;
    cam_config.fy = 1.0;
    cam_config.cx = 0.5;
    cam_config.cy = 0.5;
    cam_config.extrinsics = config::VirtualConfig<SensorExtrinsics>(
        IdentitySensorExtrinsics::Config(), "identity");
    sensor = std::make_shared<Camera>(cam_config);

    config.agent_config.descriptors = config::VirtualConfig<SensorDescriptorFactory>(
        FakeAgentFactory::Config(), "FakeAgent");
    config.agent_config.matching = {0.8, 0.8, 0.0, 1, 0.0, 0.0};

    ObjectGraphDescriptorFactory::Config object_factory_config{{5.0}};
    PlaceGraphDescriptorFactory::Config place_factory_config{{5.0}, {0.5, 2.5, 30}};

    LayerLcdConfig object_config;
    object_config.layer = DsgLayers::OBJECTS;
    object_config.matching = {0.8, 0.8, 0.0, 1, 0.0, 0.0};
    object_config.descriptors = config::VirtualConfig<GraphDescriptorFactory>(
        object_factory_config, "ObjectGraphDescriptor");
    config.graph_configs.push_back(object_config);

    LayerLcdConfig place_config;
    place_config.layer = DsgLayers::PLACES;
    place_config.matching = {0.8, 0.8, 0.0, 1, 0.0, 0.0};
    place_config.descriptors = config::VirtualConfig<GraphDescriptorFactory>(
        place_factory_config, "PlaceGraphDescriptor");
    config.graph_configs.push_back(place_config);
  }

  LcdDetectorConfig config;
  DynamicSceneGraph::Ptr dsg;
  std::shared_ptr<Sensor> sensor;
};

TEST_F(LcdDetectorTests, TestEmptyUpdate) {
  LcdDetector module(config);
  std::unordered_set<NodeId> active_places;
  module.updateDescriptorCache(*dsg, active_places);
  EXPECT_EQ(0u, module.numGraphDescriptors(DsgLayers::PLACES));
  EXPECT_EQ(0u, module.numGraphDescriptors(DsgLayers::OBJECTS));
  EXPECT_EQ(0u, module.numAgentDescriptors());
}

TEST_F(LcdDetectorTests, TestInvalidNodeUpdate) {
  LcdDetector module(config);
  std::unordered_set<NodeId> active_places{1, 2, 3, 4, 5};
  module.updateDescriptorCache(*dsg, active_places);
  EXPECT_EQ(0u, module.numGraphDescriptors(DsgLayers::PLACES));
  EXPECT_EQ(0u, module.numGraphDescriptors(DsgLayers::OBJECTS));
  EXPECT_EQ(0u, module.numAgentDescriptors());
}

TEST_F(LcdDetectorTests, TestNoChildren) {
  dsg->emplaceNode(DsgLayers::PLACES, 1, std::make_unique<NodeAttributes>());
  dsg->emplaceNode(DsgLayers::PLACES, 2, std::make_unique<NodeAttributes>());
  dsg->emplaceNode(DsgLayers::PLACES, 3, std::make_unique<NodeAttributes>());
  dsg->emplaceNode(DsgLayers::PLACES, 4, std::make_unique<NodeAttributes>());
  dsg->emplaceNode(DsgLayers::PLACES, 5, std::make_unique<NodeAttributes>());

  LcdDetector module(config);
  std::unordered_set<NodeId> active_places{1, 2, 3, 4, 5};
  module.updateDescriptorCache(*dsg, active_places);
  EXPECT_EQ(0u, module.numGraphDescriptors(DsgLayers::PLACES));
  EXPECT_EQ(0u, module.numGraphDescriptors(DsgLayers::OBJECTS));
  EXPECT_EQ(0u, module.numAgentDescriptors());
}

TEST_F(LcdDetectorTests, TestNoDynamicChildren) {
  dsg->emplaceNode(DsgLayers::PLACES, 1, std::make_unique<NodeAttributes>());
  dsg->emplaceNode(DsgLayers::OBJECTS, 2, std::make_unique<NodeAttributes>());
  dsg->emplaceNode(DsgLayers::OBJECTS, 3, std::make_unique<NodeAttributes>());
  dsg->insertEdge(1, 2);
  dsg->insertEdge(1, 3);

  LcdDetector module(config);
  std::unordered_set<NodeId> active_places{1, 2, 3, 4, 5};
  module.updateDescriptorCache(*dsg, active_places);
  EXPECT_EQ(0u, module.numGraphDescriptors(DsgLayers::PLACES));
  EXPECT_EQ(0u, module.numGraphDescriptors(DsgLayers::OBJECTS));
  EXPECT_EQ(0u, module.numAgentDescriptors());
}

TEST_F(LcdDetectorTests, TestActualChildren) {
  using namespace std::chrono_literals;
  const auto guard = test::ConfigGuard::FixedLabels(5);
  dsg->emplaceNode(DsgLayers::PLACES, 1, std::make_unique<PlaceNodeAttributes>());
  dsg->emplaceNode(DsgLayers::OBJECTS, 2, std::make_unique<ObjectNodeAttributes>());
  dsg->emplaceNode(DsgLayers::AGENTS,
                   'a',
                   10ns,
                   std::make_unique<AgentNodeAttributes>(
                       Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0));
  dsg->emplaceNode(DsgLayers::AGENTS,
                   'a',
                   20ns,
                   std::make_unique<AgentNodeAttributes>(
                       Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0));
  dsg->insertEdge(1, 2);
  dsg->insertEdge(1, NodeSymbol('a', 0));
  dsg->insertEdge(1, NodeSymbol('a', 1));

  LcdDetector module(config);
  module.addSensorDescriptor(*sensor, *dsg, NodeSymbol('a', 0), {});
  module.addSensorDescriptor(*sensor, *dsg, NodeSymbol('a', 1), {});
  EXPECT_EQ(0u, module.numAgentDescriptors());

  std::unordered_set<NodeId> active_places{1};
  module.updateDescriptorCache(*dsg, active_places);
  EXPECT_EQ(1u, module.numGraphDescriptors(DsgLayers::PLACES));
  EXPECT_EQ(1u, module.numGraphDescriptors(DsgLayers::OBJECTS));
  EXPECT_EQ(2u, module.numAgentDescriptors());
}

TEST_F(LcdDetectorTests, TestEmptySearch) {
  using namespace std::chrono_literals;
  dsg->emplaceNode(DsgLayers::AGENTS,
                   'a',
                   10ns,
                   std::make_unique<AgentNodeAttributes>(
                       Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0));

  LcdDetector module(config);
  std::unordered_set<NodeId> active_places{1};

  const auto results = module.detect(*dsg, NodeSymbol('a', 0));
  EXPECT_EQ(0u, results.size());
}

TEST_F(LcdDetectorTests, TestNonEmptySearch) {
  using namespace std::chrono_literals;
  const auto guard = test::ConfigGuard::FixedLabels(5);
  dsg->emplaceNode(DsgLayers::PLACES, 1, std::make_unique<PlaceNodeAttributes>());
  dsg->emplaceNode(DsgLayers::OBJECTS, 2, std::make_unique<ObjectNodeAttributes>());
  dsg->emplaceNode(DsgLayers::AGENTS,
                   'a',
                   10ns,
                   std::make_unique<AgentNodeAttributes>(
                       Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0));
  dsg->emplaceNode(DsgLayers::AGENTS,
                   'a',
                   20ns,
                   std::make_unique<AgentNodeAttributes>(
                       Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0));
  dsg->insertEdge(1, 2);
  dsg->insertEdge(1, NodeSymbol('a', 0));
  dsg->insertEdge(1, NodeSymbol('a', 1));

  LcdDetector module(config);
  module.addSensorDescriptor(*sensor, *dsg, NodeSymbol('a', 0), {});
  module.addSensorDescriptor(*sensor, *dsg, NodeSymbol('a', 1), {});

  std::unordered_set<NodeId> active_places{1};
  module.updateDescriptorCache(*dsg, active_places);
  EXPECT_EQ(1u, module.numGraphDescriptors(DsgLayers::PLACES));
  EXPECT_EQ(1u, module.numGraphDescriptors(DsgLayers::OBJECTS));
  EXPECT_EQ(2u, module.numAgentDescriptors());

  const auto results = module.detect(*dsg, NodeSymbol('a', 0));
  EXPECT_EQ(0u, results.size());
}

}  // namespace lcd
}  // namespace hydra
