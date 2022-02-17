#include "kimera_dsg_builder/dsg_lcd_module.h"

#include <gtest/gtest.h>

namespace kimera {
namespace lcd {

using incremental::SharedDsgInfo;

struct DsgLcdModuleTests : public ::testing::Test {
  DsgLcdModuleTests() = default;

  virtual ~DsgLcdModuleTests() = default;

  virtual void SetUp() override {
    dsg.reset(new DynamicSceneGraph());

    config.object_radius_m = 5.0;
    config.place_radius_m = 5.0;
    config.num_semantic_classes = 20;
    config.place_histogram_config = HistogramConfig<double>(0.5, 2.5, 30);
    config.agent_search_config = {0.8, 0.8, 0.0, 1, 0.0, 0.0};
    config.search_configs[KimeraDsgLayers::OBJECTS] = {0.8, 0.8, 0.0, 1, 0.0, 0.0};
    config.search_configs[KimeraDsgLayers::PLACES] = {0.8, 0.8, 0.0, 1, 0.0, 0.0};
  }

  DsgLcdConfig config;
  DynamicSceneGraph::Ptr dsg;
};

TEST_F(DsgLcdModuleTests, TestEmptyUpdate) {
  DsgLcdModule module(config);
  std::unordered_set<NodeId> active_places;
  module.updateDescriptorCache(*dsg, active_places);
  EXPECT_EQ(0u, module.numGraphDescriptors(KimeraDsgLayers::PLACES));
  EXPECT_EQ(0u, module.numGraphDescriptors(KimeraDsgLayers::OBJECTS));
  EXPECT_EQ(0u, module.numAgentDescriptors());
}

TEST_F(DsgLcdModuleTests, TestInvalidNodeUpdate) {
  DsgLcdModule module(config);
  std::unordered_set<NodeId> active_places{1, 2, 3, 4, 5};
  module.updateDescriptorCache(*dsg, active_places);
  EXPECT_EQ(0u, module.numGraphDescriptors(KimeraDsgLayers::PLACES));
  EXPECT_EQ(0u, module.numGraphDescriptors(KimeraDsgLayers::OBJECTS));
  EXPECT_EQ(0u, module.numAgentDescriptors());
}

TEST_F(DsgLcdModuleTests, TestNoChildren) {
  dsg->emplaceNode(KimeraDsgLayers::PLACES, 1, std::make_unique<NodeAttributes>());
  dsg->emplaceNode(KimeraDsgLayers::PLACES, 2, std::make_unique<NodeAttributes>());
  dsg->emplaceNode(KimeraDsgLayers::PLACES, 3, std::make_unique<NodeAttributes>());
  dsg->emplaceNode(KimeraDsgLayers::PLACES, 4, std::make_unique<NodeAttributes>());
  dsg->emplaceNode(KimeraDsgLayers::PLACES, 5, std::make_unique<NodeAttributes>());

  DsgLcdModule module(config);
  std::unordered_set<NodeId> active_places{1, 2, 3, 4, 5};
  module.updateDescriptorCache(*dsg, active_places);
  EXPECT_EQ(0u, module.numGraphDescriptors(KimeraDsgLayers::PLACES));
  EXPECT_EQ(0u, module.numGraphDescriptors(KimeraDsgLayers::OBJECTS));
  EXPECT_EQ(0u, module.numAgentDescriptors());
}

TEST_F(DsgLcdModuleTests, TestNoDynamicChildren) {
  dsg->emplaceNode(KimeraDsgLayers::PLACES, 1, std::make_unique<NodeAttributes>());
  dsg->emplaceNode(KimeraDsgLayers::OBJECTS, 2, std::make_unique<NodeAttributes>());
  dsg->emplaceNode(KimeraDsgLayers::OBJECTS, 3, std::make_unique<NodeAttributes>());
  dsg->insertEdge(1, 2);
  dsg->insertEdge(1, 3);

  DsgLcdModule module(config);
  std::unordered_set<NodeId> active_places{1, 2, 3, 4, 5};
  module.updateDescriptorCache(*dsg, active_places);
  EXPECT_EQ(0u, module.numGraphDescriptors(KimeraDsgLayers::PLACES));
  EXPECT_EQ(0u, module.numGraphDescriptors(KimeraDsgLayers::OBJECTS));
  EXPECT_EQ(0u, module.numAgentDescriptors());
}

TEST_F(DsgLcdModuleTests, TestActualChildren) {
  using namespace std::chrono_literals;
  dsg->emplaceNode(KimeraDsgLayers::PLACES, 1, std::make_unique<PlaceNodeAttributes>());
  dsg->emplaceNode(
      KimeraDsgLayers::OBJECTS, 2, std::make_unique<ObjectNodeAttributes>());
  dsg->emplaceDynamicNode(
      KimeraDsgLayers::AGENTS,
      'a',
      10ns,
      std::make_unique<AgentNodeAttributes>(
          Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0));
  dsg->emplaceDynamicNode(
      KimeraDsgLayers::AGENTS,
      'a',
      20ns,
      std::make_unique<AgentNodeAttributes>(
          Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0));
  dsg->insertEdge(1, 2);
  dsg->insertEdge(1, NodeSymbol('a', 0));
  dsg->insertEdge(1, NodeSymbol('a', 1));

  DsgLcdModule module(config);
  std::unordered_set<NodeId> active_places{1};
  module.updateDescriptorCache(*dsg, active_places);
  EXPECT_EQ(1u, module.numGraphDescriptors(KimeraDsgLayers::PLACES));
  EXPECT_EQ(1u, module.numGraphDescriptors(KimeraDsgLayers::OBJECTS));
  EXPECT_EQ(2u, module.numAgentDescriptors());
}

TEST_F(DsgLcdModuleTests, TestEmptySearch) {
  using namespace std::chrono_literals;
  dsg->emplaceDynamicNode(
      KimeraDsgLayers::AGENTS,
      'a',
      10ns,
      std::make_unique<AgentNodeAttributes>(
          Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0));

  DsgLcdModule module(config);
  std::unordered_set<NodeId> active_places{1};

  std::vector<DsgRegistrationSolution> results =
      module.detect(*dsg, NodeSymbol('a', 0));
  EXPECT_EQ(0u, results.size());
}

TEST_F(DsgLcdModuleTests, TestNonEmptySearch) {
  using namespace std::chrono_literals;
  dsg->emplaceNode(KimeraDsgLayers::PLACES, 1, std::make_unique<PlaceNodeAttributes>());
  dsg->emplaceNode(
      KimeraDsgLayers::OBJECTS, 2, std::make_unique<ObjectNodeAttributes>());
  dsg->emplaceDynamicNode(
      KimeraDsgLayers::AGENTS,
      'a',
      10ns,
      std::make_unique<AgentNodeAttributes>(
          Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0));
  dsg->emplaceDynamicNode(
      KimeraDsgLayers::AGENTS,
      'a',
      20ns,
      std::make_unique<AgentNodeAttributes>(
          Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0));
  dsg->insertEdge(1, 2);
  dsg->insertEdge(1, NodeSymbol('a', 0));
  dsg->insertEdge(1, NodeSymbol('a', 1));

  DsgLcdModule module(config);
  std::unordered_set<NodeId> active_places{1};
  module.updateDescriptorCache(*dsg, active_places);
  EXPECT_EQ(1u, module.numGraphDescriptors(KimeraDsgLayers::PLACES));
  EXPECT_EQ(1u, module.numGraphDescriptors(KimeraDsgLayers::OBJECTS));
  EXPECT_EQ(2u, module.numAgentDescriptors());

  std::vector<DsgRegistrationSolution> results =
      module.detect(*dsg, NodeSymbol('a', 0));
  EXPECT_EQ(0u, results.size());
}

}  // namespace lcd
}  // namespace kimera
