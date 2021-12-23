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

    object_factory.reset(new ObjectDescriptorFactory(radius, num_classes));

    HistogramConfig<double> hist_config(hist_min, hist_max, hist_bins);
    place_factory.reset(new PlaceDescriptorFactory(radius, hist_config));

    LayerRegistrationConfig reg_config;
    TeaserParams teaser_params;
    object_registration.reset(new ObjectRegistrationFunctor(reg_config, teaser_params));
    places_registration.reset(new PlaceRegistrationFunctor(reg_config, teaser_params));

    config.agent_search_config = {KimeraDsgLayers::AGENTS, 0.8, 0.8, 0.0, 1, 0.0, 0.0};
    config.search_configs.push_back(
        DescriptorMatchConfig{KimeraDsgLayers::OBJECTS, 0.8, 0.8, 0.0, 1, 0.0, 0.0});
    config.search_configs.push_back(
        DescriptorMatchConfig{KimeraDsgLayers::PLACES, 0.8, 0.8, 0.0, 1, 0.0, 0.0});

    descriptor_factories[KimeraDsgLayers::OBJECTS] =
        [&](const DynamicSceneGraph& graph, const DynamicSceneGraphNode& node) {
          return (*object_factory)(graph, node);
        };
    descriptor_factories[KimeraDsgLayers::PLACES] =
        [&](const DynamicSceneGraph& graph, const DynamicSceneGraphNode& node) {
          return (*place_factory)(graph, node);
        };

    registration_funcs[KimeraDsgLayers::OBJECTS] =
        [&](const DynamicSceneGraph& dsg,
            const DsgRegistrationInput& match,
            NodeId agent_node) {
          return (*object_registration)(dsg, match, agent_node);
        };
    registration_funcs[KimeraDsgLayers::PLACES] = [&](const DynamicSceneGraph& dsg,
                                                      const DsgRegistrationInput& match,
                                                      NodeId agent_node) {
      return (*places_registration)(dsg, match, agent_node);
    };
  }

  const double radius = 5.0;
  const int num_classes = 20;
  const double hist_min = 0.5;
  const double hist_max = 2.5;
  const size_t hist_bins = 30;

  DsgLcdConfig config;

  std::unique_ptr<ObjectDescriptorFactory> object_factory;
  std::unique_ptr<PlaceDescriptorFactory> place_factory;
  std::unique_ptr<ObjectRegistrationFunctor> object_registration;
  std::unique_ptr<PlaceRegistrationFunctor> places_registration;

  std::map<LayerId, DescriptorFactoryFunc> descriptor_factories;
  std::map<LayerId, RegistrationFunc> registration_funcs;
  std::map<LayerId, ValidationFunc> validation_funcs;

  DynamicSceneGraph::Ptr dsg;
};

TEST_F(DsgLcdModuleTests, TestEmptyUpdate) {
  DsgLcdModule module(
      config, descriptor_factories, registration_funcs, validation_funcs);
  std::unordered_set<NodeId> active_places;
  module.updateDescriptorCache(*dsg, active_places);
  EXPECT_EQ(0u, module.numGraphDescriptors(KimeraDsgLayers::PLACES));
  EXPECT_EQ(0u, module.numGraphDescriptors(KimeraDsgLayers::OBJECTS));
  EXPECT_EQ(0u, module.numAgentDescriptors());
}

TEST_F(DsgLcdModuleTests, TestInvalidNodeUpdate) {
  DsgLcdModule module(
      config, descriptor_factories, registration_funcs, validation_funcs);
  std::unordered_set<NodeId> active_places{1, 2, 3, 4, 5};
  module.updateDescriptorCache(*dsg, active_places);
  EXPECT_EQ(0u, module.numGraphDescriptors(KimeraDsgLayers::PLACES));
  EXPECT_EQ(0u, module.numGraphDescriptors(KimeraDsgLayers::OBJECTS));
  EXPECT_EQ(0u, module.numAgentDescriptors());
}

TEST_F(DsgLcdModuleTests, TestNoChildren) {
  dsg->emplaceNode(
      KimeraDsgLayers::PLACES, 1, std::make_unique<NodeAttributes>());
  dsg->emplaceNode(
      KimeraDsgLayers::PLACES, 2, std::make_unique<NodeAttributes>());
  dsg->emplaceNode(
      KimeraDsgLayers::PLACES, 3, std::make_unique<NodeAttributes>());
  dsg->emplaceNode(
      KimeraDsgLayers::PLACES, 4, std::make_unique<NodeAttributes>());
  dsg->emplaceNode(
      KimeraDsgLayers::PLACES, 5, std::make_unique<NodeAttributes>());

  DsgLcdModule module(
      config, descriptor_factories, registration_funcs, validation_funcs);
  std::unordered_set<NodeId> active_places{1, 2, 3, 4, 5};
  module.updateDescriptorCache(*dsg, active_places);
  EXPECT_EQ(0u, module.numGraphDescriptors(KimeraDsgLayers::PLACES));
  EXPECT_EQ(0u, module.numGraphDescriptors(KimeraDsgLayers::OBJECTS));
  EXPECT_EQ(0u, module.numAgentDescriptors());
}

TEST_F(DsgLcdModuleTests, TestNoDynamicChildren) {
  dsg->emplaceNode(
      KimeraDsgLayers::PLACES, 1, std::make_unique<NodeAttributes>());
  dsg->emplaceNode(
      KimeraDsgLayers::OBJECTS, 2, std::make_unique<NodeAttributes>());
  dsg->emplaceNode(
      KimeraDsgLayers::OBJECTS, 3, std::make_unique<NodeAttributes>());
  dsg->insertEdge(1, 2);
  dsg->insertEdge(1, 3);

  DsgLcdModule module(
      config, descriptor_factories, registration_funcs, validation_funcs);
  std::unordered_set<NodeId> active_places{1, 2, 3, 4, 5};
  module.updateDescriptorCache(*dsg, active_places);
  EXPECT_EQ(0u, module.numGraphDescriptors(KimeraDsgLayers::PLACES));
  EXPECT_EQ(0u, module.numGraphDescriptors(KimeraDsgLayers::OBJECTS));
  EXPECT_EQ(0u, module.numAgentDescriptors());
}

TEST_F(DsgLcdModuleTests, TestActualChildren) {
  using namespace std::chrono_literals;
  dsg->emplaceNode(
      KimeraDsgLayers::PLACES, 1, std::make_unique<PlaceNodeAttributes>());
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

  DsgLcdModule module(
      config, descriptor_factories, registration_funcs, validation_funcs);
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

  DsgLcdModule module(
      config, descriptor_factories, registration_funcs, validation_funcs);
  std::unordered_set<NodeId> active_places{1};

  std::vector<DsgRegistrationSolution> results =
      module.detect(*dsg, NodeSymbol('a', 0));
  EXPECT_EQ(0u, results.size());
}

TEST_F(DsgLcdModuleTests, TestNonEmptySearch) {
  using namespace std::chrono_literals;
  dsg->emplaceNode(
      KimeraDsgLayers::PLACES, 1, std::make_unique<PlaceNodeAttributes>());
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

  DsgLcdModule module(
      config, descriptor_factories, registration_funcs, validation_funcs);
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
