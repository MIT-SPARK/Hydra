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

namespace hydra::lcd {

struct LcdDetectorTests : public ::testing::Test {
  LcdDetectorTests() = default;

  virtual ~LcdDetectorTests() = default;

  virtual void SetUp() override {
    dsg.reset(new DynamicSceneGraph());

    config.object_extraction = SubgraphConfig(5.0);
    config.places_extraction = SubgraphConfig(5.0);
    config.num_semantic_classes = 20;
    config.place_histogram_config = HistogramConfig<double>(0.5, 2.5, 30);
    config.agent_search_config = {0.8, 0.8, 0.0, 1, 0.0, 0.0};
    config.objects.matching = {0.8, 0.8, 0.0, 1, 0.0, 0.0};
    config.places.matching = {0.8, 0.8, 0.0, 1, 0.0, 0.0};
  }

  LcdDetectorConfig config;
  DynamicSceneGraph::Ptr dsg;
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
  std::unordered_set<NodeId> active_places{1};
  module.updateDescriptorCache(*dsg, active_places);
  EXPECT_EQ(1u, module.numGraphDescriptors(DsgLayers::PLACES));
  EXPECT_EQ(1u, module.numGraphDescriptors(DsgLayers::OBJECTS));
  EXPECT_EQ(2u, module.numAgentDescriptors());

  const auto results = module.detect(*dsg, NodeSymbol('a', 0));
  EXPECT_EQ(0u, results.size());
}

}  // namespace hydra::lcd
