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
#include <hydra/loop_closure/scene_graph_descriptors.h>

namespace hydra::lcd {

namespace {

inline const SceneGraphNode& makeDefaultAgentNode(DynamicSceneGraph& graph) {
  using namespace std::chrono_literals;
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  graph.emplaceNode(2, 'a', 10ns, std::make_unique<AgentNodeAttributes>(q, t, 0));
  return graph.getNode(NodeSymbol('a', 0));
}

inline void emplacePlaceNode(DynamicSceneGraph& graph,
                             const Eigen::Vector3d& pos,
                             double distance,
                             size_t& next_index) {
  auto attrs = std::make_unique<PlaceNodeAttributes>(distance, 0);
  attrs->position = pos;
  graph.emplaceNode(DsgLayers::PLACES, NodeSymbol('p', next_index), std::move(attrs));
  ++next_index;
}

}  // namespace

TEST(LoopClosureModuleDescriptorTests, TestAgentDescriptor) {
  DynamicSceneGraph graph;
  const auto& node = makeDefaultAgentNode(graph);

  auto& attrs = node.attributes<AgentNodeAttributes>();
  attrs.dbow_ids.resize(5, 1);
  attrs.dbow_ids << 1, 2, 3, 4, 5;
  attrs.dbow_values.resize(3, 1);
  attrs.dbow_values << 6.0, 7.0, 8.0;

  AgentDescriptorFactory factory;

  {  // no parent -> no descriptor
    auto descriptor = factory.construct(graph, node);
    EXPECT_TRUE(descriptor == nullptr);
  }

  size_t next_place_index = 0;
  emplacePlaceNode(graph, Eigen::Vector3d::Zero(), 1.0, next_place_index);
  graph.insertEdge(NodeSymbol('a', 0), NodeSymbol('p', 0));

  {  // valid parent -> valid descriptor
    auto descriptor = factory.construct(graph, node);
    ASSERT_TRUE(descriptor != nullptr);
    EXPECT_EQ(attrs.dbow_ids, descriptor->words);
    EXPECT_EQ(attrs.dbow_values, descriptor->values);
    EXPECT_TRUE(descriptor->normalized);

    EXPECT_EQ(descriptor->root_node, NodeSymbol('p', 0));

    std::set<NodeId> expected_nodes{NodeSymbol('a', 0)};
    EXPECT_EQ(expected_nodes, descriptor->nodes);
  }
}

TEST(LoopClosureModuleDescriptorTests, TestGetBinCorrect) {
  HistogramConfig<double> config(0.0, 1.0, 5);
  EXPECT_EQ(0u, config.getBin(-1.0));
  EXPECT_EQ(4u, config.getBin(2.0));
  EXPECT_EQ(0u, config.getBin(0.11));
  EXPECT_EQ(0u, config.getBin(0.0));
  EXPECT_EQ(1u, config.getBin(0.201));
  EXPECT_EQ(2u, config.getBin(0.401));
  EXPECT_EQ(3u, config.getBin(0.601));
  EXPECT_EQ(4u, config.getBin(0.801));
  EXPECT_EQ(4u, config.getBin(1.0));
}

TEST(LoopClosureModuleDescriptorTests, TestPlaceDescriptor) {
  DynamicSceneGraph graph;
  const auto& node = makeDefaultAgentNode(graph);

  size_t next_node_index = 0;
  // core measurements
  emplacePlaceNode(graph, Eigen::Vector3d(1.0, 2.0, 3.0), 0.1, next_node_index);
  emplacePlaceNode(graph, Eigen::Vector3d(1.0, 2.0, 3.0), 0.2, next_node_index);
  emplacePlaceNode(graph, Eigen::Vector3d(1.0, 2.0, 3.0), 0.3, next_node_index);
  emplacePlaceNode(graph, Eigen::Vector3d(1.0, 2.0, 3.0), 0.4, next_node_index);
  graph.insertEdge(NodeSymbol('p', 0), NodeSymbol('p', 1));
  graph.insertEdge(NodeSymbol('p', 1), NodeSymbol('p', 2));
  graph.insertEdge(NodeSymbol('p', 2), NodeSymbol('p', 3));

  // measurements outside radius
  emplacePlaceNode(graph, Eigen::Vector3d(10.0, 20.0, 30.0), 0.1, next_node_index);
  emplacePlaceNode(graph, Eigen::Vector3d(10.0, 20.0, 30.0), 0.2, next_node_index);
  emplacePlaceNode(graph, Eigen::Vector3d(10.0, 20.0, 30.0), 0.3, next_node_index);
  graph.insertEdge(NodeSymbol('p', 0), NodeSymbol('p', 4));
  graph.insertEdge(NodeSymbol('p', 1), NodeSymbol('p', 5));
  graph.insertEdge(NodeSymbol('p', 2), NodeSymbol('p', 6));

  // measurements inside radius, but unconnected
  emplacePlaceNode(graph, Eigen::Vector3d(1.0, 2.0, 3.0), 0.3, next_node_index);

  {  // no parent: empty descriptor
    PlaceDescriptorFactory factory(1.0, HistogramConfig<double>(0, 1.0, 1));
    Descriptor::Ptr descriptor = factory.construct(graph, node);
    EXPECT_TRUE(descriptor == nullptr);
  }

  graph.insertEdge(node.id, NodeSymbol('p', 0));

  {  // 1 bin: correct count
    PlaceDescriptorFactory factory(1.0, HistogramConfig<double>(0, 1.0, 1));
    Descriptor::Ptr descriptor = factory.construct(graph, node);
    ASSERT_TRUE(descriptor != nullptr);
    ASSERT_EQ(1, descriptor->values.rows());
    EXPECT_NEAR(4.0f, descriptor->values(0), 1.0e-5);
    EXPECT_EQ(descriptor->root_node, NodeSymbol('p', 0));
    std::set<NodeId> expected_nodes{
        NodeSymbol('p', 0), NodeSymbol('p', 1), NodeSymbol('p', 2), NodeSymbol('p', 3)};
    EXPECT_EQ(expected_nodes, descriptor->nodes);
  }

  {  // 1 bin + bigger radius: correct count
    PlaceDescriptorFactory factory(1000.0, HistogramConfig<double>(0, 1.0, 1));
    Descriptor::Ptr descriptor = factory.construct(graph, node);
    ASSERT_TRUE(descriptor != nullptr);
    ASSERT_EQ(1, descriptor->values.rows());
    EXPECT_NEAR(7.0f, descriptor->values(0), 1.0e-5);
    EXPECT_EQ(descriptor->root_node, NodeSymbol('p', 0));
    std::set<NodeId> expected_nodes{NodeSymbol('p', 0),
                                    NodeSymbol('p', 1),
                                    NodeSymbol('p', 2),
                                    NodeSymbol('p', 3),
                                    NodeSymbol('p', 4),
                                    NodeSymbol('p', 5),
                                    NodeSymbol('p', 6)};
    EXPECT_EQ(expected_nodes, descriptor->nodes);
  }

  {  // 4 bins: one in each
    PlaceDescriptorFactory factory(1.0, HistogramConfig<double>(0.05, 0.45, 4));
    Descriptor::Ptr descriptor = factory.construct(graph, node);
    ASSERT_TRUE(descriptor != nullptr);
    ASSERT_EQ(4, descriptor->values.rows());
    EXPECT_NEAR(1.0f, descriptor->values(0), 1.0e-5);
    EXPECT_NEAR(1.0f, descriptor->values(1), 1.0e-5);
    EXPECT_NEAR(1.0f, descriptor->values(2), 1.0e-5);
    EXPECT_NEAR(1.0f, descriptor->values(3), 1.0e-5);
    EXPECT_EQ(descriptor->root_node, NodeSymbol('p', 0));
    std::set<NodeId> expected_nodes{
        NodeSymbol('p', 0), NodeSymbol('p', 1), NodeSymbol('p', 2), NodeSymbol('p', 3)};
    EXPECT_EQ(expected_nodes, descriptor->nodes);
  }
}

void emplaceObjectNode(DynamicSceneGraph& graph,
                       const Eigen::Vector3d& pos,
                       uint8_t label,
                       size_t& next_index) {
  auto attrs = std::make_unique<SemanticNodeAttributes>();
  attrs->semantic_label = label;
  attrs->position = pos;
  graph.emplaceNode(DsgLayers::OBJECTS, NodeSymbol('o', next_index), std::move(attrs));
  ++next_index;
}

TEST(LoopClosureModuleDescriptorTests, TestObjectDescriptor) {
  DynamicSceneGraph graph;
  const auto& node = makeDefaultAgentNode(graph);

  size_t next_place_index = 0;
  // core measurements
  emplacePlaceNode(graph, Eigen::Vector3d(1.0, 2.0, 3.0), 0.1, next_place_index);
  emplacePlaceNode(graph, Eigen::Vector3d(1.0, 2.0, 3.0), 0.2, next_place_index);
  emplacePlaceNode(graph, Eigen::Vector3d(1.0, 2.0, 3.0), 0.3, next_place_index);
  emplacePlaceNode(graph, Eigen::Vector3d(1.0, 2.0, 3.0), 0.4, next_place_index);
  graph.insertEdge(NodeSymbol('p', 0), NodeSymbol('p', 1));
  graph.insertEdge(NodeSymbol('p', 1), NodeSymbol('p', 2));
  graph.insertEdge(NodeSymbol('p', 2), NodeSymbol('p', 3));

  // measurements outside radius
  emplacePlaceNode(graph, Eigen::Vector3d(10.0, 20.0, 30.0), 0.1, next_place_index);
  emplacePlaceNode(graph, Eigen::Vector3d(10.0, 20.0, 30.0), 0.2, next_place_index);
  emplacePlaceNode(graph, Eigen::Vector3d(10.0, 20.0, 30.0), 0.3, next_place_index);
  graph.insertEdge(NodeSymbol('p', 0), NodeSymbol('p', 4));
  graph.insertEdge(NodeSymbol('p', 1), NodeSymbol('p', 5));
  graph.insertEdge(NodeSymbol('p', 2), NodeSymbol('p', 6));

  {  // no parent: empty descriptor
    ObjectDescriptorFactory factory(1.0, 5);
    Descriptor::Ptr descriptor = factory.construct(graph, node);
    ASSERT_TRUE(descriptor == nullptr);
  }

  graph.insertEdge(node.id, NodeSymbol('p', 0));

  {  // no objects: zero counts
    ObjectDescriptorFactory factory(1.0, 5);
    Descriptor::Ptr descriptor = factory.construct(graph, node);
    ASSERT_TRUE(descriptor != nullptr);
    EXPECT_EQ(Eigen::VectorXf::Zero(5), descriptor->values);
    EXPECT_EQ(NodeSymbol('p', 0), descriptor->root_node);
    EXPECT_TRUE(descriptor->nodes.empty());
  }

  size_t next_object_index = 0;
  emplaceObjectNode(graph, Eigen::Vector3d(1.0, 2.0, 3.0), 1, next_object_index);
  emplaceObjectNode(graph, Eigen::Vector3d(1.0, 2.0, 3.0), 3, next_object_index);
  emplaceObjectNode(graph, Eigen::Vector3d(1.0, 2.0, 3.0), 4, next_object_index);
  graph.insertEdge(NodeSymbol('p', 0), NodeSymbol('o', 0));
  graph.insertEdge(NodeSymbol('p', 0), NodeSymbol('o', 1));
  graph.insertEdge(NodeSymbol('p', 1), NodeSymbol('o', 2));

  {  // some objects: expected counts
    ObjectDescriptorFactory factory(1.0, 5);
    Descriptor::Ptr descriptor = factory.construct(graph, node);
    ASSERT_TRUE(descriptor != nullptr);

    Eigen::VectorXf expected(5);
    expected << 0, 1, 0, 1, 1;
    EXPECT_EQ(expected, descriptor->values);
    EXPECT_EQ(NodeSymbol('p', 0), descriptor->root_node);

    std::set<NodeId> expected_nodes{
        NodeSymbol('o', 0), NodeSymbol('o', 1), NodeSymbol('o', 2)};
    EXPECT_EQ(expected_nodes, descriptor->nodes);
  }

  emplaceObjectNode(graph, Eigen::Vector3d(10.0, 20.0, 30.0), 2, next_object_index);
  emplaceObjectNode(graph, Eigen::Vector3d(1.0, 2.0, 3.0), 0, next_object_index);
  graph.insertEdge(NodeSymbol('p', 2), NodeSymbol('o', 3));
  // place outside radius, but object inside
  graph.insertEdge(NodeSymbol('p', 4), NodeSymbol('o', 4));

  {  // boundary conditions: expected counts
    ObjectDescriptorFactory factory(1.0, 5);
    Descriptor::Ptr descriptor = factory.construct(graph, node);
    ASSERT_TRUE(descriptor != nullptr);

    Eigen::VectorXf expected(5);
    expected << 1, 1, 0, 1, 1;
    EXPECT_EQ(expected, descriptor->values);
    EXPECT_EQ(NodeSymbol('p', 0), descriptor->root_node);

    std::set<NodeId> expected_nodes{
        NodeSymbol('o', 0), NodeSymbol('o', 1), NodeSymbol('o', 2), NodeSymbol('o', 4)};
    EXPECT_EQ(expected_nodes, descriptor->nodes);
  }
}

}  // namespace hydra::lcd
