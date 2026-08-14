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
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <hydra/loop_closure/gnn_descriptors.h>

#include "hydra_test/resources.h"

namespace hydra::lcd {

namespace {

inline const DynamicSceneGraphNode& makeDefaultAgentNode(DynamicSceneGraph& graph) {
  using namespace std::chrono_literals;
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  graph.emplaceNode(2, 'a', 10ns, std::make_unique<AgentNodeAttributes>(q, t, 0));
  return graph.getDynamicNode(NodeSymbol('a', 0)).value();
}

inline void emplacePlaceNode(DynamicSceneGraph& graph,
                             const Eigen::Vector3d& pos,
                             double distance,
                             int num_basis_points,
                             size_t& next_index) {
  auto attrs = std::make_unique<PlaceNodeAttributes>(distance, num_basis_points);
  attrs->position = pos;
  graph.emplaceNode(DsgLayers::PLACES, NodeSymbol('p', next_index), std::move(attrs));
  ++next_index;
}

inline void emplaceObjectNode(DynamicSceneGraph& graph,
                              const Eigen::Vector3d& pos,
                              const Eigen::Vector3f& dims,
                              size_t& next_index) {
  auto attrs = std::make_unique<ObjectNodeAttributes>();
  attrs->position = pos;
  attrs->bounding_box.dimensions = dims;
  attrs->semantic_label = 0;
  graph.emplaceNode(DsgLayers::OBJECTS, NodeSymbol('o', next_index), std::move(attrs));
  ++next_index;
}

}  // namespace

TEST(GnnLcdTests, testEmptyDescriptor) {
  SubgraphConfig config;
  PlaceGnnDescriptor factory(test::get_resource_path("loop_closure/places.onnx"),
                             config);

  DynamicSceneGraph graph;
  const auto& root_node = makeDefaultAgentNode(graph);
  const auto descriptor = factory.construct(graph, root_node);
  EXPECT_EQ(descriptor, nullptr);
}

TEST(GnnLcdTests, testPlacesTensors) {
  SubgraphConfig config;
  config.max_radius_m = 5.0;
  config.min_radius_m = 2.0;
  config.min_nodes = 10;
  PlaceGnnDescriptor factory(test::get_resource_path("loop_closure/places.onnx"),
                             config);

  size_t node_idx = 0;
  DynamicSceneGraph graph;
  emplacePlaceNode(graph, Eigen::Vector3d(0.1, 0.0, 5.0), 1.1, 1, node_idx);
  emplacePlaceNode(graph, Eigen::Vector3d(0.2, 0.0, 5.0), 1.2, 2, node_idx);
  emplacePlaceNode(graph, Eigen::Vector3d(0.3, 0.0, 5.0), 1.3, 3, node_idx);
  emplacePlaceNode(graph, Eigen::Vector3d(0.4, 0.0, 5.0), 1.4, 4, node_idx);
  graph.insertEdge("p0"_id, "p1"_id);
  graph.insertEdge("p1"_id, "p2"_id);
  graph.insertEdge("p2"_id, "p3"_id);

  auto tensors = factory.makeInput(graph, {"p0"_id, "p1"_id, "p2"_id, "p3"_id});
  ASSERT_TRUE(tensors.count("x"));
  ASSERT_TRUE(tensors.count("edge_index"));

  auto x_map = tensors.at("x").map<float>();
  Eigen::MatrixXf x_expected(4, 5);
  ASSERT_EQ(x_expected.cols(), x_map.cols());
  ASSERT_EQ(x_expected.rows(), x_map.rows());
  x_expected << 0.1, 0.0, 5.0, 1.1, 1, 0.2, 0.0, 5.0, 1.2, 2, 0.3, 0.0, 5.0, 1.3, 3,
      0.4, 0.0, 5.0, 1.4, 4;
  for (int r = 0; r < x_map.rows(); ++r) {
    for (int c = 0; c < x_map.cols(); ++c) {
      EXPECT_NEAR(x_expected(r, c), x_map(r, c), 1.0e-9) << x_map << std::endl
                                                         << std::endl
                                                         << x_expected;
    }
  }

  Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic> edge_indices(2, 6);
  edge_indices << 0, 1, 1, 2, 2, 3, 1, 0, 2, 1, 3, 2;
  EXPECT_EQ(edge_indices, tensors.at("edge_index").map<int64_t>())
      << edge_indices << std::endl
      << std::endl
      << tensors.at("edge_index").map<int64_t>();
}

TEST(GnnLcdTests, testObjectTensors) {
  SubgraphConfig config;
  config.max_radius_m = 5.0;
  config.min_radius_m = 2.0;
  config.min_nodes = 10;

  Eigen::VectorXf fake_embedding(2);
  fake_embedding << 1.0, 2.0;

  ObjectGnnDescriptor factory(test::get_resource_path("loop_closure/objects.onnx"),
                              config,
                              0.1,
                              {{0, fake_embedding}});

  size_t node_idx = 0;
  DynamicSceneGraph graph;
  emplaceObjectNode(
      graph, Eigen::Vector3d(0.1, 3.0, 7.0), Eigen::Vector3f(0.2, 0.3, 0.4), node_idx);
  emplaceObjectNode(
      graph, Eigen::Vector3d(0.2, 3.0, 8.0), Eigen::Vector3f(0.3, 0.4, 0.5), node_idx);
  emplaceObjectNode(
      graph, Eigen::Vector3d(0.3, 3.0, 9.0), Eigen::Vector3f(0.4, 0.5, 0.6), node_idx);

  auto tensors = factory.makeInput(graph, {"o0"_id, "o1"_id, "o2"_id});
  ASSERT_TRUE(tensors.count("x"));
  ASSERT_TRUE(tensors.count("edge_index"));

  auto x_map = tensors.at("x").map<float>();
  Eigen::MatrixXf x_expected(3, 8);
  ASSERT_EQ(x_expected.cols(), x_map.cols());
  ASSERT_EQ(x_expected.rows(), x_map.rows());
  x_expected << 0.1, 3.0, 7.0, 0.2, 0.3, 0.4, 1.0, 2.0, 0.2, 3.0, 8.0, 0.3, 0.4, 0.5,
      1.0, 2.0, 0.3, 3.0, 9.0, 0.4, 0.5, 0.6, 1.0, 2.0;
  for (int r = 0; r < x_map.rows(); ++r) {
    for (int c = 0; c < x_map.cols(); ++c) {
      EXPECT_NEAR(x_expected(r, c), x_map(r, c), 1.0e-9) << x_map << std::endl
                                                         << std::endl
                                                         << x_expected;
    }
  }

  Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic> edge_indices(2, 6);
  edge_indices << 0, 1, 1, 2, 2, 3, 1, 0, 2, 1, 3, 2;
  EXPECT_EQ(edge_indices, tensors.at("edge_index").map<int64_t>())
      << edge_indices << std::endl
      << std::endl
      << tensors.at("edge_index").map<int64_t>();
}

TEST(GnnLcdTests, testLoadEmbeddings) {
  const auto embeddings =
      loadLabelEmbeddings(test::get_resource_path("loop_closure/test_embeddings.yaml"));

  ObjectGnnDescriptor::LabelEmbeddings expected_embeddings;
  expected_embeddings[0] = Eigen::VectorXf(4);
  expected_embeddings[0] << 1.0, 2.0, 3.0, 4.0;

  expected_embeddings[3] = Eigen::VectorXf(3);
  expected_embeddings[3] << 5.0, 6.0, 7.0;

  std::set<int> found_labels;
  for (const auto& label_vec_pair : embeddings) {
    found_labels.insert(label_vec_pair.first);
    const bool has_label = expected_embeddings.count(label_vec_pair.first);
    EXPECT_TRUE(has_label) << "missing: " << static_cast<int>(label_vec_pair.first);
    if (!has_label) {
      continue;
    }

    const auto& vec = label_vec_pair.second;
    const auto& expected_vec = expected_embeddings.at(label_vec_pair.first);
    ASSERT_EQ(vec.rows(), expected_vec.rows());
    EXPECT_NEAR((vec - expected_vec).norm(), 0.0, 1.0e-7);
  }

  std::set<int> expected_labels{0, 3};
  EXPECT_EQ(expected_labels, found_labels);
}

TEST(GnnLcdTests, testPlacesDescriptor) {
  SubgraphConfig config;
  config.max_radius_m = 5.0;
  config.min_radius_m = 2.0;
  config.min_nodes = 10;
  PlaceGnnDescriptor factory(test::get_resource_path("loop_closure/places.onnx"),
                             config);

  DynamicSceneGraph graph;
  const auto& root_node = makeDefaultAgentNode(graph);

  size_t node_idx = 0;
  emplacePlaceNode(graph, Eigen::Vector3d(1.0, 1.0, 1.0), 1.0, 1, node_idx);
  emplacePlaceNode(graph, Eigen::Vector3d(1.0, 1.0, 1.0), 1.0, 1, node_idx);
  emplacePlaceNode(graph, Eigen::Vector3d(1.0, 1.0, 1.0), 1.0, 1, node_idx);
  emplacePlaceNode(graph, Eigen::Vector3d(1.0, 1.0, 1.0), 1.0, 1, node_idx);
  emplacePlaceNode(graph, Eigen::Vector3d(1.0, 1.0, 1.0), 1.0, 1, node_idx);
  graph.insertEdge("p0"_id, "a0"_id);
  graph.insertEdge("p0"_id, "p1"_id);
  graph.insertEdge("p1"_id, "p2"_id);
  graph.insertEdge("p2"_id, "p3"_id);
  graph.insertEdge("p3"_id, "p4"_id);
  graph.insertEdge("p4"_id, "p0"_id);

  const auto descriptor = factory.construct(graph, root_node);
  ASSERT_TRUE(descriptor != nullptr);
  std::set<NodeId> expected_nodes{"p0"_id, "p1"_id, "p2"_id, "p3"_id, "p4"_id};
  EXPECT_EQ(descriptor->nodes, expected_nodes);

  ASSERT_EQ(descriptor->values.size(), 5);

  Eigen::VectorXf expected(5);
  expected << 5, 5, 5, 5, 5;
  EXPECT_NEAR((descriptor->values - expected).norm(), 0.0, 1.0e-5);
}

TEST(GnnLcdTests, testObjectDescriptor) {
  SubgraphConfig config;
  config.max_radius_m = 5.0;
  config.min_radius_m = 2.0;
  config.min_nodes = 10;

  Eigen::VectorXf fake_embedding(2);
  fake_embedding << 1.0, 1.0;

  ObjectGnnDescriptor factory(test::get_resource_path("loop_closure/objects.onnx"),
                              config,
                              0.1,
                              {{0, fake_embedding}});

  size_t node_idx = 0;
  DynamicSceneGraph graph;
  const auto& root_node = makeDefaultAgentNode(graph);

  emplacePlaceNode(graph, Eigen::Vector3d(1.0, 1.0, 1.0), 1.0, 1, node_idx);
  emplacePlaceNode(graph, Eigen::Vector3d(1.0, 1.0, 1.0), 1.0, 1, node_idx);
  emplacePlaceNode(graph, Eigen::Vector3d(1.0, 1.0, 1.0), 1.0, 1, node_idx);
  emplacePlaceNode(graph, Eigen::Vector3d(1.0, 1.0, 1.0), 1.0, 1, node_idx);
  emplacePlaceNode(graph, Eigen::Vector3d(1.0, 1.0, 1.0), 1.0, 1, node_idx);
  graph.insertEdge("p0"_id, "a0"_id);
  graph.insertEdge("p0"_id, "p1"_id);
  graph.insertEdge("p1"_id, "p2"_id);
  graph.insertEdge("p2"_id, "p3"_id);
  graph.insertEdge("p3"_id, "p4"_id);
  graph.insertEdge("p4"_id, "p0"_id);

  node_idx = 0;
  emplaceObjectNode(
      graph, Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Vector3f(1.0, 1.0, 1.0), node_idx);
  emplaceObjectNode(
      graph, Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Vector3f(1.0, 1.0, 1.0), node_idx);
  emplaceObjectNode(
      graph, Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Vector3f(1.0, 1.0, 1.0), node_idx);
  graph.insertEdge("p0"_id, "o0"_id);
  graph.insertEdge("p0"_id, "o1"_id);
  graph.insertEdge("p1"_id, "o2"_id);

  const auto descriptor = factory.construct(graph, root_node);
  ASSERT_TRUE(descriptor != nullptr);
  ASSERT_TRUE(!descriptor->is_null);
  std::set<NodeId> expected_nodes{"o0"_id, "o1"_id, "o2"_id};
  EXPECT_EQ(descriptor->nodes, expected_nodes);

  ASSERT_EQ(descriptor->values.size(), 8);

  Eigen::VectorXf expected(8);
  expected << 3, 3, 3, 3, 3, 3, 3, 3;
  EXPECT_NEAR((descriptor->values - expected).norm(), 0.0, 1.0e-5);
}

TEST(GnnLcdTests, testPlacesPosDescriptor) {
  SubgraphConfig config;
  config.max_radius_m = 5.0;
  config.min_radius_m = 2.0;
  config.min_nodes = 10;
  PlaceGnnDescriptor factory(
      test::get_resource_path("loop_closure/places_pos.onnx"), config, false);

  DynamicSceneGraph graph;
  const auto& root_node = makeDefaultAgentNode(graph);

  size_t node_idx = 0;
  // non-uniform edge weights
  emplacePlaceNode(graph, Eigen::Vector3d(0.0, 0.0, 0.0), 1.0, 2, node_idx);
  emplacePlaceNode(graph, Eigen::Vector3d(2.0, 0.0, 0.0), 1.0, 2, node_idx);
  emplacePlaceNode(graph, Eigen::Vector3d(3.0, 0.0, 0.0), 1.0, 2, node_idx);
  emplacePlaceNode(graph, Eigen::Vector3d(4.0, 0.0, 0.0), 1.0, 2, node_idx);
  emplacePlaceNode(graph, Eigen::Vector3d(3.0, 0.0, 0.0), 1.0, 2, node_idx);
  emplacePlaceNode(graph, Eigen::Vector3d(2.0, 0.0, 0.0), 1.0, 2, node_idx);
  graph.insertEdge("p0"_id, "a0"_id);
  graph.insertEdge("p0"_id, "p1"_id);
  graph.insertEdge("p1"_id, "p2"_id);
  graph.insertEdge("p2"_id, "p3"_id);
  graph.insertEdge("p3"_id, "p4"_id);
  graph.insertEdge("p4"_id, "p5"_id);
  graph.insertEdge("p5"_id, "p0"_id);

  const auto descriptor = factory.construct(graph, root_node);
  ASSERT_TRUE(descriptor != nullptr);
  std::set<NodeId> expected_nodes{"p0"_id, "p1"_id, "p2"_id, "p3"_id, "p4"_id, "p5"_id};
  EXPECT_EQ(descriptor->nodes, expected_nodes);

  ASSERT_EQ(descriptor->values.size(), 2);

  Eigen::VectorXf expected(2);
  expected << 5.9769, 11.95398;
  EXPECT_NEAR((descriptor->values - expected).norm(), 0.0, 1.0e-3);
}

TEST(GnnLcdTests, testObjectPosDescriptor) {
  SubgraphConfig config;
  config.max_radius_m = 5.0;
  config.min_radius_m = 2.0;
  config.min_nodes = 10;

  Eigen::VectorXf fake_embedding(2);
  fake_embedding << 4.0, 5.0;

  ObjectGnnDescriptor factory(test::get_resource_path("loop_closure/objects_pos.onnx"),
                              config,
                              1.6,
                              {{0, fake_embedding}},
                              false);

  DynamicSceneGraph graph;
  const auto& root_node = makeDefaultAgentNode(graph);

  size_t node_idx = 0;
  emplacePlaceNode(graph, Eigen::Vector3d(1.0, 1.0, 1.0), 1.0, 1, node_idx);
  graph.insertEdge("p0"_id, "a0"_id);

  node_idx = 0;
  emplaceObjectNode(
      graph, Eigen::Vector3d(0.5, 0.0, 0.0), Eigen::Vector3f(1.0, 2.0, 3.0), node_idx);
  emplaceObjectNode(
      graph, Eigen::Vector3d(2.0, 0.0, 0.0), Eigen::Vector3f(1.0, 2.0, 3.0), node_idx);
  emplaceObjectNode(
      graph, Eigen::Vector3d(3.0, 0.0, 0.0), Eigen::Vector3f(1.0, 2.0, 3.0), node_idx);
  emplaceObjectNode(
      graph, Eigen::Vector3d(4.0, 0.0, 0.0), Eigen::Vector3f(1.0, 2.0, 3.0), node_idx);
  emplaceObjectNode(
      graph, Eigen::Vector3d(3.0, 0.0, 0.0), Eigen::Vector3f(1.0, 2.0, 3.0), node_idx);
  emplaceObjectNode(
      graph, Eigen::Vector3d(2.0, 0.0, 0.0), Eigen::Vector3f(1.0, 2.0, 3.0), node_idx);
  graph.insertEdge("p0"_id, "o0"_id);
  graph.insertEdge("p0"_id, "o1"_id);
  graph.insertEdge("p0"_id, "o2"_id);
  graph.insertEdge("p0"_id, "o3"_id);
  graph.insertEdge("p0"_id, "o4"_id);
  graph.insertEdge("p0"_id, "o5"_id);

  const auto descriptor = factory.construct(graph, root_node);
  ASSERT_TRUE(descriptor != nullptr);
  ASSERT_TRUE(!descriptor->is_null);
  std::set<NodeId> expected_nodes{"o0"_id, "o1"_id, "o2"_id, "o3"_id, "o4"_id, "o5"_id};
  EXPECT_EQ(descriptor->nodes, expected_nodes);

  ASSERT_EQ(descriptor->values.size(), 5);

  // this is annoying to compute because we no longer have a linear graph
  const double weight_sum = 5.98231;
  Eigen::VectorXf expected(5);
  expected << 1 * weight_sum, 2 * weight_sum, 3 * weight_sum, 4 * weight_sum,
      5 * weight_sum;
  EXPECT_NEAR((descriptor->values - expected).norm(), 0.0, 1.0e-5);
}

}  // namespace hydra::lcd
