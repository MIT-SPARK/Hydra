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
#include <hydra/loop_closure/subgraph_extraction.h>

namespace hydra {

namespace {

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

TEST(GnnLcdTests, testGetSubgraphPlaces) {
  DynamicSceneGraph graph;

  size_t node_idx = 0;
  emplacePlaceNode(graph, Eigen::Vector3d(0.1, 0.0, 0.0), 0.1, 1, node_idx);
  emplacePlaceNode(graph, Eigen::Vector3d(0.2, 0.0, 0.0), 0.1, 2, node_idx);
  emplacePlaceNode(graph, Eigen::Vector3d(0.2, 0.0, 0.0), 0.1, 3, node_idx);
  emplacePlaceNode(graph, Eigen::Vector3d(0.3, 0.0, 0.0), 0.1, 4, node_idx);
  emplacePlaceNode(graph, Eigen::Vector3d(0.4, 0.0, 0.0), 0.1, 5, node_idx);

  graph.insertEdge("p0"_id, "p1"_id);
  graph.insertEdge("p1"_id, "p2"_id);
  graph.insertEdge("p2"_id, "p3"_id);
  graph.insertEdge("p3"_id, "p4"_id);

  {  // test case 1: all valid
    SubgraphConfig config;
    config.fixed_radius = false;
    config.max_radius_m = 5.0;
    config.min_radius_m = 2.0;
    config.min_nodes = 10;
    auto result = getSubgraphNodes(config, graph, "p0"_id, true);
    const decltype(result) expected{"p0"_id, "p1"_id, "p2"_id, "p3"_id, "p4"_id};
    EXPECT_EQ(result, expected);
  }

  {  // test case 2: single node
    SubgraphConfig config;
    config.fixed_radius = false;
    config.max_radius_m = 0.005;
    config.min_radius_m = 0.005;
    config.min_nodes = 10;
    auto result = getSubgraphNodes(config, graph, "p0"_id, true);
    const decltype(result) expected{"p0"_id};
    EXPECT_EQ(result, expected);
  }

  {  // test case 3: min nodes
    SubgraphConfig config;
    config.fixed_radius = false;
    config.max_radius_m = 1.0;
    config.min_radius_m = 0.04;
    config.min_nodes = 2;
    auto result = getSubgraphNodes(config, graph, "p0"_id, true);
    const decltype(result) expected{"p0"_id, "p1"_id, "p2"_id};
    EXPECT_EQ(result, expected);
  }

  {  // test case 3: max radius
    SubgraphConfig config;
    config.fixed_radius = false;
    config.max_radius_m = 0.3;
    config.min_radius_m = 0.04;
    config.min_nodes = 20;
    auto result = getSubgraphNodes(config, graph, "p0"_id, true);
    const decltype(result) expected{"p0"_id, "p1"_id, "p2"_id, "p3"_id};
    EXPECT_EQ(result, expected);
  }
}

TEST(GnnLcdTests, testGetSubgraphObjects) {
  DynamicSceneGraph graph;

  size_t p_idx = 0;
  emplacePlaceNode(graph, Eigen::Vector3d(0.1, 0.0, 0.0), 0.1, 1, p_idx);
  emplacePlaceNode(graph, Eigen::Vector3d(0.2, 0.0, 0.0), 0.1, 2, p_idx);
  emplacePlaceNode(graph, Eigen::Vector3d(0.2, 0.0, 0.0), 0.1, 3, p_idx);
  emplacePlaceNode(graph, Eigen::Vector3d(0.3, 0.0, 0.0), 0.1, 4, p_idx);
  emplacePlaceNode(graph, Eigen::Vector3d(0.4, 0.0, 0.0), 0.1, 5, p_idx);

  size_t o_idx = 0;
  emplaceObjectNode(
      graph, Eigen::Vector3d(0.15, 0.0, 0.0), Eigen::Vector3f::Zero(), o_idx);
  emplaceObjectNode(
      graph, Eigen::Vector3d(0.2, 0.0, 0.0), Eigen::Vector3f::Zero(), o_idx);
  emplaceObjectNode(
      graph, Eigen::Vector3d(0.2, 0.0, 0.0), Eigen::Vector3f::Zero(), o_idx);
  emplaceObjectNode(
      graph, Eigen::Vector3d(0.3, 0.0, 0.0), Eigen::Vector3f::Zero(), o_idx);
  emplaceObjectNode(
      graph, Eigen::Vector3d(0.4, 0.0, 0.0), Eigen::Vector3f::Zero(), o_idx);
  emplaceObjectNode(
      graph, Eigen::Vector3d(0.5, 0.0, 0.0), Eigen::Vector3f::Zero(), o_idx);
  emplaceObjectNode(
      graph, Eigen::Vector3d(0.6, 0.0, 0.0), Eigen::Vector3f::Zero(), o_idx);

  graph.insertEdge("p0"_id, "p1"_id);
  graph.insertEdge("p1"_id, "p2"_id);
  graph.insertEdge("p2"_id, "p3"_id);
  graph.insertEdge("p3"_id, "p4"_id);
  graph.insertEdge("p1"_id, "o0"_id);
  graph.insertEdge("p1"_id, "o1"_id);
  graph.insertEdge("p2"_id, "o6"_id);
  graph.insertEdge("p2"_id, "o3"_id);
  graph.insertEdge("p3"_id, "o4"_id);
  graph.insertEdge("p3"_id, "o5"_id);
  graph.insertEdge("p4"_id, "o2"_id);

  {  // test case 1: all valid
    SubgraphConfig config;
    config.fixed_radius = false;
    config.min_radius_m = 2.0;
    config.max_radius_m = 5.0;
    config.min_nodes = 10;
    auto result = getSubgraphNodes(config, graph, "p0"_id, false);
    const decltype(result) expected{
        "o0"_id, "o1"_id, "o2"_id, "o3"_id, "o4"_id, "o5"_id, "o6"_id};
    EXPECT_EQ(result, expected);
  }

  {  // test case 1: empty
    SubgraphConfig config;
    config.fixed_radius = false;
    config.min_radius_m = 0.005;
    config.max_radius_m = 0.005;
    config.min_nodes = 10;
    auto result = getSubgraphNodes(config, graph, "p0"_id, false);
    const decltype(result) expected{};
    EXPECT_EQ(result, expected);
  }

  {  // test case 1: min nodes
    SubgraphConfig config;
    config.fixed_radius = false;
    config.min_radius_m = 0.005;
    config.max_radius_m = 2.0;
    config.min_nodes = 2;
    auto result = getSubgraphNodes(config, graph, "p0"_id, false);
    const decltype(result) expected{"o0"_id, "o1"_id, "o2"_id};
    EXPECT_EQ(result, expected);
  }

  {  // test case 1: max distance
    SubgraphConfig config;
    config.fixed_radius = false;
    config.min_radius_m = 0.005;
    config.max_radius_m = 0.21;
    config.min_nodes = 20;
    auto result = getSubgraphNodes(config, graph, "p0"_id, false);
    const decltype(result) expected{"o0"_id, "o1"_id, "o2"_id, "o3"_id};
    EXPECT_EQ(result, expected);
  }
}

}  // namespace hydra
