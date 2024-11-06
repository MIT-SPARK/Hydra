/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * all rights reserved
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
#include <hydra/frontend/graph_connector.h>
#include <spark_dsg/dynamic_scene_graph.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>
#include <spark_dsg/printing.h>

namespace hydra {

using namespace spark_dsg;

namespace {

// set to preserve ordering
std::set<EdgeKey> getEdges(const DynamicSceneGraph& graph) {
  std::set<EdgeKey> edges;
  for (const auto& [key, edge] : graph.interlayer_edges()) {
    edges.insert(key);
  }

  for (const auto& [key, edge] : graph.dynamic_interlayer_edges()) {
    edges.insert(key);
  }

  return edges;
}

NodeAttributes::Ptr makeAttrs(double x_coordinate) {
  const Eigen::Vector3d pos(x_coordinate, 0.0, 0.0);
  auto attrs = std::make_unique<NodeAttributes>(pos);
  attrs->is_active = true;
  return attrs;
}

void setupGraph(DynamicSceneGraph& graph) {
  for (size_t i = 0; i < 5; ++i) {
    graph.emplaceNode(DsgLayers::PLACES, NodeSymbol('p', i), makeAttrs(i));
    graph.emplaceNode(DsgLayers::OBJECTS, NodeSymbol('o', i), makeAttrs(i));
    graph.emplaceNode(DsgLayers::ROOMS, NodeSymbol('r', i), makeAttrs(i));
    graph.emplaceNode(
        DsgLayers::AGENTS, 'a', std::chrono::nanoseconds(100 * i), makeAttrs(i));
  }
}

GraphConnector::Config getDefaultConfig(bool include_static, bool include_dynamic) {
  GraphConnector::Config config;
  config.layers.clear();
  config.layers.push_back(LayerConnector::Config{
      DsgLayers::PLACES, {{DsgLayers::OBJECTS, include_static, include_dynamic}}});
  return config;
}

}  // namespace

TEST(GraphConnector, TestStatic) {
  DynamicSceneGraph graph;
  setupGraph(graph);

  GraphConnector connector(getDefaultConfig(true, false));
  connector.connect(graph);

  std::set<EdgeKey> expected_edges{{"p0"_id, "o0"_id},
                                   {"p1"_id, "o1"_id},
                                   {"p2"_id, "o2"_id},
                                   {"p3"_id, "o3"_id},
                                   {"p4"_id, "o4"_id}};
  EXPECT_EQ(expected_edges, getEdges(graph));
}

TEST(GraphConnector, TestDynamic) {
  DynamicSceneGraph graph;
  setupGraph(graph);

  GraphConnector connector(getDefaultConfig(false, true));
  connector.connect(graph);

  std::set<EdgeKey> expected_edges{{"p0"_id, "a0"_id},
                                   {"p1"_id, "a1"_id},
                                   {"p2"_id, "a2"_id},
                                   {"p3"_id, "a3"_id},
                                   {"p4"_id, "a4"_id}};
  EXPECT_EQ(expected_edges, getEdges(graph));
}

TEST(GraphConnector, TestNewNodes) {
  DynamicSceneGraph graph;
  setupGraph(graph);

  GraphConnector connector(getDefaultConfig(true, true));
  connector.connect(graph);

  // check that initial connections make sense
  std::set<EdgeKey> expected_edges{{"p0"_id, "o0"_id},
                                   {"p1"_id, "o1"_id},
                                   {"p2"_id, "o2"_id},
                                   {"p3"_id, "o3"_id},
                                   {"p4"_id, "o4"_id},
                                   {"p0"_id, "a0"_id},
                                   {"p1"_id, "a1"_id},
                                   {"p2"_id, "a2"_id},
                                   {"p3"_id, "a3"_id},
                                   {"p4"_id, "a4"_id}};
  EXPECT_EQ(expected_edges, getEdges(graph));

  // no change in graph -> no change in edges
  connector.connect(graph);
  EXPECT_EQ(expected_edges, getEdges(graph));

  // add new nodes
  graph.emplaceNode(DsgLayers::OBJECTS, "o5"_id, makeAttrs(3.0));
  graph.emplaceNode(
      DsgLayers::AGENTS, 'a', std::chrono::nanoseconds(500), makeAttrs(2.0));
  graph.emplaceNode(DsgLayers::ROOMS, "r5"_id, makeAttrs(4.0));
  connector.connect(graph);

  // new nodes -> new connections
  expected_edges.insert(EdgeKey("p2"_id, "a5"_id));
  expected_edges.insert(EdgeKey("p3"_id, "o5"_id));
  EXPECT_EQ(expected_edges, getEdges(graph));
}

TEST(GraphConnector, TestRewiring) {
  DynamicSceneGraph graph;
  setupGraph(graph);

  GraphConnector connector(getDefaultConfig(true, false));
  connector.connect(graph);

  std::set<EdgeKey> expected_edges{{"p0"_id, "o0"_id},
                                   {"p1"_id, "o1"_id},
                                   {"p2"_id, "o2"_id},
                                   {"p3"_id, "o3"_id},
                                   {"p4"_id, "o4"_id}};
  EXPECT_EQ(expected_edges, getEdges(graph));

  for (size_t i = 0; i < 5; ++i) {
    const auto& node = graph.getNode(NodeSymbol('p', i));
    node.attributes().position = Eigen::Vector3d(4.0 - i, 0.0, 0.0);
  }

  connector.connect(graph);

  expected_edges = {{"p0"_id, "o4"_id},
                    {"p1"_id, "o3"_id},
                    {"p2"_id, "o2"_id},
                    {"p3"_id, "o1"_id},
                    {"p4"_id, "o0"_id}};
  EXPECT_EQ(expected_edges, getEdges(graph));
}

TEST(GraphConnector, TestArchiving) {
  DynamicSceneGraph graph;
  setupGraph(graph);

  GraphConnector connector(getDefaultConfig(true, false));
  connector.connect(graph);

  std::set<EdgeKey> expected_edges{{"p0"_id, "o0"_id},
                                   {"p1"_id, "o1"_id},
                                   {"p2"_id, "o2"_id},
                                   {"p3"_id, "o3"_id},
                                   {"p4"_id, "o4"_id}};
  EXPECT_EQ(expected_edges, getEdges(graph));

  for (size_t i = 0; i < 5; ++i) {
    const auto& node = graph.getNode(NodeSymbol('p', i));
    node.attributes().position = Eigen::Vector3d(4.0 - i, 0.0, 0.0);
    if (i == 4) {
      node.attributes().is_active = false;
    }
  }

  connector.connect(graph);

  expected_edges = {{"p1"_id, "o3"_id},
                    {"p2"_id, "o2"_id},
                    {"p3"_id, "o1"_id},
                    {"p3"_id, "o0"_id},
                    {"p4"_id, "o4"_id}};
  EXPECT_EQ(expected_edges, getEdges(graph));
}

}  // namespace hydra
