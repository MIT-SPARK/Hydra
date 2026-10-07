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
#include <hydra/rooms/room_finder.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>

#include <limits>

#include "hydra_test/config_guard.h"

using namespace spark_dsg;

namespace hydra {
namespace {

class TestableRoomFinder : public RoomFinder {
 public:
  explicit TestableRoomFinder(const RoomFinderConfig& config) : RoomFinder(config) {}

  virtual ~TestableRoomFinder() = default;

  using RoomFinder::makeRoomLayer;

  void setResults(const ClusterResults& new_results,
                  const std::map<size_t, NodeId> room_map) {
    last_results_ = new_results;
    cluster_room_map_ = room_map;
  }

  const std::map<size_t, NodeId>& getLabelMap() const { return cluster_room_map_; }
};

// Use a nonstandard prefix for real places and a normal place prefix for a frontier.
void fillMixedPlaces(SceneGraphLayer& layer) {
  for (size_t i = 0; i < 6; ++i) {
    auto attrs = std::make_unique<PlaceNodeAttributes>(2.0, 3);
    attrs->position = Eigen::Vector3d(i, 0.0, 0.0);
    attrs->last_update_time_ns = i;
    layer.emplaceNode(NodeSymbol('x', i), std::move(attrs));
  }

  auto trav = std::make_unique<TraversabilityNodeAttributes>();
  trav->distance = 2.0;
  trav->position = Eigen::Vector3d(6.0, 0.0, 0.0);
  layer.emplaceNode("t0"_id, std::move(trav));
  auto frontier = std::make_unique<PlaceNodeAttributes>(0.001, 0);
  frontier->real_place = false;
  layer.emplaceNode("p0"_id, std::move(frontier));
  layer.emplaceNode("a0"_id, std::make_unique<NodeAttributes>());
  layer.emplaceNode("p1"_id, std::make_unique<PlaceNodeAttributes>(0.0, 0));
  layer.emplaceNode("p2"_id, std::make_unique<PlaceNodeAttributes>(-1.0, 0));
  layer.insertEdge("x0"_id, "x1"_id, std::make_unique<EdgeAttributes>(0.8));
  layer.insertEdge("x1"_id, "x2"_id, std::make_unique<EdgeAttributes>(0.9));
  layer.insertEdge("x2"_id, "x3"_id, std::make_unique<EdgeAttributes>(0.2));
  layer.insertEdge("x3"_id, "x4"_id, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge("x4"_id, "x5"_id, std::make_unique<EdgeAttributes>(1.1));
  layer.insertEdge("x5"_id, "t0"_id, std::make_unique<EdgeAttributes>(1.2));
  // Rejected bridge, rejected leaf, and rejected-only component.
  layer.insertEdge("x0"_id, "p0"_id);
  layer.insertEdge("p0"_id, "x5"_id);
  layer.insertEdge("x1"_id, "a0"_id);
  layer.insertEdge("p1"_id, "p2"_id);
}

RoomFinderConfig smallRoomsConfig() {
  RoomFinderConfig config;
  config.min_component_size = 2;
  config.min_room_size = 2;
  config.max_dilation_m = 1.1;
  return config;
}

void addNode(SceneGraphLayer& layer, size_t node_id, size_t timestamp_ns) {
  auto attrs = std::make_unique<PlaceNodeAttributes>();
  attrs->position = Eigen::Vector3d::Zero();
  attrs->distance = 1.0;
  attrs->last_update_time_ns = timestamp_ns;
  layer.emplaceNode(node_id, std::move(attrs));
}

}  // namespace

TEST(RoomFinderTests, TestRoomPlaceEdges) {
  SceneGraph graph;
  graph.emplaceNode(DsgLayers::ROOMS, "r0"_id, std::make_unique<NodeAttributes>());
  graph.emplaceNode(DsgLayers::ROOMS, "r1"_id, std::make_unique<NodeAttributes>());
  graph.emplaceNode(DsgLayers::ROOMS, "r2"_id, std::make_unique<NodeAttributes>());
  graph.emplaceNode(DsgLayers::PLACES, "p0"_id, std::make_unique<NodeAttributes>());
  graph.emplaceNode(DsgLayers::PLACES, "p1"_id, std::make_unique<NodeAttributes>());
  graph.emplaceNode(DsgLayers::PLACES, "p2"_id, std::make_unique<NodeAttributes>());
  graph.emplaceNode(DsgLayers::PLACES, "p3"_id, std::make_unique<NodeAttributes>());
  graph.emplaceNode(DsgLayers::PLACES, "p4"_id, std::make_unique<NodeAttributes>());

  RoomFinderConfig config;

  {  // test case: no clusters
    TestableRoomFinder room_finder(config);

    ClusterResults results;
    results.fillFromInitialClusters({});
    std::map<size_t, NodeId> map;
    room_finder.setResults(results, map);

    auto graph_to_use = graph.clone();
    room_finder.addRoomPlaceEdges(*graph_to_use, DsgLayers::PLACES);
    EXPECT_EQ(graph_to_use->numEdges(), 0u);
  }

  {  // test case: 1 valid, 1 clustered but no room
    TestableRoomFinder room_finder(config);

    ClusterResults results;
    results.fillFromInitialClusters({{"p0"_id}, {"p1"_id}});
    std::map<size_t, NodeId> map{{0, "r0"_id}};
    room_finder.setResults(results, map);

    auto graph_to_use = graph.clone();
    room_finder.addRoomPlaceEdges(*graph_to_use, DsgLayers::PLACES);
    EXPECT_EQ(graph_to_use->numEdges(), 1u);
    EXPECT_TRUE(graph_to_use->hasEdge("r0"_id, "p0"_id));
  }
}

TEST(RoomFinderTests, TestMakeRoomLayer) {
  test::ConfigGuard guard(false);
  PipelineConfig pipeline_config;
  GlobalInfo::init(pipeline_config);

  SceneGraphLayer places(DsgLayers::PLACES);
  addNode(places, 0, 3);
  addNode(places, 1, 4);
  addNode(places, 2, 10);
  addNode(places, 3, 2);
  addNode(places, 4, 5);
  addNode(places, 5, 50);

  RoomFinderConfig config;
  config.min_room_size = 2;

  TestableRoomFinder room_finder(config);

  ClusterResults results;
  results.fillFromInitialClusters({{0, 1, 2}, {3, 4}, {5}});
  std::map<size_t, NodeId> map;
  room_finder.setResults(results, map);

  const auto rooms = room_finder.makeRoomLayer(places);
  ASSERT_TRUE(rooms != nullptr);
  EXPECT_EQ(rooms->numNodes(), 2u);
  EXPECT_TRUE(rooms->hasNode("R0"_id));
  EXPECT_TRUE(rooms->hasNode("R1"_id));
  // room ids should be flipped: second cluster is older than first
  std::map<size_t, NodeId> expected_labels{{0, "R1"_id}, {1, "R0"_id}};
  EXPECT_EQ(expected_labels, room_finder.getLabelMap());
}

TEST(RoomFinderTests, DistanceEligibility) {
  SceneGraphLayer layer(DsgLayers::PLACES);
  layer.emplaceNode("x0"_id, std::make_unique<PlaceNodeAttributes>(2.0, 3));
  auto trav = std::make_unique<TraversabilityNodeAttributes>();
  trav->distance = 2.0;
  layer.emplaceNode("t0"_id, std::move(trav));
  auto frontier = std::make_unique<PlaceNodeAttributes>(0.001, 0);
  frontier->real_place = false;
  layer.emplaceNode("p0"_id, std::move(frontier));
  layer.emplaceNode("a0"_id, std::make_unique<NodeAttributes>());
  const DistanceAdaptor distance;
  EXPECT_EQ(distance(layer.getNode("x0"_id)), 2.0);
  EXPECT_EQ(distance(layer.getNode("t0"_id)), 2.0);
  EXPECT_FALSE(distance(layer.getNode("p0"_id)));
  EXPECT_FALSE(distance(layer.getNode("a0"_id)));
  for (const auto value : {0.0,
                           -1.0,
                           std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity()}) {
    layer.getNode("x0"_id).attributes<PlaceNodeAttributes>().distance = value;
    layer.getNode("t0"_id).attributes<TraversabilityNodeAttributes>().distance = value;
    EXPECT_FALSE(distance(layer.getNode("x0"_id)));
    EXPECT_FALSE(distance(layer.getNode("t0"_id)));
  }
}

TEST(RoomFinderTests, FilteredCloneEquivalence) {
  test::ConfigGuard guard(false);
  GlobalInfo::init(PipelineConfig{});
  SceneGraphLayer places(DsgLayers::PLACES);
  fillMixedPlaces(places);
  const auto filtered = places.clone([](const auto& node) {
    return NodeSymbol(node.id).category() == 'x' || node.id == "t0"_id;
  });
  for (const auto mode : {RoomClusterMode::NONE,
                          RoomClusterMode::NEIGHBORS,
                          RoomClusterMode::MODULARITY,
                          RoomClusterMode::MODULARITY_DISTANCE}) {
    SCOPED_TRACE(static_cast<int>(mode));
    auto config = smallRoomsConfig();
    config.clustering_mode = mode;
    RoomFinder finder(config);
    RoomFinder reference(config);
    const auto actual = finder.findRooms(places);
    const auto expected = reference.findRooms(*filtered);
    ASSERT_TRUE(actual);
    ASSERT_TRUE(expected);
    ASSERT_GT(expected->numNodes(), 0u);
    EXPECT_EQ(actual->numNodes(), expected->numNodes());
    EXPECT_EQ(actual->numEdges(), expected->numEdges());
    for (const auto& node : expected->nodes()) {
      ASSERT_TRUE(actual->hasNode(node.id));
      EXPECT_TRUE(actual->getNode(node.id).attributes().position.isApprox(
          node.attributes().position));
    }

    for (const auto& edge : expected->edges()) {
      EXPECT_TRUE(actual->hasEdge(edge.source, edge.target));
    }

    RoomFinder::ClusterMap assignments;
    RoomFinder::ClusterMap expected_assignments;
    finder.fillClusterMap(places, assignments);
    reference.fillClusterMap(*filtered, expected_assignments);
    ASSERT_FALSE(expected_assignments.empty());
    EXPECT_EQ(assignments, expected_assignments);

    SceneGraphLayer rejected(DsgLayers::PLACES);
    rejected.emplaceNode("x0"_id, std::make_unique<PlaceNodeAttributes>(0.0, 0));
    EXPECT_FALSE(finder.findRooms(rejected));
    finder.fillClusterMap(places, assignments);
    EXPECT_TRUE(assignments.empty());
    EXPECT_FALSE(finder.findRooms(SceneGraphLayer(DsgLayers::PLACES)));
  }
}

TEST(RoomFinderTests, ClusteringRejectsInvalidSeeds) {
  SceneGraphLayer places(DsgLayers::PLACES);
  places.emplaceNode("x0"_id, std::make_unique<PlaceNodeAttributes>(2.0, 3));
  auto frontier = std::make_unique<PlaceNodeAttributes>(0.001, 0);
  frontier->real_place = false;
  places.emplaceNode("p0"_id, std::move(frontier));
  places.emplaceNode("a0"_id, std::make_unique<NodeAttributes>());
  const InitialClusters seeds{{"x0"_id, "p0"_id}, {"a0"_id}};
  for (const auto& result : {clusterGraphByNeighbors(places, seeds),
                             clusterGraphByModularity(places, seeds)}) {
    const std::map<NodeId, size_t> expected_labels{{"x0"_id, 0}};
    EXPECT_EQ(result.labels, expected_labels);
  }
}

}  // namespace hydra
