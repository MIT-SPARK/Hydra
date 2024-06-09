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

#include "hydra_test/config_guard.h"

namespace hydra {

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

namespace {

void addNode(IsolatedSceneGraphLayer& layer, size_t node_id, size_t timestamp_ns) {
  auto attrs = std::make_unique<PlaceNodeAttributes>();
  attrs->position = Eigen::Vector3d::Zero();
  attrs->distance = 1.0;
  attrs->last_update_time_ns = timestamp_ns;
  layer.emplaceNode(node_id, std::move(attrs));
}

}  // namespace

TEST(RoomFinderTests, TestRoomPlaceEdges) {
  DynamicSceneGraph graph;
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
    room_finder.addRoomPlaceEdges(*graph_to_use);
    EXPECT_EQ(graph_to_use->numEdges(), 0u);
  }

  {  // test case: 1 valid, 1 clustered but no room
    TestableRoomFinder room_finder(config);

    ClusterResults results;
    results.fillFromInitialClusters({{"p0"_id}, {"p1"_id}});
    std::map<size_t, NodeId> map{{0, "r0"_id}};
    room_finder.setResults(results, map);

    auto graph_to_use = graph.clone();
    room_finder.addRoomPlaceEdges(*graph_to_use);
    EXPECT_EQ(graph_to_use->numEdges(), 1u);
    EXPECT_TRUE(graph_to_use->hasEdge("r0"_id, "p0"_id));
  }
}

TEST(RoomFinderTests, TestMakeRoomLayer) {
  test::ConfigGuard guard(false);
  PipelineConfig pipeline_config;
  GlobalInfo::init(pipeline_config);

  IsolatedSceneGraphLayer places(DsgLayers::PLACES);
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

}  // namespace hydra
