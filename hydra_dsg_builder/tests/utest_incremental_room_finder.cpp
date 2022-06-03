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
#include <hydra_dsg_builder/incremental_room_finder.h>

#include <gtest/gtest.h>

namespace hydra {
namespace incremental {

class TestableRoomFinder : public RoomFinder {
 public:
  explicit TestableRoomFinder(const RoomFinder::Config& config) : RoomFinder(config) {}

  virtual ~TestableRoomFinder() = default;

  using RoomFinder::getThresholds;
  using RoomFinder::updateRoomsFromClusters;
};

TEST(IncrementalRoomsTests, ActiveSubgraphEmptyLayer) {
  DynamicSceneGraph graph({1}, 0);

  ActiveNodeSet nodes{1, 2, 3, 4};
  IsolatedSceneGraphLayer::Ptr new_layer = getActiveSubgraph(graph, 1, nodes);
  ASSERT_TRUE(new_layer != nullptr);
  EXPECT_EQ(1u, new_layer->id);
  EXPECT_EQ(0u, new_layer->numNodes());
  EXPECT_EQ(0u, new_layer->numEdges());
}

TEST(IncrementalRoomsTests, ActiveSubgraphEmptyNodes) {
  DynamicSceneGraph graph({1}, 0);
  graph.emplaceNode(1, 1, std::make_unique<PlaceNodeAttributes>());
  graph.emplaceNode(1, 2, std::make_unique<PlaceNodeAttributes>());
  graph.emplaceNode(1, 3, std::make_unique<PlaceNodeAttributes>());
  graph.emplaceNode(1, 4, std::make_unique<PlaceNodeAttributes>());

  ActiveNodeSet nodes;
  IsolatedSceneGraphLayer::Ptr new_layer = getActiveSubgraph(graph, 1, nodes);
  ASSERT_TRUE(new_layer != nullptr);
  EXPECT_EQ(1u, new_layer->id);
  EXPECT_EQ(0u, new_layer->numNodes());
  EXPECT_EQ(0u, new_layer->numEdges());
}

TEST(IncrementalRoomsTests, ActiveSubgraphSomeNodes) {
  DynamicSceneGraph graph({1}, 0);
  graph.emplaceNode(1, 1, std::make_unique<PlaceNodeAttributes>());
  graph.emplaceNode(1, 2, std::make_unique<PlaceNodeAttributes>());
  graph.emplaceNode(1, 3, std::make_unique<PlaceNodeAttributes>());
  graph.emplaceNode(1, 4, std::make_unique<PlaceNodeAttributes>());

  ActiveNodeSet nodes{2, 3};
  IsolatedSceneGraphLayer::Ptr new_layer = getActiveSubgraph(graph, 1, nodes);
  ASSERT_TRUE(new_layer != nullptr);
  EXPECT_EQ(1u, new_layer->id);
  EXPECT_EQ(2u, new_layer->numNodes());
  EXPECT_EQ(0u, new_layer->numEdges());
}

TEST(IncrementalRoomsTests, ActiveSubgraphNodesAndEdges) {
  DynamicSceneGraph graph({1}, 0);
  graph.emplaceNode(1, 1, std::make_unique<PlaceNodeAttributes>());
  graph.emplaceNode(1, 2, std::make_unique<PlaceNodeAttributes>());
  graph.emplaceNode(1, 3, std::make_unique<PlaceNodeAttributes>());
  graph.emplaceNode(1, 4, std::make_unique<PlaceNodeAttributes>());
  graph.insertEdge(1, 2);
  graph.insertEdge(1, 4);
  graph.insertEdge(2, 3);

  ActiveNodeSet nodes{2, 3};
  IsolatedSceneGraphLayer::Ptr new_layer = getActiveSubgraph(graph, 1, nodes);
  ASSERT_TRUE(new_layer != nullptr);
  EXPECT_EQ(1u, new_layer->id);
  EXPECT_EQ(2u, new_layer->numNodes());
  EXPECT_EQ(1u, new_layer->numEdges());
}

TEST(IncrementalRoomsTests, TestThresholds) {
  {  // just start and end
    TestableRoomFinder::Config config;
    config.min_dilation_m = 0.2;
    config.max_dilation_m = 0.5;
    config.num_steps = 0;

    TestableRoomFinder finder(config);

    std::vector<double> expected{0.2, 0.5};
    std::vector<double> result = finder.getThresholds();
    EXPECT_EQ(expected, result);
  }

  {  // full schedule
    TestableRoomFinder::Config config;
    config.min_dilation_m = 0.2;
    config.max_dilation_m = 0.5;
    config.num_steps = 2;

    TestableRoomFinder finder(config);

    std::vector<double> expected{0.2, 0.3, 0.4, 0.5};
    std::vector<double> result = finder.getThresholds();
    ASSERT_EQ(expected.size(), result.size());
    for (size_t i = 0; i < expected.size(); ++i) {
      EXPECT_NEAR(expected[i], result[i], 1.0e-7)
          << " step " << i << " disagrees: " << expected[i] << " != " << result[i];
    }
  }
}

TEST(IncrementalRoomsTests, UpdateFromEmptyClustersCorrect) {
  const LayerId mesh_layer_id = 1;
  const std::map<LayerId, char>& layer_id_map{{DsgLayers::OBJECTS, 'o'},
                                              {DsgLayers::PLACES, 'p'},
                                              {DsgLayers::ROOMS, 'r'},
                                              {DsgLayers::BUILDINGS, 'b'}};

  SharedDsgInfo::Ptr dsg(new SharedDsgInfo(layer_id_map, mesh_layer_id));

  RoomFinder::Config config;
  TestableRoomFinder finder(config);

  ClusterResults clusters;
  RoomMap previous_rooms;
  finder.updateRoomsFromClusters(*dsg, clusters, previous_rooms, {});

  const auto& rooms = dsg->graph->getLayer(DsgLayers::ROOMS);
  EXPECT_EQ(0u, rooms.numNodes());
  EXPECT_EQ(0u, rooms.numEdges());
}

TEST(IncrementalRoomsTests, UpdateFromClustersWithPruningCorrect) {
  const LayerId mesh_layer_id = 1;
  const std::map<LayerId, char>& layer_id_map{{DsgLayers::OBJECTS, 'o'},
                                              {DsgLayers::PLACES, 'p'},
                                              {DsgLayers::ROOMS, 'r'},
                                              {DsgLayers::BUILDINGS, 'b'}};

  SharedDsgInfo::Ptr dsg(new SharedDsgInfo(layer_id_map, mesh_layer_id));
  dsg->graph->emplaceNode(
      DsgLayers::PLACES, NodeSymbol('p', 3), std::make_unique<PlaceNodeAttributes>());

  RoomFinder::Config config;
  config.min_room_size = 1;
  TestableRoomFinder finder(config);

  ClusterResults results;
  results.clusters[0] = {NodeSymbol('p', 0), NodeSymbol('p', 1), NodeSymbol('p', 2)};
  results.clusters[1] = {NodeSymbol('p', 3), NodeSymbol('p', 4)};

  RoomMap previous_rooms;
  finder.updateRoomsFromClusters(*dsg, results, previous_rooms, {});

  const auto& rooms = dsg->graph->getLayer(DsgLayers::ROOMS);
  EXPECT_EQ(1u, rooms.numNodes());
  EXPECT_TRUE(rooms.hasNode(NodeSymbol('R', 0)));
  EXPECT_EQ(0u, rooms.numEdges());
}

TEST(IncrementalRoomsTests, UpdateFromClustersWithMinSizeCorrect) {
  const LayerId mesh_layer_id = 1;
  const std::map<LayerId, char>& layer_id_map{{DsgLayers::OBJECTS, 'o'},
                                              {DsgLayers::PLACES, 'p'},
                                              {DsgLayers::ROOMS, 'r'},
                                              {DsgLayers::BUILDINGS, 'b'}};

  SharedDsgInfo::Ptr dsg(new SharedDsgInfo(layer_id_map, mesh_layer_id));
  dsg->graph->emplaceNode(
      DsgLayers::PLACES, NodeSymbol('p', 3), std::make_unique<PlaceNodeAttributes>());
  dsg->graph->emplaceNode(
      DsgLayers::PLACES, NodeSymbol('p', 5), std::make_unique<PlaceNodeAttributes>());
  dsg->graph->emplaceNode(
      DsgLayers::PLACES, NodeSymbol('p', 6), std::make_unique<PlaceNodeAttributes>());

  RoomFinder::Config config;
  config.min_room_size = 2;
  TestableRoomFinder finder(config);

  ClusterResults results;
  results.clusters[0] = {NodeSymbol('p', 0), NodeSymbol('p', 1), NodeSymbol('p', 2)};
  results.clusters[1] = {NodeSymbol('p', 3), NodeSymbol('p', 4)};
  results.clusters[2] = {NodeSymbol('p', 5), NodeSymbol('p', 6)};

  RoomMap previous_rooms;
  finder.updateRoomsFromClusters(*dsg, results, previous_rooms, {});

  const auto& rooms = dsg->graph->getLayer(DsgLayers::ROOMS);
  EXPECT_EQ(1u, rooms.numNodes());
  EXPECT_TRUE(rooms.hasNode(NodeSymbol('R', 0)));
  EXPECT_EQ(0u, rooms.numEdges());
}

TEST(IncrementalRoomsTests, UpdateFromClustersWithAssocationCorrect) {
  const LayerId mesh_layer_id = 1;
  const std::map<LayerId, char>& layer_id_map{{DsgLayers::OBJECTS, 'o'},
                                              {DsgLayers::PLACES, 'p'},
                                              {DsgLayers::ROOMS, 'r'},
                                              {DsgLayers::BUILDINGS, 'b'}};

  SharedDsgInfo::Ptr dsg(new SharedDsgInfo(layer_id_map, mesh_layer_id));
  std::vector<NodeId> to_add{3, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15, 16};
  for (const auto& idx : to_add) {
    dsg->graph->emplaceNode(DsgLayers::PLACES,
                            NodeSymbol('p', idx),
                            std::make_unique<PlaceNodeAttributes>());
  }

  // previous rooms
  dsg->graph->emplaceNode(
      DsgLayers::ROOMS, NodeSymbol('R', 3), std::make_unique<SemanticNodeAttributes>());
  dsg->graph->emplaceNode(
      DsgLayers::ROOMS, NodeSymbol('R', 4), std::make_unique<SemanticNodeAttributes>());
  dsg->graph->emplaceNode(
      DsgLayers::ROOMS, NodeSymbol('R', 5), std::make_unique<SemanticNodeAttributes>());
  dsg->graph->emplaceNode(
      DsgLayers::ROOMS, NodeSymbol('R', 6), std::make_unique<SemanticNodeAttributes>());

  // two rooms without cluster associations: one at min size, one not
  dsg->graph->insertEdge(NodeSymbol('R', 5), NodeSymbol('p', 14));
  dsg->graph->insertEdge(NodeSymbol('R', 5), NodeSymbol('p', 15));
  dsg->graph->insertEdge(NodeSymbol('R', 5), NodeSymbol('p', 10));
  dsg->graph->insertEdge(NodeSymbol('R', 4), NodeSymbol('p', 16));

  // two rooms with cluster associations: both under min size
  dsg->graph->insertEdge(NodeSymbol('R', 3), NodeSymbol('p', 13));
  dsg->graph->insertEdge(NodeSymbol('R', 6), NodeSymbol('p', 12));

  RoomFinder::Config config;
  config.min_room_size = 3;
  TestableRoomFinder finder(config);

  ClusterResults results;
  results.clusters[0] = {NodeSymbol('p', 0), NodeSymbol('p', 1), NodeSymbol('p', 2)};
  results.clusters[1] = {NodeSymbol('p', 3), NodeSymbol('p', 4)};
  results.clusters[2] = {NodeSymbol('p', 5), NodeSymbol('p', 6)};
  results.clusters[3] = {NodeSymbol('p', 8), NodeSymbol('p', 9), NodeSymbol('p', 11)};

  RoomMap previous_rooms{
      {NodeSymbol('R', 3), {NodeSymbol('p', 3), NodeSymbol('p', 4)}},
      {NodeSymbol('R', 4), {}},
      {NodeSymbol('R', 5), {}},
      {NodeSymbol('R', 6), {NodeSymbol('p', 5), NodeSymbol('p', 6)}}};

  finder.updateRoomsFromClusters(*dsg, results, previous_rooms, {});

  const auto& rooms = dsg->graph->getLayer(DsgLayers::ROOMS);
  EXPECT_EQ(3u, rooms.numNodes());
  EXPECT_TRUE(rooms.hasNode(NodeSymbol('R', 0)));
  EXPECT_TRUE(rooms.hasNode(NodeSymbol('R', 5)));
  EXPECT_TRUE(rooms.hasNode(NodeSymbol('R', 6)));
  EXPECT_EQ(0u, rooms.numEdges());

  const SceneGraphNode& r0_node = dsg->graph->getNode(NodeSymbol('R', 0)).value();
  std::set<NodeId> r0_children{
      NodeSymbol('p', 8), NodeSymbol('p', 9), NodeSymbol('p', 11)};
  EXPECT_EQ(r0_children, r0_node.children())
      << displayNodeSymbolContainer(r0_node.children());

  const SceneGraphNode& r5_node = dsg->graph->getNode(NodeSymbol('R', 5)).value();
  std::set<NodeId> r5_children{
      NodeSymbol('p', 14), NodeSymbol('p', 15), NodeSymbol('p', 10)};
  EXPECT_EQ(r5_children, r5_node.children())
      << displayNodeSymbolContainer(r5_node.children());

  const SceneGraphNode& r6_node = dsg->graph->getNode(NodeSymbol('R', 6)).value();
  std::set<NodeId> r6_children{
      NodeSymbol('p', 12), NodeSymbol('p', 5), NodeSymbol('p', 6)};
  EXPECT_EQ(r6_children, r6_node.children())
      << displayNodeSymbolContainer(r6_node.children());
}

TEST(IncrementalRoomsTests, SpectralClusteringCorrect) {
  IsolatedSceneGraphLayer layer(1);
  for (size_t i = 0; i < 10; ++i) {
    layer.emplaceNode(i, std::make_unique<NodeAttributes>());
  }

  // first clique (3 nodes)
  layer.insertEdge(0, 1, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(0, 2, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(1, 2, std::make_unique<EdgeAttributes>(1.0));
  // bridge between cliques (biased so 3 is in first cluster)
  layer.insertEdge(2, 3, std::make_unique<EdgeAttributes>(0.1));
  layer.insertEdge(3, 4, std::make_unique<EdgeAttributes>(0.01));
  // second clique (5 nodes)
  layer.insertEdge(4, 5, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(4, 6, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(4, 7, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(4, 8, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(5, 6, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(5, 7, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(5, 8, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(6, 7, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(6, 8, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(7, 8, std::make_unique<EdgeAttributes>(1.0));
  // extra connection to second clique
  layer.insertEdge(6, 9, std::make_unique<EdgeAttributes>(0.3));

  Components components{{1}, {4, 6}};  // seed components with both cliques
  auto cluster_results = clusterGraph(layer, components);
  const auto& labels = cluster_results.labels;
  for (size_t i = 0; i < 10; ++i) {
    ASSERT_TRUE(labels.count(i));
  }

  // expect that first clique is in one cluster
  EXPECT_EQ(0u, labels.at(0));
  EXPECT_EQ(0u, labels.at(1));
  EXPECT_EQ(0u, labels.at(2));
  // expect that bridge node is in first cluster (because of edge weight difference)
  EXPECT_EQ(0u, labels.at(3));
  // expect that second clique is in once cluster
  EXPECT_EQ(1u, labels.at(4));
  EXPECT_EQ(1u, labels.at(5));
  EXPECT_EQ(1u, labels.at(5));
  EXPECT_EQ(1u, labels.at(7));
  EXPECT_EQ(1u, labels.at(8));
  // expect that extra node is connected to second clique
  EXPECT_EQ(1u, labels.at(9));

  ASSERT_EQ(2u, cluster_results.clusters.size());
  ASSERT_TRUE(cluster_results.clusters.count(0));
  ASSERT_TRUE(cluster_results.clusters.count(1));
  std::unordered_set<NodeId> first_cluster{0, 1, 2, 3};
  std::unordered_set<NodeId> second_cluster{4, 5, 6, 7, 8, 9};
  EXPECT_EQ(first_cluster, cluster_results.clusters.at(0));
  EXPECT_EQ(second_cluster, cluster_results.clusters.at(1));
}

TEST(IncrementalRoomsTests, ClusteringSparseDenseCorrecet) {
  IsolatedSceneGraphLayer layer(1);
  for (size_t i = 0; i < 10; ++i) {
    layer.emplaceNode(i, std::make_unique<NodeAttributes>());
  }

  // first clique (3 nodes)
  layer.insertEdge(0, 1, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(0, 2, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(1, 2, std::make_unique<EdgeAttributes>(1.0));
  // bridge between cliques (biased so 3 is in first cluster)
  layer.insertEdge(2, 3, std::make_unique<EdgeAttributes>(0.1));
  layer.insertEdge(3, 4, std::make_unique<EdgeAttributes>(0.01));
  // second clique (5 nodes)
  layer.insertEdge(4, 5, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(4, 6, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(4, 7, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(4, 8, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(5, 6, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(5, 7, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(5, 8, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(6, 7, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(6, 8, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(7, 8, std::make_unique<EdgeAttributes>(1.0));
  // extra connection to second clique
  layer.insertEdge(6, 9, std::make_unique<EdgeAttributes>(0.3));

  Components components{{1}, {4, 6}};  // seed components with both cliques
  auto dense_results = clusterGraph(layer, components, 5, false);
  auto sparse_results = clusterGraph(layer, components, 5, false);

  EXPECT_EQ(dense_results.total_iters, sparse_results.total_iters);
  EXPECT_EQ(dense_results.labels, sparse_results.labels);
  EXPECT_EQ(dense_results.clusters, sparse_results.clusters);
}

TEST(IncrementalRoomsTests, ModularityClusteringCorrect) {
  IsolatedSceneGraphLayer layer(1);
  for (size_t i = 0; i < 10; ++i) {
    layer.emplaceNode(i, std::make_unique<NodeAttributes>());
  }

  // first clique (3 nodes)
  layer.insertEdge(0, 1, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(0, 2, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(1, 2, std::make_unique<EdgeAttributes>(1.0));
  // bridge between cliques (biased so 3 is in first cluster)
  layer.insertEdge(2, 3, std::make_unique<EdgeAttributes>(0.1));
  layer.insertEdge(3, 4, std::make_unique<EdgeAttributes>(0.01));
  // second clique (5 nodes)
  layer.insertEdge(4, 5, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(4, 6, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(4, 7, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(4, 8, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(5, 6, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(5, 7, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(5, 8, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(6, 7, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(6, 8, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(7, 8, std::make_unique<EdgeAttributes>(1.0));
  // extra connection to second clique
  layer.insertEdge(6, 9, std::make_unique<EdgeAttributes>(0.3));

  Components components{{1}, {4, 6, 8}};  // seed components with both cliques
  auto cluster_results = clusterGraphByModularity(layer, components, 4);
  const auto& labels = cluster_results.labels;
  for (size_t i = 0; i < 10; ++i) {
    ASSERT_TRUE(labels.count(i));
  }

  // expect that first clique is in one cluster
  EXPECT_EQ(0u, labels.at(0));
  EXPECT_EQ(0u, labels.at(1));
  EXPECT_EQ(0u, labels.at(2));
  // expect that bridge node is in first cluster (because of edge weight difference)
  EXPECT_EQ(0u, labels.at(3));
  // expect that second clique is in once cluster
  EXPECT_EQ(1u, labels.at(4));
  EXPECT_EQ(1u, labels.at(5));
  EXPECT_EQ(1u, labels.at(5));
  EXPECT_EQ(1u, labels.at(7));
  EXPECT_EQ(1u, labels.at(8));
  // expect that extra node is connected to second clique
  EXPECT_EQ(1u, labels.at(9));

  ASSERT_EQ(2u, cluster_results.clusters.size());
  ASSERT_TRUE(cluster_results.clusters.count(0));
  ASSERT_TRUE(cluster_results.clusters.count(1));
  std::unordered_set<NodeId> first_cluster{0, 1, 2, 3};
  std::unordered_set<NodeId> second_cluster{4, 5, 6, 7, 8, 9};
  EXPECT_EQ(first_cluster, cluster_results.clusters.at(0));
  EXPECT_EQ(second_cluster, cluster_results.clusters.at(1));
}

TEST(IncrementalRoomsTests, TestLongestSequence) {
  {  // empty values -> no best index
    std::vector<size_t> values;
    EXPECT_FALSE(getLongestSequence(values));
  }

  {  // first value is 0 -> no best index
    std::vector<size_t> values{0};
    EXPECT_FALSE(getLongestSequence(values));
  }

  {  // all values are 0 -> no best index
    std::vector<size_t> values{0, 0, 0, 0, 0};
    EXPECT_FALSE(getLongestSequence(values));
  }

  {  // single value -> 0
    std::vector<size_t> values{1};
    auto result = getLongestSequence(values);
    ASSERT_TRUE(result);
    EXPECT_EQ(0u, *result);
  }

  {  // all one non-zero sequence
    std::vector<size_t> values{1, 1, 1, 1};
    auto result = getLongestSequence(values);
    ASSERT_TRUE(result);
    EXPECT_EQ(0u, *result);
  }

  {  // two sequences
    std::vector<size_t> values{1, 1, 2, 2, 2};
    auto result = getLongestSequence(values);
    ASSERT_TRUE(result);
    EXPECT_EQ(2u, *result);
  }

  {  // arbitrary set of values
    std::vector<size_t> values{1, 4, 2, 3, 2, 2, 5, 6, 6, 6, 9, 1, 1, 1, 2};
    auto result = getLongestSequence(values);
    ASSERT_TRUE(result);
    EXPECT_EQ(7u, *result);
  }
}

TEST(IncrementalRoomsTests, TestMedianComponent) {
  {  // empty values -> no best index
    std::vector<size_t> values;
    EXPECT_FALSE(getMedianComponentSize(values));
  }

  {  // first value is 0 -> no best index
    std::vector<size_t> values{0};
    EXPECT_FALSE(getMedianComponentSize(values));
  }

  {  // all values are 0 -> no best index
    std::vector<size_t> values{0, 0, 0, 0, 0};
    EXPECT_FALSE(getMedianComponentSize(values));
  }

  {  // single value -> 0
    std::vector<size_t> values{1};
    auto result = getMedianComponentSize(values);
    ASSERT_TRUE(result);
    EXPECT_EQ(0u, *result);
  }

  {  // all one non-zero sequence
    std::vector<size_t> values{1, 1, 1, 1};
    auto result = getMedianComponentSize(values);
    ASSERT_TRUE(result);
    EXPECT_EQ(0u, *result);
  }

  {  // two sequences
    std::vector<size_t> values{1, 1, 2, 2, 2};
    auto result = getMedianComponentSize(values);
    ASSERT_TRUE(result);
    EXPECT_EQ(2u, *result);
  }

  {  // arbitrary set of values
    std::vector<size_t> values{1, 4, 2, 3, 2, 2, 5, 6, 6, 6, 9, 1, 1, 1, 2};
    // sorted: 1, 2, 2, 2, 3, 4, 5, 6, 6, 6, 9
    auto result = getMedianComponentSize(values);
    ASSERT_TRUE(result);
    EXPECT_EQ(1u, *result);
  }
}

}  // namespace incremental
}  // namespace hydra
