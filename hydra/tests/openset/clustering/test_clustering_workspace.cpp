#include <clio/clustering_workspace.h>
#include <clio/common.h>
#include <gtest/gtest.h>

namespace clio {

namespace {

inline Eigen::VectorXd getOneHot(size_t i, size_t dim) {
  Eigen::VectorXd p = Eigen::VectorXd::Zero(dim);
  p(i) = 1.0;
  return p;
}

}  // namespace

TEST(ClusteringWorkspace, InitCorrect) {
  IsolatedSceneGraphLayer layer(2);

  NodeEmbeddingMap map;
  for (size_t i = 0; i < 5; ++i) {
    layer.emplaceNode(2 * i, std::make_unique<NodeAttributes>());
    map[2 * i] = getOneHot(i, 10);
  }

  for (size_t i = 0; i < 5; ++i) {
    layer.emplaceNode(2 * i + 1, std::make_unique<NodeAttributes>());
  }

  {  // nodes only: only should get features, no edges
    ClusteringWorkspace ws(layer, map);
    EXPECT_EQ(ws.size(), 5);
    EXPECT_EQ(ws.featureDim(), 10);
    EXPECT_TRUE(ws.edges.empty());

    std::map<size_t, NodeId> expected_lookup{{0, 0}, {1, 2}, {2, 4}, {3, 6}, {4, 8}};
    EXPECT_EQ(ws.node_lookup, expected_lookup);
    std::map<NodeId, size_t> expected_order{{0, 0}, {2, 1}, {4, 2}, {6, 3}, {8, 4}};
    EXPECT_EQ(ws.order, expected_order);
    std::vector<size_t> expected_assignments{0, 1, 2, 3, 4};
    EXPECT_EQ(ws.assignments, expected_assignments);
  }

  for (size_t i = 0; i < 9; ++i) {
    layer.insertEdge(i, i + 1);
  }

  {  // no siblings in map: only should get features, no edges
    ClusteringWorkspace ws(layer, map);
    EXPECT_EQ(ws.size(), 5);
    EXPECT_EQ(ws.featureDim(), 10);
    EXPECT_TRUE(ws.edges.empty());

    std::map<size_t, NodeId> expected_lookup{{0, 0}, {1, 2}, {2, 4}, {3, 6}, {4, 8}};
    EXPECT_EQ(ws.node_lookup, expected_lookup);
    std::map<NodeId, size_t> expected_order{{0, 0}, {2, 1}, {4, 2}, {6, 3}, {8, 4}};
    EXPECT_EQ(ws.order, expected_order);
    std::vector<size_t> expected_assignments{0, 1, 2, 3, 4};
    EXPECT_EQ(ws.assignments, expected_assignments);
  }

  for (size_t i = 0; i < 4; ++i) {
    layer.insertEdge(2 * i, 2 * (i + 1));
  }

  {  // linear siblings: should get edges
    ClusteringWorkspace ws(layer, map);
    EXPECT_EQ(ws.size(), 5);
    EXPECT_EQ(ws.featureDim(), 10);
    // edges are keyed by index in workspace
    std::map<EdgeKey, double> expected_edges{
        {{0, 1}, 0.0}, {{1, 2}, 0.0}, {{2, 3}, 0.0}, {{3, 4}, 0.0}};
    EXPECT_EQ(ws.edges, expected_edges);

    std::map<size_t, NodeId> expected_lookup{{0, 0}, {1, 2}, {2, 4}, {3, 6}, {4, 8}};
    EXPECT_EQ(ws.node_lookup, expected_lookup);
    std::map<NodeId, size_t> expected_order{{0, 0}, {2, 1}, {4, 2}, {6, 3}, {8, 4}};
    EXPECT_EQ(ws.order, expected_order);
    std::vector<size_t> expected_assignments{0, 1, 2, 3, 4};
    EXPECT_EQ(ws.assignments, expected_assignments);
  }
}

TEST(ClusteringWorkspace, MergeCorrect) {
  IsolatedSceneGraphLayer layer(2);

  NodeEmbeddingMap map;
  for (size_t i = 0; i < 5; ++i) {
    layer.emplaceNode(2 * i, std::make_unique<NodeAttributes>());
    map[2 * i] = getOneHot(i, 10);
  }

  for (size_t i = 0; i < 5; ++i) {
    layer.emplaceNode(2 * i + 1, std::make_unique<NodeAttributes>());
  }

  for (size_t i = 0; i < 4; ++i) {
    layer.insertEdge(2 * i, 2 * (i + 1));
  }

  ClusteringWorkspace ws(layer, map);

  auto updated_edges = ws.addMerge({1, 2});
  std::list<EdgeKey> expected_updates{{0, 1}, {1, 3}};
  EXPECT_EQ(updated_edges, expected_updates);
  EXPECT_EQ(ws.size(), 5);
  EXPECT_EQ(ws.featureDim(), 10);
  std::map<EdgeKey, double> expected_edges{{{0, 1}, 0.0}, {{1, 3}, 0.0}, {{3, 4}, 0.0}};
  EXPECT_EQ(ws.edges, expected_edges);
  std::vector<size_t> expected_assignments{0, 1, 1, 3, 4};
  EXPECT_EQ(ws.assignments, expected_assignments);

  // note: edge keys are ordered
  updated_edges = ws.addMerge({4, 3});
  expected_updates = {{1, 3}};
  EXPECT_EQ(updated_edges, expected_updates);
  EXPECT_EQ(ws.size(), 5);
  EXPECT_EQ(ws.featureDim(), 10);
  expected_edges = {{{0, 1}, 0.0}, {{1, 3}, 0.0}};
  EXPECT_EQ(ws.edges, expected_edges);
  expected_assignments = {0, 1, 1, 3, 3};
  EXPECT_EQ(ws.assignments, expected_assignments);

  updated_edges = ws.addMerge({0, 1});
  expected_updates = {{0, 3}};
  EXPECT_EQ(updated_edges, expected_updates);
  EXPECT_EQ(ws.size(), 5);
  EXPECT_EQ(ws.featureDim(), 10);
  expected_edges = {{{0, 3}, 0.0}};
  EXPECT_EQ(ws.edges, expected_edges);
  expected_assignments = {0, 0, 0, 3, 3};
  EXPECT_EQ(ws.assignments, expected_assignments);

  // note: edge keys are ordered
  updated_edges = ws.addMerge({3, 0});
  EXPECT_TRUE(updated_edges.empty());
  EXPECT_EQ(ws.size(), 5);
  EXPECT_EQ(ws.featureDim(), 10);
  expected_edges = {};
  EXPECT_EQ(ws.edges, expected_edges);
  expected_assignments = {0, 0, 0, 0, 0};
  EXPECT_EQ(ws.assignments, expected_assignments);
}

}  // namespace clio
