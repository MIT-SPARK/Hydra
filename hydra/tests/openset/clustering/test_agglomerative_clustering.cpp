#include <config_utilities/config.h>
#include <gtest/gtest.h>
#include <hydra/openset/clustering/agglomerative_clustering.h>
#include <spark_dsg/edge_attributes.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/printing.h>

namespace hydra {

using namespace spark_dsg;
using Cluster = AgglomerativeClustering::Cluster;
using ClusterIds = std::vector<std::vector<NodeId>>;
using ClusteringWorkspace = AgglomerativeClustering::Workspace;
using NodeEmbeddingMap = AgglomerativeClustering::NodeEmbeddingMap;

namespace {

inline Eigen::VectorXd getOneHot(size_t i, size_t dim) {
  Eigen::VectorXd p = Eigen::VectorXd::Zero(dim);
  p(i) = 1.0;
  return p;
}

struct FakeEmbeddingGroup : public EmbeddingGroup {
  struct Config {};

  explicit FakeEmbeddingGroup(const Config&) {
    Eigen::VectorXd ref(10);
    ref << 1.0, 2.0, 3.0, 4.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    embeddings.push_back(ref);
    names.push_back("name_0");
  }

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<EmbeddingGroup, FakeEmbeddingGroup, Config>(
          "fake_embeddings");
};

[[maybe_unused]] void declare_config(FakeEmbeddingGroup::Config&) {}

inline AgglomerativeClustering::Config getFakeConfig() {
  using namespace config;
  AgglomerativeClustering::Config conf;
  conf.metric = CosineDistance::Config{};
  conf.tasks = FakeEmbeddingGroup::Config{};
  return conf;
}

std::map<NodeId, size_t> getNodeAssignments(const ClusterIds& clusters) {
  std::map<NodeId, size_t> assignments;
  for (size_t i = 0; i < clusters.size(); ++i) {
    for (const auto& node_id : clusters[i]) {
      assignments[node_id] = i;
    }
  }

  return assignments;
}

template <typename K, typename V>
std::string printMap(const std::map<K, V>& values) {
  std::stringstream ss;
  ss << "{";

  auto iter = values.begin();
  while (iter != values.end()) {
    ss << iter->first << ": " << iter->second;
    ++iter;
    if (iter != values.end()) {
      ss << ", ";
    }
  }

  ss << "}";
  return ss.str();
}

template <typename T>
std::string printVec(const std::vector<T>& values) {
  std::stringstream ss;
  ss << "[";

  auto iter = values.begin();
  while (iter != values.end()) {
    ss << *iter;
    ++iter;
    if (iter != values.end()) {
      ss << ", ";
    }
  }

  ss << "]";
  return ss.str();
}

std::string workspaceState(const AgglomerativeClustering::Workspace& ws) {
  std::stringstream ss;
  ss << "lookup: " << printMap(ws.node_lookup) << std::endl;
  ss << "order: " << printMap(ws.order) << std::endl;
  ss << "assignments: " << printVec(ws.assignments) << std::endl;
  ss << "edges: " << printMap(ws.edges) << std::endl;
  return ss.str();
}

}  // namespace

TEST(ClusteringWorkspace, InitCorrect) {
  NodeEmbeddingMap map;
  for (size_t i = 0; i < 5; ++i) {
    map[2 * i] = getOneHot(i, 10);
  }

  {  // nodes only: only should get features, no edges
    ClusteringWorkspace ws(EdgeContainer::Edges{}, map);
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

  EdgeContainer::Edges edges;
  for (size_t i = 0; i < 9; ++i) {
    const EdgeKey key{i, i + 1};
    edges.emplace(key, SceneGraphEdge{key.k1, key.k2, nullptr});
  }

  {  // no siblings in map: only should get features, no edges
    ClusteringWorkspace ws(edges, map);
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
    const EdgeKey key{2 * i, 2 * (i + 1)};
    edges.emplace(key, SceneGraphEdge{key.k1, key.k2, nullptr});
  }

  {  // linear siblings: should get edges
    ClusteringWorkspace ws(edges, map);
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
  NodeEmbeddingMap map;
  for (size_t i = 0; i < 5; ++i) {
    map[2 * i] = getOneHot(i, 10);
  }

  EdgeContainer::Edges edges;
  for (size_t i = 0; i < 4; ++i) {
    const EdgeKey key{2 * i, 2 * (i + 1)};
    edges.emplace(key, SceneGraphEdge{key.k1, key.k2, nullptr});
  }

  ClusteringWorkspace ws(edges, map);

  std::list<EdgeKey> updated_edges;
  ws.merge({1, 2}, -std::numeric_limits<double>::infinity(), updated_edges);
  std::list<EdgeKey> expected_updates{{0, 1}, {1, 3}};
  EXPECT_EQ(updated_edges, expected_updates);
  EXPECT_EQ(ws.size(), 5);
  EXPECT_EQ(ws.featureDim(), 10);
  std::map<EdgeKey, double> expected_edges{{{0, 1}, 0.0}, {{1, 3}, 0.0}, {{3, 4}, 0.0}};
  EXPECT_EQ(ws.edges, expected_edges);
  std::vector<size_t> expected_assignments{0, 1, 1, 3, 4};
  EXPECT_EQ(ws.assignments, expected_assignments);

  // note: edge keys are ordered
  ws.merge({4, 3}, -std::numeric_limits<double>::infinity(), updated_edges);
  expected_updates = {{1, 3}};
  EXPECT_EQ(updated_edges, expected_updates);
  EXPECT_EQ(ws.size(), 5);
  EXPECT_EQ(ws.featureDim(), 10);
  expected_edges = {{{0, 1}, 0.0}, {{1, 3}, 0.0}};
  EXPECT_EQ(ws.edges, expected_edges);
  expected_assignments = {0, 1, 1, 3, 3};
  EXPECT_EQ(ws.assignments, expected_assignments);

  ws.merge({0, 1}, -std::numeric_limits<double>::infinity(), updated_edges);
  expected_updates = {{0, 3}};
  EXPECT_EQ(updated_edges, expected_updates);
  EXPECT_EQ(ws.size(), 5);
  EXPECT_EQ(ws.featureDim(), 10);
  expected_edges = {{{0, 3}, 0.0}};
  EXPECT_EQ(ws.edges, expected_edges);
  expected_assignments = {0, 0, 0, 3, 3};
  EXPECT_EQ(ws.assignments, expected_assignments);

  // note: edge keys are ordered
  ws.merge({3, 0}, -std::numeric_limits<double>::infinity(), updated_edges);
  EXPECT_TRUE(updated_edges.empty());
  EXPECT_EQ(ws.size(), 5);
  EXPECT_EQ(ws.featureDim(), 10);
  expected_edges = {};
  EXPECT_EQ(ws.edges, expected_edges);
  expected_assignments = {0, 0, 0, 0, 0};
  EXPECT_EQ(ws.assignments, expected_assignments);
}

TEST(AgglomerativeClustering, GetClustersCorrect) {
  NodeEmbeddingMap map;
  for (size_t i = 0; i < 5; ++i) {
    map[2 * i] = getOneHot(i, 10);
  }

  for (size_t i = 0; i < 5; ++i) {
  }

  EdgeContainer::Edges edges;
  for (size_t i = 0; i < 4; ++i) {
    const EdgeKey key{2 * i, 2 * (i + 1)};
    edges.emplace(key, SceneGraphEdge{key.k1, key.k2, nullptr});
  }

  const auto config = getFakeConfig();
  ClusteringWorkspace ws(edges, map);
  AgglomerativeClustering clustering(config);

  {  // expected_assignments = {0, 1, 2, 3, 4}

    auto clusters = ws.getClusters();
    EXPECT_EQ(clusters.size(), 5);
    const auto result = getNodeAssignments(clusters);
    std::map<NodeId, size_t> expected{{0, 0}, {2, 1}, {4, 2}, {6, 3}, {8, 4}};
    EXPECT_EQ(expected, result) << workspaceState(ws);
  }

  std::list<EdgeKey> updated;
  ws.merge({1, 2}, -std::numeric_limits<double>::infinity(), updated);
  {  // expected_assignments = {0, 1, 1, 3, 4}
    auto clusters = ws.getClusters();
    EXPECT_EQ(clusters.size(), 4);
    const auto result = getNodeAssignments(clusters);
    std::map<NodeId, size_t> expected{{0, 0}, {2, 1}, {4, 1}, {6, 2}, {8, 3}};
    EXPECT_EQ(expected, result) << workspaceState(ws);
  }

  ws.merge({3, 4}, -std::numeric_limits<double>::infinity(), updated);
  {  // expected_assignments = {0, 1, 1, 3, 3}
    auto clusters = ws.getClusters();
    EXPECT_EQ(clusters.size(), 3);
    const auto result = getNodeAssignments(clusters);
    std::map<NodeId, size_t> expected{{0, 0}, {2, 1}, {4, 1}, {6, 2}, {8, 2}};
    EXPECT_EQ(expected, result) << workspaceState(ws);
  }

  ws.merge({0, 1}, -std::numeric_limits<double>::infinity(), updated);
  {  // expected_assignments = {0, 0, 0, 3, 3}
    auto clusters = ws.getClusters();
    EXPECT_EQ(clusters.size(), 2);
    const auto result = getNodeAssignments(clusters);
    std::map<NodeId, size_t> expected{{0, 0}, {2, 0}, {4, 0}, {6, 1}, {8, 1}};
    EXPECT_EQ(expected, result) << workspaceState(ws);
  }

  ws.merge({0, 3}, -std::numeric_limits<double>::infinity(), updated);
  {  // expected_assignments = {0, 0, 0, 0, 0}
    auto clusters = ws.getClusters();
    EXPECT_EQ(clusters.size(), 1);
    const auto result = getNodeAssignments(clusters);
    std::map<NodeId, size_t> expected{{0, 0}, {2, 0}, {4, 0}, {6, 0}, {8, 0}};
    EXPECT_EQ(expected, result) << workspaceState(ws);
  }
}

}  // namespace hydra
