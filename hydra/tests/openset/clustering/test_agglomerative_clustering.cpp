#include <config_utilities/config.h>
#include <gtest/gtest.h>
#include <hydra/openset/clustering/agglomerative_clustering.h>
#include <hydra/utils/printing.h>
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

TEST(AgglomerativeClustering, WorkspaceInitCorrect) {
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

TEST(AgglomerativeClustering, WorkspaceMergeCorrect) {
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

TEST(AgglomerativeClustering, SetupSimpleCorrect) {
  NodeEmbeddingMap x_segments;
  for (size_t i = 0; i < 5; ++i) {
    x_segments[2 * i] = getOneHot(i, 10);
  }

  EmbeddingGroup y_tasks;
  for (size_t i = 0; i < 3; ++i) {
    y_tasks.embeddings.push_back(getOneHot(i, 10));
    y_tasks.names.push_back(std::to_string(i));
  }

  CosineDistance dist;
  ClusteringWorkspace ws(EdgeContainer::Edges{}, x_segments);

  IBEdgeSelector::Config config;
  config.py_x.score_threshold = 1.0;
  // Test simplest case
  config.py_x.cumulative = false;
  config.py_x.null_task_preprune = false;
  config.py_x.top_k = 100;  // Large so that essentially disabled
  ws.setup(y_tasks, dist);

  ASSERT_EQ(ws.px.rows(), 5);
  ASSERT_EQ(ws.pz.rows(), 5);
  ASSERT_EQ(ws.py.rows(), 4);
  ASSERT_EQ(ws.pz_x.rows(), 5);
  ASSERT_EQ(ws.pz_x.cols(), 5);
  ASSERT_EQ(ws.py_x.rows(), 4);
  ASSERT_EQ(ws.py_x.cols(), 5);
  ASSERT_EQ(ws.py_z.rows(), 4);
  ASSERT_EQ(ws.py_z.cols(), 5);
  EXPECT_TRUE(ws.px.isApprox(ws.pz));
  EXPECT_TRUE(ws.py_x.isApprox(ws.py_z));

  const auto fmt = getDefaultFormat();
  Eigen::VectorXd expected_px(5);
  expected_px << 0.2, 0.2, 0.2, 0.2, 0.2;  // row-wise sum normalized via l1-norm
  EXPECT_TRUE(ws.px.isApprox(expected_px))
      << "expected: " << expected_px.format(fmt) << ", result: " << ws.px.format(fmt);

  Eigen::MatrixXd expected_py_x(4, 5);
  expected_py_x << 0.5, 0.5, 0.5, 1.0, 1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.5, 0.0, 0.0;
  EXPECT_TRUE(ws.py_x.isApprox(expected_py_x, 1e-9))
      << "expected: " << expected_py_x.format(fmt)
      << ", result: " << ws.py_x.format(fmt);

  Eigen::VectorXd expected_py(4);
  expected_py << 0.25, 0.25, 0.25, 0.25;  // row-wise sum normalized via l1-norm
  EXPECT_TRUE(ws.py.isApprox(expected_py))
      << "expected: " << expected_py.format(fmt) << ", result: " << ws.py.format(fmt);
}

TEST(IBEdgeSelector, SetupTopKCorrect) {
  NodeEmbeddingMap x_segments;
  for (size_t i = 0; i < 5; ++i) {
    x_segments[2 * i] = getOneHot(i, 10);
  }

  EmbeddingGroup y_tasks;
  for (size_t i = 0; i < 3; ++i) {
    y_tasks.embeddings.push_back(getOneHot(i, 10));
    y_tasks.names.push_back(std::to_string(i));
  }

  CosineDistance dist;
  ClusteringWorkspace ws(EdgeContainer::Edges{}, x_segments);

  IBEdgeSelector::Config config;
  config.py_x.score_threshold = 0.9;
  // Test top k (k = 1)
  config.py_x.cumulative = false;
  config.py_x.null_task_preprune = false;
  config.py_x.top_k = 1;  // Test single top k (one hot)
  ws.setup(y_tasks, dist);

  const auto fmt = getDefaultFormat();

  Eigen::MatrixXd expected_py_x(4, 5);
  expected_py_x << 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
  EXPECT_TRUE(ws.py_x.isApprox(expected_py_x, 1e-9))
      << "expected: " << expected_py_x.format(fmt)
      << ", result: " << ws.py_x.format(fmt);
}

TEST(IBEdgeSelector, SetupCumulativeCorrect) {
  NodeEmbeddingMap x_segments;
  for (size_t i = 0; i < 5; ++i) {
    x_segments[2 * i] = getOneHot(i, 10);
  }

  EmbeddingGroup y_tasks;
  for (size_t i = 0; i < 3; ++i) {
    y_tasks.embeddings.push_back(getOneHot(i, 10));
    y_tasks.names.push_back(std::to_string(i));
  }

  CosineDistance dist;
  ClusteringWorkspace ws(EdgeContainer::Edges{}, x_segments);

  IBEdgeSelector::Config config;
  config.py_x.score_threshold = 0.9;
  // Test cumulative
  config.py_x.cumulative = true;
  config.py_x.null_task_preprune = false;
  config.py_x.top_k = 2;
  ws.setup(y_tasks, dist);

  const auto fmt = getDefaultFormat();

  Eigen::MatrixXd expected_py_x(4, 5);
  expected_py_x << 0.9 / 2.9, 0.9 / 2.9, 0.9 / 2.9, 1.0, 1.0, 2.0 / 2.9, 0.0, 0.0, 0.0,
      0.0, 0.0, 2.0 / 2.9, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0 / 2.9, 0.0, 0.0;
  EXPECT_TRUE(ws.py_x.isApprox(expected_py_x, 1e-9))
      << "expected: " << expected_py_x.format(fmt)
      << ", result: " << ws.py_x.format(fmt);
}

TEST(IBEdgeSelector, SetupNullPruneCorrect) {
  NodeEmbeddingMap x_segments;
  for (size_t i = 0; i < 3; ++i) {
    x_segments[2 * i] = getOneHot(i, 10);
  }
  for (size_t i = 3; i < 5; ++i) {
    x_segments[2 * i] = getOneHot(i, 10) + getOneHot(0, 10);
  }

  EmbeddingGroup y_tasks;
  for (size_t i = 0; i < 3; ++i) {
    y_tasks.embeddings.push_back(getOneHot(i, 10));
    y_tasks.names.push_back(std::to_string(i));
  }

  CosineDistance dist;
  ClusteringWorkspace ws(EdgeContainer::Edges{}, x_segments);

  IBEdgeSelector::Config config;
  config.py_x.score_threshold = 0.9;
  // Test null preprune
  config.py_x.cumulative = false;
  config.py_x.null_task_preprune = true;
  config.py_x.top_k = 100;
  TestableIBEdgeSelector selector(config);
  selector.setup(ws, y_tasks, dist);

  const auto fmt = getDefaultFormat();

  Eigen::MatrixXd expected_py_x(4, 5);
  expected_py_x << 0.9 / 1.9, 0.9 / 1.9, 0.9 / 1.9, 1.0, 1.0, 1.0 / 1.9, 0.0, 0.0, 0.0,
      0.0, 0.0, 1.0 / 1.9, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 / 1.9, 0.0, 0.0;
  EXPECT_TRUE(selector.py_x_.isApprox(expected_py_x, 1e-9))
      << "expected: " << expected_py_x.format(fmt)
      << ", result: " << selector.py_x_.format(fmt);
}

TEST(IBEdgeSelector, UpdateCorrect) {
  IsolatedSceneGraphLayer layer(2);

  NodeEmbeddingMap x_segments;
  for (size_t i = 0; i < 5; ++i) {
    layer.emplaceNode(2 * i, std::make_unique<NodeAttributes>());
    x_segments[2 * i] = getOneHot(i, 10);
  }

  EmbeddingGroup y_tasks;
  for (size_t i = 0; i < 3; ++i) {
    y_tasks.embeddings.push_back(getOneHot(i, 10));
    y_tasks.tasks.push_back(std::to_string(i));
  }

  for (size_t i = 0; i < 5; ++i) {
    layer.emplaceNode(2 * i + 1, std::make_unique<NodeAttributes>());
  }

  for (size_t i = 0; i < 4; ++i) {
    layer.insertEdge(2 * i, 2 * (i + 1));
  }

  ClusteringWorkspace ws(layer, x_segments);
  CosineDistance dist;

  IBEdgeSelector::Config config;
  config.py_x.score_threshold = 1.0;
  // Test simplest case
  config.py_x.cumulative = false;
  config.py_x.null_task_preprune = false;
  config.py_x.top_k = 100;  // Large so that essentially disabled
  TestableIBEdgeSelector selector(config);
  selector.setup(ws, y_tasks, dist);
  selector.updateFromEdge(EdgeKey(0, 1));

  const auto fmt = getDefaultFormat();

  // check that hard-cluster assumptions hold
  EXPECT_EQ(selector.pz_(1), 0.0);
  EXPECT_EQ(selector.pz_(0), 0.4);
  EXPECT_EQ(selector.pz_x_(1, 1), 0.0);
  EXPECT_EQ(selector.pz_x_(0, 0), 1.0);
  EXPECT_EQ(selector.pz_x_(1, 0), 1.0) << "p(z|x): " << selector.pz_x_.format(fmt);

  Eigen::VectorXd py_0(4);
  py_0 << 0.5, 0.25, 0.25, 0.0;
  EXPECT_TRUE(selector.py_z_.col(0).isApprox(py_0, 1e-9))
      << "p(y|z=0): " << selector.py_z_.col(0).format(fmt);
  Eigen::VectorXd py_1(4);
  py_1 << 0.0, 0.0, 0.0, 0.0;
  EXPECT_TRUE(selector.py_z_.col(1).isApprox(py_1))
      << "p(y|z=1): " << selector.py_z_.col(1).format(fmt);
}

TEST(IBEdgeSelector, CompareEdgesCorrect) {
  IBEdgeSelector::Config config;
  TestableIBEdgeSelector selector(config);
  std::pair<EdgeKey, double> e1{{0, 1}, 0.0};
  std::pair<EdgeKey, double> e2{{0, 1}, 0.1};
  EXPECT_TRUE(selector.compareEdges(e1, e2));
  EXPECT_FALSE(selector.compareEdges(e1, e1));
  EXPECT_FALSE(selector.compareEdges(e2, e1));
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
