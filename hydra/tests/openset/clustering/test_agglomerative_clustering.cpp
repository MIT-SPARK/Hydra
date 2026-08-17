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

static const auto fmt = getDefaultFormat();

Eigen::VectorXf getOneHot(size_t i, size_t dim) {
  Eigen::VectorXf p = Eigen::VectorXf::Zero(dim);
  p(i) = 1.0;
  return p;
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

void fillEdges(const std::vector<EdgeKey>& keys, EdgeContainer::Edges& edges) {
  for (const auto& [k1, k2] : keys) {
    edges.emplace(std::piecewise_construct,
                  std::forward_as_tuple(k1, k2),
                  std::forward_as_tuple(k1, k2, nullptr));
  }
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
  std::vector<EdgeKey> keys{
      {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}, {6, 7}, {7, 8}, {8, 9}};
  fillEdges(keys, edges);

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

  keys = {{0, 2}, {2, 4}, {4, 6}, {6, 8}};
  fillEdges(keys, edges);

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
  std::vector<EdgeKey> keys{{0, 2}, {2, 4}, {4, 6}, {6, 8}};
  fillEdges(keys, edges);

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

  IBProbabilityConfig config;
  config.score_threshold = 1.0;
  // Test simplest case
  config.cumulative = false;
  config.null_task_preprune = false;
  config.top_k = 100;  // Large so that essentially disabled
  ws.setup(config, y_tasks, dist);

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

TEST(AgglomerativeClustering, SetupTopKCorrect) {
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

  IBProbabilityConfig config;
  config.score_threshold = 0.9;
  // Test top k (k = 1)
  config.cumulative = false;
  config.null_task_preprune = false;
  config.top_k = 1;  // Test single top k (one hot)
  ws.setup(config, y_tasks, dist);

  Eigen::MatrixXd expected_py_x(4, 5);
  expected_py_x << 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
  EXPECT_TRUE(ws.py_x.isApprox(expected_py_x, 1e-9))
      << "expected: " << expected_py_x.format(fmt)
      << ", result: " << ws.py_x.format(fmt);
}

TEST(AgglomerativeClustering, SetupCumulativeCorrect) {
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

  IBProbabilityConfig config;
  config.score_threshold = 0.9;
  config.cumulative = true;  // Test cumulative
  config.null_task_preprune = false;
  config.top_k = 2;
  ws.setup(config, y_tasks, dist);

  Eigen::MatrixXd expected_py_x(4, 5);
  expected_py_x << 0.9 / 2.9, 0.9 / 2.9, 0.9 / 2.9, 1.0, 1.0, 2.0 / 2.9, 0.0, 0.0, 0.0,
      0.0, 0.0, 2.0 / 2.9, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0 / 2.9, 0.0, 0.0;
  EXPECT_TRUE(ws.py_x.isApprox(expected_py_x, 1e-9))
      << "expected: " << expected_py_x.format(fmt)
      << ", result: " << ws.py_x.format(fmt);
}

TEST(AgglomerativeClustering, SetupNullPruneCorrect) {
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

  IBProbabilityConfig config;
  config.score_threshold = 0.9;
  config.cumulative = false;
  config.null_task_preprune = true;  // Test null preprune
  config.top_k = 100;
  ws.setup(config, y_tasks, dist);

  Eigen::MatrixXd expected_py_x(4, 5);
  expected_py_x << 0.9 / 1.9, 0.9 / 1.9, 0.9 / 1.9, 1.0, 1.0, 1.0 / 1.9, 0.0, 0.0, 0.0,
      0.0, 0.0, 1.0 / 1.9, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 / 1.9, 0.0, 0.0;
  EXPECT_TRUE(ws.py_x.isApprox(expected_py_x, 1e-9))
      << "expected: " << expected_py_x.format(fmt)
      << ", result: " << ws.py_x.format(fmt);
}

TEST(AgglomerativeClustering, UpdateCorrect) {
  NodeEmbeddingMap x_segments;
  for (size_t i = 0; i < 5; ++i) {
    x_segments[2 * i] = getOneHot(i, 10);
  }

  EmbeddingGroup y_tasks;
  for (size_t i = 0; i < 3; ++i) {
    y_tasks.embeddings.push_back(getOneHot(i, 10));
    y_tasks.names.push_back(std::to_string(i));
  }

  EdgeContainer::Edges edges;
  std::vector<EdgeKey> keys{{0, 2}, {2, 4}, {4, 6}, {6, 8}};
  fillEdges(keys, edges);

  CosineDistance dist;
  ClusteringWorkspace ws(edges, x_segments);

  IBProbabilityConfig config;
  config.score_threshold = 1.0;
  config.cumulative = false;  // Test simplest case
  config.null_task_preprune = false;
  config.top_k = 100;  // Large so that essentially disabled
  ws.setup(config, y_tasks, dist);

  std::list<EdgeKey> updated;
  ws.merge(EdgeKey(0, 1), -std::numeric_limits<double>::infinity(), updated);

  // check that hard-cluster assumptions hold
  EXPECT_EQ(ws.pz(1), 0.0);
  EXPECT_EQ(ws.pz(0), 0.4);
  EXPECT_EQ(ws.pz_x(1, 1), 0.0);
  EXPECT_EQ(ws.pz_x(0, 0), 1.0);
  EXPECT_EQ(ws.pz_x(1, 0), 1.0) << "p(z|x): " << ws.pz_x.format(fmt);

  Eigen::VectorXd py_0(4);
  py_0 << 0.5, 0.25, 0.25, 0.0;
  EXPECT_TRUE(ws.py_z.col(0).isApprox(py_0, 1e-9))
      << "p(y|z=0): " << ws.py_z.col(0).format(fmt);
  Eigen::VectorXd py_1(4);
  py_1 << 0.0, 0.0, 0.0, 0.0;
  EXPECT_TRUE(ws.py_z.col(1).isApprox(py_1))
      << "p(y|z=1): " << ws.py_z.col(1).format(fmt);
}

TEST(AgglomerativeClustering, GetClustersCorrect) {
  NodeEmbeddingMap map;
  for (size_t i = 0; i < 5; ++i) {
    map[2 * i] = getOneHot(i, 10);
  }

  EdgeContainer::Edges edges;
  std::vector<EdgeKey> keys{{0, 2}, {2, 4}, {4, 6}, {6, 8}};
  fillEdges(keys, edges);

  ClusteringWorkspace ws(edges, map);

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
