#include <config_utilities/parsing/yaml.h>
#include <gtest/gtest.h>
#include <hydra/openset/clustering/agglomerative_ib_clustering.h>
#include <hydra/utils/printing.h>
#include <spark_dsg/edge_attributes.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/printing.h>
#include <spark_dsg/scene_graph_layer.h>

namespace hydra {

using namespace spark_dsg;
using Cluster = AgglomerativeIBClustering::Cluster;
using ClusterIds = std::vector<std::vector<NodeId>>;
using ClusterConfig = AgglomerativeIBClustering::ClusteringConfig;
using ClusteringWorkspace = AgglomerativeIBClustering::Workspace;
using NodeEmbeddingMap = ClusteringWorkspace::NodeEmbeddings;

namespace {

static const auto fmt = getDefaultFormat(6);

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

std::string workspaceState(const AgglomerativeIBClustering::Workspace& ws) {
  std::stringstream ss;
  ss << "lookup: " << printMap(ws.node_lookup) << std::endl;
  ss << "order: " << printMap(ws.order) << std::endl;
  ss << "assignments: " << printVec(ws.assignments) << std::endl;
  ss << "edges: " << printMap(ws.edges) << std::endl;
  return ss.str();
}

ClusteringWorkspace makeWorkspace(const std::vector<EdgeKey>& keys,
                                  const NodeEmbeddingMap& map,
                                  const ClusterConfig& config = {}) {
  EmbeddingGroup y_tasks;
  for (size_t i = 0; i < 3; ++i) {
    y_tasks.embeddings.push_back(getOneHot(i, 10));
    y_tasks.names.push_back(std::to_string(i));
  }

  CosineDistance dist;
  return ClusteringWorkspace(config, keys, map, y_tasks, dist);
}

}  // namespace

TEST(AgglomerativeIBClustering, WorkspaceInitCorrect) {
  NodeEmbeddingMap map;
  for (size_t i = 0; i < 5; ++i) {
    map[2 * i] = getOneHot(i, 10);
  }

  {  // nodes only: only should get features, no edges
    auto ws = makeWorkspace({}, map);
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

  std::vector<EdgeKey> keys{
      {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}, {6, 7}, {7, 8}, {8, 9}};

  {  // no siblings in map: only should get features, no edges

    auto ws = makeWorkspace(keys, map);
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

  {  // linear siblings: should get edges
    auto ws = makeWorkspace(keys, map);
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

TEST(AgglomerativeIBClustering, WorkspaceMergeCorrect) {
  NodeEmbeddingMap map;
  for (size_t i = 0; i < 5; ++i) {
    map[2 * i] = getOneHot(i, 10);
  }

  std::vector<EdgeKey> keys{{0, 2}, {2, 4}, {4, 6}, {6, 8}};
  auto ws = makeWorkspace(keys, map);

  std::list<EdgeKey> updated_edges;
  ws.merge({1, 2}, updated_edges, true);
  std::list<EdgeKey> expected_updates{{0, 1}, {1, 3}};
  EXPECT_EQ(updated_edges, expected_updates);
  EXPECT_EQ(ws.size(), 5);
  EXPECT_EQ(ws.featureDim(), 10);
  std::map<EdgeKey, double> expected_edges{{{0, 1}, 0.0}, {{1, 3}, 0.0}, {{3, 4}, 0.0}};
  EXPECT_EQ(ws.edges, expected_edges);
  std::vector<size_t> expected_assignments{0, 1, 1, 3, 4};
  EXPECT_EQ(ws.assignments, expected_assignments);

  // note: edge keys are ordered
  ws.merge({4, 3}, updated_edges, true);
  expected_updates = {{1, 3}};
  EXPECT_EQ(updated_edges, expected_updates);
  EXPECT_EQ(ws.size(), 5);
  EXPECT_EQ(ws.featureDim(), 10);
  expected_edges = {{{0, 1}, 0.0}, {{1, 3}, 0.0}};
  EXPECT_EQ(ws.edges, expected_edges);
  expected_assignments = {0, 1, 1, 3, 3};
  EXPECT_EQ(ws.assignments, expected_assignments);

  ws.merge({0, 1}, updated_edges, true);
  expected_updates = {{0, 3}};
  EXPECT_EQ(updated_edges, expected_updates);
  EXPECT_EQ(ws.size(), 5);
  EXPECT_EQ(ws.featureDim(), 10);
  expected_edges = {{{0, 3}, 0.0}};
  EXPECT_EQ(ws.edges, expected_edges);
  expected_assignments = {0, 0, 0, 3, 3};
  EXPECT_EQ(ws.assignments, expected_assignments);

  // note: edge keys are ordered
  ws.merge({3, 0}, updated_edges, true);
  EXPECT_TRUE(updated_edges.empty());
  EXPECT_EQ(ws.size(), 5);
  EXPECT_EQ(ws.featureDim(), 10);
  expected_edges = {};
  EXPECT_EQ(ws.edges, expected_edges);
  expected_assignments = {0, 0, 0, 0, 0};
  EXPECT_EQ(ws.assignments, expected_assignments);
}

TEST(AgglomerativeIBClustering, SetupSimpleCorrect) {
  NodeEmbeddingMap x_segments;
  for (size_t i = 0; i < 5; ++i) {
    x_segments[2 * i] = getOneHot(i, 10);
  }

  ClusterConfig config;
  config.score_threshold = 1.0;
  config.cumulative = false;
  config.null_task_preprune = false;
  config.top_k = 100;  // Large so that essentially disabled
  auto ws = makeWorkspace({}, x_segments, config);

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

TEST(AgglomerativeIBClustering, SetupTopKCorrect) {
  NodeEmbeddingMap x_segments;
  for (size_t i = 0; i < 5; ++i) {
    x_segments[2 * i] = getOneHot(i, 10);
  }

  ClusterConfig config;
  config.score_threshold = 0.9;
  // Test top k (k = 1)
  config.cumulative = false;
  config.null_task_preprune = false;
  config.top_k = 1;  // Test single top k (one hot)
  auto ws = makeWorkspace({}, x_segments, config);

  Eigen::MatrixXd expected_py_x(4, 5);
  expected_py_x << 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
  EXPECT_TRUE(ws.py_x.isApprox(expected_py_x, 1e-9))
      << "expected: " << expected_py_x.format(fmt)
      << ", result: " << ws.py_x.format(fmt);
}

TEST(AgglomerativeIBClustering, SetupCumulativeCorrect) {
  NodeEmbeddingMap x_segments;
  for (size_t i = 0; i < 5; ++i) {
    x_segments[2 * i] = getOneHot(i, 10);
  }

  ClusterConfig config;
  config.score_threshold = 0.9;
  config.cumulative = true;  // Test cumulative
  config.null_task_preprune = false;
  config.top_k = 2;
  auto ws = makeWorkspace({}, x_segments, config);

  Eigen::MatrixXd expected_py_x(4, 5);
  expected_py_x << 0.9 / 2.9, 0.9 / 2.9, 0.9 / 2.9, 1.0, 1.0, 2.0 / 2.9, 0.0, 0.0, 0.0,
      0.0, 0.0, 2.0 / 2.9, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0 / 2.9, 0.0, 0.0;
  EXPECT_TRUE(ws.py_x.isApprox(expected_py_x, 1.0e-6))
      << "expected: " << expected_py_x.format(fmt) << "\n"
      << "result:   " << ws.py_x.format(fmt) << "\n"
      << "diff:     " << (expected_py_x - ws.py_x).format(fmt);
}

TEST(AgglomerativeIBClustering, SetupNullPruneCorrect) {
  NodeEmbeddingMap x_segments;
  for (size_t i = 0; i < 3; ++i) {
    x_segments[2 * i] = getOneHot(i, 10);
  }
  for (size_t i = 3; i < 5; ++i) {
    x_segments[2 * i] = getOneHot(i, 10) + getOneHot(0, 10);
  }

  ClusterConfig config;
  config.score_threshold = 0.9;
  config.cumulative = false;
  config.null_task_preprune = true;  // Test null preprune
  config.top_k = 100;
  auto ws = makeWorkspace({}, x_segments, config);

  Eigen::MatrixXd expected_py_x(4, 5);
  expected_py_x << 0.9 / 1.9, 0.9 / 1.9, 0.9 / 1.9, 1.0, 1.0, 1.0 / 1.9, 0.0, 0.0, 0.0,
      0.0, 0.0, 1.0 / 1.9, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 / 1.9, 0.0, 0.0;
  EXPECT_TRUE(ws.py_x.isApprox(expected_py_x, 1.0e-6))
      << "expected: " << expected_py_x.format(fmt) << "\n"
      << "result:   " << ws.py_x.format(fmt) << "\n"
      << "diff:     " << (expected_py_x - ws.py_x).format(fmt);
}

TEST(AgglomerativeIBClustering, UpdateCorrect) {
  NodeEmbeddingMap x_segments;
  for (size_t i = 0; i < 5; ++i) {
    x_segments[2 * i] = getOneHot(i, 10);
  }

  std::vector<EdgeKey> keys{{0, 2}, {2, 4}, {4, 6}, {6, 8}};

  ClusterConfig config;
  config.score_threshold = 1.0;
  config.cumulative = false;  // Test simplest case
  config.null_task_preprune = false;
  config.top_k = 100;  // Large so that essentially disabled
  auto ws = makeWorkspace({}, x_segments, config);

  std::list<EdgeKey> updated;
  ws.merge(EdgeKey(0, 1), updated, true);

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

TEST(AgglomerativeIBClustering, GetClustersCorrect) {
  NodeEmbeddingMap map;
  for (size_t i = 0; i < 5; ++i) {
    map[2 * i] = getOneHot(i, 10);
  }

  std::vector<EdgeKey> keys{{0, 2}, {2, 4}, {4, 6}, {6, 8}};
  auto ws = makeWorkspace(keys, map);

  {  // expected_assignments = {0, 1, 2, 3, 4}
    auto clusters = ws.getClusters();
    EXPECT_EQ(clusters.size(), 5);
    const auto result = getNodeAssignments(clusters);
    std::map<NodeId, size_t> expected{{0, 0}, {2, 1}, {4, 2}, {6, 3}, {8, 4}};
    EXPECT_EQ(expected, result) << workspaceState(ws);
  }

  std::list<EdgeKey> updated;
  ws.merge({1, 2}, updated, true);
  {  // expected_assignments = {0, 1, 1, 3, 4}
    auto clusters = ws.getClusters();
    EXPECT_EQ(clusters.size(), 4);
    const auto result = getNodeAssignments(clusters);
    std::map<NodeId, size_t> expected{{0, 0}, {2, 1}, {4, 1}, {6, 2}, {8, 3}};
    EXPECT_EQ(expected, result) << workspaceState(ws);
  }

  ws.merge({3, 4}, updated, true);
  {  // expected_assignments = {0, 1, 1, 3, 3}
    auto clusters = ws.getClusters();
    EXPECT_EQ(clusters.size(), 3);
    const auto result = getNodeAssignments(clusters);
    std::map<NodeId, size_t> expected{{0, 0}, {2, 1}, {4, 1}, {6, 2}, {8, 2}};
    EXPECT_EQ(expected, result) << workspaceState(ws);
  }

  ws.merge({0, 1}, updated, true);
  {  // expected_assignments = {0, 0, 0, 3, 3}
    auto clusters = ws.getClusters();
    EXPECT_EQ(clusters.size(), 2);
    const auto result = getNodeAssignments(clusters);
    std::map<NodeId, size_t> expected{{0, 0}, {2, 0}, {4, 0}, {6, 1}, {8, 1}};
    EXPECT_EQ(expected, result) << workspaceState(ws);
  }

  ws.merge({0, 3}, updated, true);
  {  // expected_assignments = {0, 0, 0, 0, 0}
    auto clusters = ws.getClusters();
    EXPECT_EQ(clusters.size(), 1);
    const auto result = getNodeAssignments(clusters);
    std::map<NodeId, size_t> expected{{0, 0}, {2, 0}, {4, 0}, {6, 0}, {8, 0}};
    EXPECT_EQ(expected, result) << workspaceState(ws);
  }
}

namespace {

// Features that all share one dominant direction (e0) plus a small distinct part:
// nodes 0-2 lean towards e1 and nodes 3-5 lean towards e2
Eigen::VectorXf sharedFeature(size_t node) {
  Eigen::VectorXf f = Eigen::VectorXf::Zero(6);
  f(0) = 1.0f;
  f(node < 3 ? 1 : 2) = 0.3f;
  return f;
}

// Query 'a' (distinct part e1) overlaps the shared direction much more than query 'b'
// (distinct part e2)
const std::string kUnequalQueries = R"(
type: from_config
names: [a, b]
embeddings: [[0.5, 0.3, 0.0, 0.0, 0.0, 0.0], [0.1, 0.0, 0.3, 0.0, 0.0, 0.0]]
)";

std::unique_ptr<SceneGraphLayer> makeSharedFeatureLayer() {
  auto layer = std::make_unique<SceneGraphLayer>(2);
  for (size_t i = 0; i < 6; ++i) {
    auto attrs = std::make_unique<SemanticNodeAttributes>();
    attrs->semantic_feature = sharedFeature(i);
    layer->emplaceNode(i, std::move(attrs));
    if (i > 0) {
      layer->insertEdge(i - 1, i);
    }
  }

  return layer;
}

AgglomerativeIBClustering::Config makeClusteringConfig(const std::string& params) {
  auto node = YAML::Load(params);
  node["queries"] = YAML::Load(kUnequalQueries);
  return config::fromYaml<AgglomerativeIBClustering::Config>(node);
}

// best query per node (-1 if the cluster score is below the threshold)
std::map<NodeId, int> getNodeLabels(const LayerClustering::Clusters& clusters,
                                    double threshold) {
  std::map<NodeId, int> labels;
  for (const auto& cluster : clusters) {
    for (const auto node_id : cluster->nodes) {
      labels[node_id] = cluster->score >= threshold ? cluster->best_query : -1;
    }
  }

  return labels;
}

}  // namespace

TEST(AgglomerativeIBClustering, CenterQueriesRemovesSharedDirection) {
  const auto queries = config::fromYaml<EmbeddingGroup>(YAML::Load(kUnequalQueries));
  NodeEmbeddingMap features;
  for (size_t i = 0; i < 6; ++i) {
    features[i] = sharedFeature(i);
  }

  CosineDistance dist;
  {  // without centering: query 'a' wins at every node
    for (const auto& [node_id, feature] : features) {
      EXPECT_EQ(queries.getBestScore(dist, feature).index, 0u) << "node " << node_id;
    }
  }

  const auto centered = AgglomerativeIBClustering::centerQueries(queries, features);
  ASSERT_EQ(centered.embeddings.size(), 2u);
  EXPECT_EQ(centered.names, queries.names);

  Eigen::VectorXf mean = Eigen::VectorXf::Zero(6);
  for (const auto& [node_id, feature] : features) {
    mean += feature.normalized();
  }
  mean.normalize();

  for (const auto& query : centered.embeddings) {
    EXPECT_NEAR(query.norm(), 1.0f, 1.0e-6f);
    EXPECT_NEAR(query.dot(mean), 0.0f, 1.0e-6f);
  }

  // with centering: the assignment follows the distinct parts
  for (const auto& [node_id, feature] : features) {
    const auto best = centered.getBestScore(dist, feature);
    EXPECT_EQ(best.index, node_id < 3 ? 0u : 1u) << "node " << node_id;
    EXPECT_LT(std::abs(best.score), 0.25f);  // centered scores are small
  }
}

TEST(AgglomerativeIBClustering, CenterQueriesClustering) {
  const auto layer = makeSharedFeatureLayer();
  ASSERT_TRUE(layer);

  {  // default: one query wins everywhere
    const auto config =
        makeClusteringConfig("{score_threshold: 0.2, top_k: 1, max_delta: 0.01}");
    ASSERT_FALSE(config.center_queries);
    const AgglomerativeIBClustering clustering(config);
    const auto labels = getNodeLabels(clustering.cluster(*layer), 0.2);
    ASSERT_EQ(labels.size(), 6u);
    for (const auto& [node_id, label] : labels) {
      EXPECT_EQ(label, 0) << "node " << node_id;
    }
  }

  {  // centered: the regions follow the distinct parts of the features
    const auto config = makeClusteringConfig(
        "{score_threshold: 0.01, top_k: 1, max_delta: 0.01, center_queries: true}");
    ASSERT_TRUE(config.center_queries);
    const AgglomerativeIBClustering clustering(config);
    const auto clusters = clustering.cluster(*layer);
    EXPECT_EQ(clusters.size(), 2u);
    const auto labels = getNodeLabels(clusters, 0.01);
    ASSERT_EQ(labels.size(), 6u);
    for (const auto& [node_id, label] : labels) {
      EXPECT_EQ(label, node_id < 3 ? 0 : 1) << "node " << node_id;
    }
  }
}

}  // namespace hydra
