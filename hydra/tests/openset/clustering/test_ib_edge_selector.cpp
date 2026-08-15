#include <clio/common.h>
#include <clio/embedding_distances.h>
#include <clio/ib_edge_selector.h>
#include <clio/ib_utils.h>
#include <gtest/gtest.h>

namespace clio {

struct TestableIBEdgeSelector : public IBEdgeSelector {
  explicit TestableIBEdgeSelector(const IBEdgeSelector::Config& config)
      : IBEdgeSelector(config) {}

  using IBEdgeSelector::px_;
  using IBEdgeSelector::py_;
  using IBEdgeSelector::py_x_;
  using IBEdgeSelector::py_z_;
  using IBEdgeSelector::pz_;
  using IBEdgeSelector::pz_x_;
};

namespace {

inline Eigen::VectorXd getOneHot(size_t i, size_t dim) {
  Eigen::VectorXd p = Eigen::VectorXd::Zero(dim);
  p(i) = 1.0;
  return p;
}

}  // namespace

TEST(IBEdgeSelector, SetupSimpleCorrect) {
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

  ASSERT_EQ(selector.px_.rows(), 5);
  ASSERT_EQ(selector.pz_.rows(), 5);
  ASSERT_EQ(selector.py_.rows(), 4);
  ASSERT_EQ(selector.pz_x_.rows(), 5);
  ASSERT_EQ(selector.pz_x_.cols(), 5);
  ASSERT_EQ(selector.py_x_.rows(), 4);
  ASSERT_EQ(selector.py_x_.cols(), 5);
  ASSERT_EQ(selector.py_z_.rows(), 4);
  ASSERT_EQ(selector.py_z_.cols(), 5);
  EXPECT_TRUE(selector.px_.isApprox(selector.pz_));
  EXPECT_TRUE(selector.py_x_.isApprox(selector.py_z_));

  const auto fmt = getDefaultFormat();
  Eigen::VectorXd expected_px(5);
  expected_px << 0.2, 0.2, 0.2, 0.2, 0.2;  // row-wise sum normalized via l1-norm
  EXPECT_TRUE(selector.px_.isApprox(expected_px))
      << "expected: " << expected_px.format(fmt)
      << ", result: " << selector.px_.format(fmt);

  Eigen::MatrixXd expected_py_x(4, 5);
  expected_py_x << 0.5, 0.5, 0.5, 1.0, 1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.5, 0.0, 0.0;
  EXPECT_TRUE(selector.py_x_.isApprox(expected_py_x, 1e-9))
      << "expected: " << expected_py_x.format(fmt)
      << ", result: " << selector.py_x_.format(fmt);

  Eigen::VectorXd expected_py(4);
  expected_py << 0.25, 0.25, 0.25, 0.25;  // row-wise sum normalized via l1-norm
  EXPECT_TRUE(selector.py_.isApprox(expected_py))
      << "expected: " << expected_py.format(fmt)
      << ", result: " << selector.py_.format(fmt);
}

TEST(IBEdgeSelector, SetupTopKCorrect) {
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
  config.py_x.score_threshold = 0.9;
  // Test top k (k = 1)
  config.py_x.cumulative = false;
  config.py_x.null_task_preprune = false;
  config.py_x.top_k = 1;  // Test single top k (one hot)
  TestableIBEdgeSelector selector(config);
  selector.setup(ws, y_tasks, dist);

  const auto fmt = getDefaultFormat();

  Eigen::MatrixXd expected_py_x(4, 5);
  expected_py_x << 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
  EXPECT_TRUE(selector.py_x_.isApprox(expected_py_x, 1e-9))
      << "expected: " << expected_py_x.format(fmt)
      << ", result: " << selector.py_x_.format(fmt);
}

TEST(IBEdgeSelector, SetupCumulativeCorrect) {
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
  config.py_x.score_threshold = 0.9;
  // Test cumulative
  config.py_x.cumulative = true;
  config.py_x.null_task_preprune = false;
  config.py_x.top_k = 2;
  TestableIBEdgeSelector selector(config);
  selector.setup(ws, y_tasks, dist);

  const auto fmt = getDefaultFormat();

  Eigen::MatrixXd expected_py_x(4, 5);
  expected_py_x << 0.9 / 2.9, 0.9 / 2.9, 0.9 / 2.9, 1.0, 1.0, 2.0 / 2.9, 0.0, 0.0, 0.0,
      0.0, 0.0, 2.0 / 2.9, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0 / 2.9, 0.0, 0.0;
  EXPECT_TRUE(selector.py_x_.isApprox(expected_py_x, 1e-9))
      << "expected: " << expected_py_x.format(fmt)
      << ", result: " << selector.py_x_.format(fmt);
}

TEST(IBEdgeSelector, SetupNullPruneCorrect) {
  IsolatedSceneGraphLayer layer(2);

  NodeEmbeddingMap x_segments;
  for (size_t i = 0; i < 3; ++i) {
    layer.emplaceNode(2 * i, std::make_unique<NodeAttributes>());
    x_segments[2 * i] = getOneHot(i, 10);
  }
  for (size_t i = 3; i < 5; ++i) {
    layer.emplaceNode(2 * i, std::make_unique<NodeAttributes>());
    x_segments[2 * i] = getOneHot(i, 10) + getOneHot(0, 10);
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

TEST(IBEdgeSelector, ComputeDeltaWeightCorrect) {
  IsolatedSceneGraphLayer layer(2);
  std::vector<NodeId> nodes;
  NodeEmbeddingMap x_segments;
  for (size_t i = 0; i < 5; ++i) {
    layer.emplaceNode(2 * i, std::make_unique<NodeAttributes>());
    x_segments[2 * i] = getOneHot(i, 10);
    nodes.push_back(2 * i);
  }

  EXPECT_EQ(computeDeltaWeight(layer, std::vector<NodeId>{}), 0.0);
  EXPECT_EQ(computeDeltaWeight(layer, nodes), 1.0);
}

}  // namespace clio
