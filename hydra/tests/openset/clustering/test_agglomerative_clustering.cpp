#include <clio/agglomerative_clustering.h>
#include <clio/clustering_workspace.h>
#include <clio/common.h>
#include <clio/embedding_distances.h>
#include <clio/ib_edge_selector.h>
#include <gtest/gtest.h>

namespace clio {

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
    tasks.push_back("task_0");
  }

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<EmbeddingGroup, FakeEmbeddingGroup, Config>(
          "fake_embeddings");
};

void declare_config(FakeEmbeddingGroup::Config&) {}

inline AgglomerativeClustering::Config getFakeConfig() {
  AgglomerativeClustering::Config config;
  config.metric =
      config::VirtualConfig<EmbeddingDistance>(CosineDistance::Config(), "cosine");
  config.selector = config::VirtualConfig<EdgeSelector>(IBEdgeSelector::Config(), "IB");
  config.tasks = config::VirtualConfig<EmbeddingGroup>(FakeEmbeddingGroup::Config(),
                                                       "fake_embeddings");
  return config;
}

std::map<NodeId, size_t> getNodeAssignments(const std::vector<Cluster::Ptr>& clusters) {
  std::map<NodeId, size_t> assignments;
  for (size_t i = 0; i < clusters.size(); ++i) {
    for (const auto& node_id : clusters[i]->nodes) {
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

std::string workspaceState(const ClusteringWorkspace& ws) {
  std::stringstream ss;
  ss << "lookup: " << printMap(ws.node_lookup) << std::endl;
  ss << "order: " << printMap(ws.order) << std::endl;
  ss << "assignments: " << printVec(ws.assignments) << std::endl;
  ss << "edges: " << printMap(ws.edges) << std::endl;
  return ss.str();
}

}  // namespace

TEST(AgglomerativeClustering, GetClustersCorrect) {
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
  const auto config = getFakeConfig();
  AgglomerativeClustering clustering(config);

  {  // expected_assignments = {0, 1, 2, 3, 4}
    auto clusters = clustering.getClusters(ws, map);
    EXPECT_EQ(clusters.size(), 5);
    const auto result = getNodeAssignments(clusters);
    std::map<NodeId, size_t> expected{{0, 0}, {2, 1}, {4, 2}, {6, 3}, {8, 4}};
    EXPECT_EQ(expected, result) << workspaceState(ws);
  }

  ws.addMerge({1, 2});
  {  // expected_assignments = {0, 1, 1, 3, 4}
    auto clusters = clustering.getClusters(ws, map);
    EXPECT_EQ(clusters.size(), 4);
    const auto result = getNodeAssignments(clusters);
    std::map<NodeId, size_t> expected{{0, 0}, {2, 1}, {4, 1}, {6, 2}, {8, 3}};
    EXPECT_EQ(expected, result) << workspaceState(ws);
  }

  ws.addMerge({3, 4});
  {  // expected_assignments = {0, 1, 1, 3, 3}
    auto clusters = clustering.getClusters(ws, map);
    EXPECT_EQ(clusters.size(), 3);
    const auto result = getNodeAssignments(clusters);
    std::map<NodeId, size_t> expected{{0, 0}, {2, 1}, {4, 1}, {6, 2}, {8, 2}};
    EXPECT_EQ(expected, result) << workspaceState(ws);
  }

  ws.addMerge({0, 1});
  {  // expected_assignments = {0, 0, 0, 3, 3}
    auto clusters = clustering.getClusters(ws, map);
    EXPECT_EQ(clusters.size(), 2);
    const auto result = getNodeAssignments(clusters);
    std::map<NodeId, size_t> expected{{0, 0}, {2, 0}, {4, 0}, {6, 1}, {8, 1}};
    EXPECT_EQ(expected, result) << workspaceState(ws);
  }

  ws.addMerge({0, 3});
  {  // expected_assignments = {0, 0, 0, 0, 0}
    auto clusters = clustering.getClusters(ws, map);
    EXPECT_EQ(clusters.size(), 1);
    const auto result = getNodeAssignments(clusters);
    std::map<NodeId, size_t> expected{{0, 0}, {2, 0}, {4, 0}, {6, 0}, {8, 0}};
    EXPECT_EQ(expected, result) << workspaceState(ws);
  }
}

}  // namespace clio
