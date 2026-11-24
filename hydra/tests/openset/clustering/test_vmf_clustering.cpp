#include <gtest/gtest.h>
#include <spark_dsg/node_attributes.h>

#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>

#include "hydra/openset/clustering/vmf_clustering.h"
#include "hydra_test/resources.h"
#include "openset/clustering/vmf_clustering_utilities.h"

namespace hydra {
namespace {

using spark_dsg::NodeId;
using spark_dsg::SceneGraphLayer;
using spark_dsg::SemanticNodeAttributes;

void addNode(SceneGraphLayer& layer,
             NodeId id,
             const std::vector<float>& feature,
             float kappa = 0.0f) {
  auto attrs = std::make_unique<SemanticNodeAttributes>();
  attrs->semantic_feature =
      Eigen::Map<const Eigen::VectorXf>(feature.data(), feature.size());
  if (!feature.empty()) {
    attrs->feature_concentration = Eigen::MatrixXf::Constant(1, 1, kappa);
  }

  ASSERT_TRUE(layer.emplaceNode(id, std::move(attrs)));
}

VmfClustering::Config makeConfig() {
  EmbeddingGroup queries;
  queries.embeddings = {Eigen::Vector2f(1, 0), Eigen::Vector2f(0, 1)};
  queries.names = {"office", "hallway"};
  VmfClustering::Config config;
  config.queries = queries;
  return config;
}

}  // namespace

TEST(VmfClustering, MatchesPinnedFoundItPropagation) {
  std::ifstream stream(test::get_resource_path("openset/vmf_reference.json"));
  ASSERT_TRUE(stream.good());
  const auto fixture = nlohmann::json::parse(stream);
  SceneGraphLayer layer(1);
  const auto ids = fixture.at("ids").get<std::vector<NodeId>>();
  vmf::ScoreWorkspace initial;
  for (size_t i = 0; i < ids.size(); ++i) {
    const auto feature = fixture.at("embeddings").at(i).get<std::vector<float>>();
    const auto kappa = fixture.at("kappas").at(i).get<float>();
    addNode(layer, ids[i], feature, kappa);
  }

  for (const auto& edge : fixture.at("edges")) {
    ASSERT_TRUE(layer.insertEdge(edge.at(0).get<NodeId>(), edge.at(1).get<NodeId>()));
  }

  const auto config = makeConfig();
  const VmfClustering clustering(config);
  const auto queries = config.queries.create();
  const CosineDistance metric;
  initial = vmf::computeScores(layer, *queries, metric, true);
  ASSERT_EQ(initial.size(), ids.size());
  for (const auto& entry : fixture.at("cases")) {
    auto settings = config;
    settings.label_propagation_iterations = entry.at("iterations");
    settings.convergence_threshold = entry.at("epsilon");
    const auto result = vmf::propagateScores(layer, initial, settings);
    ASSERT_EQ(result.size(), ids.size());
    for (size_t i = 0; i < ids.size(); ++i) {
      const auto expected = entry.at("scores").at(i).get<std::vector<float>>();
      for (size_t q = 0; q < expected.size(); ++q) {
        EXPECT_NEAR(result.at(ids[i]).scores(q), expected[q], 2.0e-5f)
            << "node=" << ids[i] << " query=" << q
            << " iterations=" << settings.label_propagation_iterations;
      }
    }
  }

  const auto clusters = clustering.cluster(layer);
  ASSERT_FALSE(clusters.empty());
  std::set<NodeId> assigned;
  for (const auto& cluster : clusters) {
    for (const auto id : cluster->nodes) {
      EXPECT_TRUE(assigned.insert(id).second);
      const auto index = std::find(ids.begin(), ids.end(), id) - ids.begin();
      const auto expected = fixture.at("cases").at(2).at("scores").at(index);
      EXPECT_EQ(cluster->best_query,
                expected.at(1).get<float>() > expected.at(0).get<float>() ? 1u : 0u);
    }
  }

  EXPECT_EQ(assigned.size(), ids.size());
}

TEST(VmfClustering, MissingFeaturesReceiveSupportOrAreExcluded) {
  SceneGraphLayer layer(1);
  addNode(layer, 1, {0, 1}, 2);
  addNode(layer, 2, {});
  addNode(layer, 3, {});
  addNode(layer, 4, {});
  ASSERT_TRUE(layer.insertEdge(1, 2));
  ASSERT_TRUE(layer.insertEdge(3, 4));
  auto config = makeConfig();
  auto clusters = VmfClustering(config).cluster(layer);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters.front()->nodes, (std::set<NodeId>{1, 2}));
  EXPECT_EQ(clusters.front()->best_query, 1u);
  EXPECT_TRUE(clusters.front()->feature.isApprox(Eigen::Vector2f(0, 1)));
  config.allow_empty_scores = false;
  clusters = VmfClustering(config).cluster(layer);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters.front()->nodes, (std::set<NodeId>{1}));
}

TEST(VmfClustering, NoEvidenceReturnsNoRegions) {
  SceneGraphLayer layer(1);
  addNode(layer, 1, {});
  addNode(layer, 2, {1, 0}, 0);
  ASSERT_TRUE(layer.insertEdge(1, 2));
  EXPECT_TRUE(VmfClustering(makeConfig()).cluster(layer).empty());
}

TEST(VmfClustering, RejectsMalformedFeaturesWithoutEigenAssertions) {
  SceneGraphLayer layer(1);
  addNode(layer, 1, {1, 0, 0}, 1);
  addNode(layer, 2, {std::numeric_limits<float>::quiet_NaN(), 0}, 1);
  addNode(layer, 3, {1, 0}, -1);
  addNode(layer, 4, {0, 0}, 1);
  auto config = makeConfig();
  config.allow_empty_scores = false;
  EXPECT_TRUE(VmfClustering(config).cluster(layer).empty());
}

TEST(VmfClustering, KeepsPropagatedLabelAndAveragesFeatures) {
  SceneGraphLayer layer(1);
  addNode(layer, 1, {1, 0}, 100);
  addNode(layer, 2, {0, 1}, 1);
  addNode(layer, 3, {0, 1}, 1);
  ASSERT_TRUE(layer.insertEdge(1, 2));
  ASSERT_TRUE(layer.insertEdge(1, 3));
  const auto clusters = VmfClustering(makeConfig()).cluster(layer);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters.front()->nodes.size(), 3u);
  EXPECT_EQ(clusters.front()->best_query, 0u);
  EXPECT_TRUE(clusters.front()->feature.isApprox(Eigen::Vector2f(1.0f / 3, 2.0f / 3)));
  EXPECT_NEAR(clusters.front()->score, 1.0f / std::sqrt(5.0f), 1.0e-6f);
}

TEST(VmfClustering, SameLabelDisconnectedRegionsStaySeparate) {
  SceneGraphLayer layer(1);
  addNode(layer, 1, {1, 0}, 2);
  addNode(layer, 2, {1, 0}, 4);
  const auto clusters = VmfClustering(makeConfig()).cluster(layer);
  ASSERT_EQ(clusters.size(), 2u);
  for (const auto& cluster : clusters) {
    EXPECT_EQ(cluster->nodes.size(), 1u);
    EXPECT_EQ(cluster->best_query, 0u);
  }
}

}  // namespace hydra
