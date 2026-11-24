#include <gtest/gtest.h>
#include <spark_dsg/node_attributes.h>

#include <array>
#include <limits>

#include "hydra/openset/clustering/vmf_clustering.h"

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
  // Reference scores from FOUND-IT's foundit/places/clustering/propagation.py
  // at commit 4836f34c45c2b088fc6c5a6ec60f43c21c40b1d6.
  constexpr std::array<NodeId, 7> ids{10, 20, 30, 40, 50, 60, 90};
  using Scores = std::array<Eigen::Vector2f, ids.size()>;
  const Scores observations{
      {{2, 0}, {0, 8}, {0, 0}, {2.4f, 3.2f}, {16, 0}, {-0.8f, 0.6f}, {0, 6}}};
  const Scores one_iteration{{{1.0f, 3.11111111f},
                              {0.58823529f, 5.64705882f},
                              {9.0f, 0.0f},
                              {1.275f, 1.3875f},
                              {13.24137931f, 0.0f},
                              {1.66153846f, 2.6f},
                              {0.0f, 6.0f}}};
  const Scores default_scores{{{4.06971627f, 3.10228059f},
                               {1.19665928f, 6.55970865f},
                               {9.47140381f, 1.69787272f},
                               {2.78223770f, 2.91326209f},
                               {14.87442540f, 0.29256023f},
                               {1.95579009f, 2.37856706f},
                               {0.0f, 6.0f}}};
  const Scores converged{{{4.07017298f, 3.10295016f},
                          {1.19711453f, 6.55970993f},
                          {9.47236447f, 1.69787543f},
                          {2.78329352f, 2.91326505f},
                          {14.87453556f, 0.29272174f},
                          {1.95633041f, 2.37935912f},
                          {0.0f, 6.0f}}};
  const struct {
    size_t iterations;
    float threshold;
    const Scores& scores;
  } cases[] = {{0, 0.0f, observations},
               {1, 0.0f, one_iteration},
               {20, 1.0e-4f, default_scores},
               {100, 1.0e-4f, converged},
               {20, 1.0e9f, one_iteration}};

  SceneGraphLayer layer(1);
  addNode(layer, 10, {1, 0}, 2);
  addNode(layer, 20, {0, 1}, 8);
  addNode(layer, 30, {});
  addNode(layer, 40, {0.6f, 0.8f}, 4);
  addNode(layer, 50, {1, 0}, 16);
  addNode(layer, 60, {-0.8f, 0.6f}, 1);
  addNode(layer, 90, {0, 1}, 6);
  ASSERT_TRUE(layer.insertEdge(10, 20));
  ASSERT_TRUE(layer.insertEdge(10, 30));
  ASSERT_TRUE(layer.insertEdge(10, 40));
  ASSERT_TRUE(layer.insertEdge(30, 50));
  ASSERT_TRUE(layer.insertEdge(40, 60));

  const auto config = makeConfig();
  const auto queries = config.queries.create();
  const CosineDistance metric;
  const auto initial = vmf::computeScores(layer, *queries, metric, true);
  ASSERT_EQ(initial.size(), ids.size());
  for (const auto& [iterations, threshold, expected] : cases) {
    SCOPED_TRACE(::testing::Message()
                 << "iterations=" << iterations << " threshold=" << threshold);
    auto settings = config;
    settings.label_propagation_iterations = iterations;
    settings.convergence_threshold = threshold;
    const auto result = vmf::propagateScores(layer, initial, settings);
    ASSERT_EQ(result.size(), ids.size());
    for (size_t i = 0; i < ids.size(); ++i) {
      const auto& scores = result.at(ids[i]).scores;
      ASSERT_EQ(scores.size(), expected[i].size());
      for (Eigen::Index q = 0; q < scores.size(); ++q) {
        EXPECT_NEAR(scores(q), expected[i](q), 2.0e-5f)
            << "node=" << ids[i] << " query=" << q;
      }
    }
  }

  const auto clusters = VmfClustering(config).cluster(layer);
  std::unordered_map<NodeId, size_t> labels;
  for (const auto& cluster : clusters) {
    for (const auto id : cluster->nodes) {
      EXPECT_TRUE(labels.emplace(id, cluster->best_query).second);
    }
  }

  ASSERT_EQ(labels.size(), ids.size());
  for (size_t i = 0; i < ids.size(); ++i) {
    Eigen::Index expected;
    default_scores[i].maxCoeff(&expected);
    EXPECT_EQ(labels.at(ids[i]), expected) << "node=" << ids[i];
  }
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

TEST(VmfClustering, ExcludesNodesBeyondPropagationReach) {
  SceneGraphLayer layer(1);
  addNode(layer, 1, {0, 1}, 2);
  addNode(layer, 2, {});
  addNode(layer, 3, {});
  ASSERT_TRUE(layer.insertEdge(1, 2));
  ASSERT_TRUE(layer.insertEdge(2, 3));
  auto config = makeConfig();
  config.label_propagation_iterations = 1;
  const auto clusters = VmfClustering(config).cluster(layer);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters.front()->nodes, (std::set<NodeId>{1, 2}));
  EXPECT_EQ(clusters.front()->best_query, 1u);
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
