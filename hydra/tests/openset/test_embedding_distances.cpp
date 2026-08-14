#include <gtest/gtest.h>
#include <hydra/openset/embedding_distances.h>

namespace hydra {

TEST(EmbeddingDistances, TestCosineCorrect) {
  FeatureVector a = FeatureVector::Zero(10);
  FeatureVector b = FeatureVector::Zero(10);
  a(0) = 1.0f;
  b(0) = -1.0f;
  CosineDistance dist;
  EXPECT_NEAR(dist.score(a, b), -1.0f, 1.0e-9f);
  EXPECT_NEAR(dist.dist(a, b), 2.0f, 1.0e-9f);

  b(0) = 2.0f;
  EXPECT_NEAR(dist.score(a, b), 1.0f, 1.0e-9f);
  EXPECT_NEAR(dist.dist(a, b), 0.0f, 1.0e-9f);

  b(0) = 0.0f;
  EXPECT_NEAR(dist.score(a, b), 0.0f, 1.0e-9f);
  EXPECT_NEAR(dist.dist(a, b), 1.0f, 1.0e-9f);

  a(0) = 0.0f;
  EXPECT_NEAR(dist.score(a, b), 0.0f, 1.0e-9f);
  EXPECT_NEAR(dist.dist(a, b), 1.0f, 1.0e-9f);
}

TEST(EmbeddingDistances, TestL1Norm) {
  FeatureVector a = FeatureVector::Zero(10);
  FeatureVector b = FeatureVector::Zero(10);
  a(0) = 1.0f;
  b(0) = -1.0f;
  L1Norm dist;
  EXPECT_NEAR(dist.score(a, b), 0.0f, 1.0e-9f);
  EXPECT_NEAR(dist.dist(a, b), 2.0f, 1.0e-9f);

  b(0) = 2.0f;
  EXPECT_NEAR(dist.score(a, b), 1.0f, 1.0e-9f);
  EXPECT_NEAR(dist.dist(a, b), 0.0f, 1.0e-9f);

  b(0) = 0.0f;
  EXPECT_NEAR(dist.score(a, b), 0.5f, 1.0e-9f);
  EXPECT_NEAR(dist.dist(a, b), 1.0f, 1.0e-9f);

  a(0) = 0.0f;
  EXPECT_NEAR(dist.score(a, b), 1.0f, 1.0e-9f);
  EXPECT_NEAR(dist.dist(a, b), 0.0f, 1.0e-9f);
}

TEST(EmbeddingDistances, TestL2Norm) {
  FeatureVector a = FeatureVector::Zero(10);
  FeatureVector b = FeatureVector::Zero(10);
  a(0) = 1.0f;
  b(0) = -1.0f;
  L2Norm dist;
  EXPECT_NEAR(dist.score(a, b), 0.0f, 1.0e-9f);
  EXPECT_NEAR(dist.dist(a, b), 2.0f, 1.0e-9f);

  b(0) = 2.0f;
  EXPECT_NEAR(dist.score(a, b), 1.0f, 1.0e-9f);
  EXPECT_NEAR(dist.dist(a, b), 0.0f, 1.0e-9f);

  b(0) = 0.0f;
  EXPECT_NEAR(dist.score(a, b), 0.5f, 1.0e-9f);
  EXPECT_NEAR(dist.dist(a, b), 1.0f, 1.0e-9f);

  a(0) = 0.0f;
  EXPECT_NEAR(dist.score(a, b), 1.0f, 1.0e-9f);
  EXPECT_NEAR(dist.dist(a, b), 0.0f, 1.0e-9f);
}

/*
// TODO(nathan) move embedding group fixture over
TEST(EmbeddingDistances, TestLerf) {
  LerfScore::Config config;
  config.cannonical_features = test::TestEmbeddingGroup::getDefault();
  LerfScore dist(config);

  FeatureVector a = FeatureVector::Zero(10);
  FeatureVector b = FeatureVector::Zero(10);
  a(0) = 1.0f;
  b(0) = -1.0f;
  {
    const double expected = std::exp(-1.0f) / (std::exp(-1.0f) + std::exp(1.0f));
    EXPECT_NEAR(dist.score(a, b), expected, 1.0e-9f);
  }

  b(0) = 2.0f;
  EXPECT_NEAR(dist.score(a, b), 0.5f, 1.0e-9f);

  b(0) = 0.0f;
  {
    const double expected = 1.0f / (1.0f + std::exp(1.0f));
    EXPECT_NEAR(dist.score(a, b), expected, 1.0e-9f);
  }

  a(0) = 0.0f;
  EXPECT_NEAR(dist.score(a, b), 0.5f, 1.0e-9f);
}
*/

}  // namespace hydra
