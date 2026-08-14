#include "hydra/openset/embedding_distances.h"

#include <glog/logging.h>

namespace hydra {

float CosineDistance::dist(const FeatureVector& lhs, const FeatureVector& rhs) const {
  // map [-1, 1] to [0, 2]
  return 1.0f - score(lhs, rhs);
}

float CosineDistance::score(const FeatureVector& lhs, const FeatureVector& rhs) const {
  const auto divisor = std::max(lhs.norm() * rhs.norm(), config.tolerance);
  return lhs.dot(rhs) / divisor;
}

float L2Norm::dist(const FeatureVector& lhs, const FeatureVector& rhs) const {
  return (lhs.normalized() - rhs.normalized()).norm();
}

float L2Norm::score(const FeatureVector& lhs, const FeatureVector& rhs) const {
  // map [2, 0] to [0, 1]
  return 1.0f - 0.5f * dist(lhs, rhs);
}

float L1Norm::dist(const FeatureVector& lhs, const FeatureVector& rhs) const {
  return (lhs.normalized() - rhs.normalized()).lpNorm<1>();
}

float L1Norm::score(const FeatureVector& lhs, const FeatureVector& rhs) const {
  // map [2, 0] to [0, 1]
  return 1.0f - 0.5f * dist(lhs, rhs);
}

LerfScore::LerfScore(const Config& config) : config(config::checkValid(config)) {
  cannonical_ = config.cannonical_features.create();
}

float LerfScore::dist(const FeatureVector& lhs, const FeatureVector& rhs) const {
  // technically the softmax isn't a dist
  return 1.0f - score(lhs, rhs);
}

float LerfScore::score(const FeatureVector& lhs, const FeatureVector& rhs) const {
  CosineDistance::Config dot_config;
  dot_config.tolerance = config.tolerance;
  const CosineDistance dot_prod(dot_config);
  // score maps to cosine similiarity (for CosineDistance norm)
  const auto l_dot_r = std::exp(dot_prod.score(lhs, rhs));
  const auto scores = cannonical_->getScores(dot_prod, lhs);
  const auto divisor = (scores.array().exp() + l_dot_r);
  return (l_dot_r / divisor).minCoeff();
}

void declare_config(LerfScore::Config& config) {
  using namespace config;
  name("LerfScore::Config");
  field(config.cannonical_features, "cannonical_features");
  field(config.tolerance, "tolerance");
}

}  // namespace hydra
