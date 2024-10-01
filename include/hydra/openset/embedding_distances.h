#pragma once
#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <config_utilities/virtual_config.h>

#include <Eigen/Dense>

#include "hydra/openset/embedding_group.h"

namespace hydra {

struct EmbeddingDistance {
  virtual ~EmbeddingDistance() = default;

  inline float operator()(const FeatureVector& lhs, const FeatureVector& rhs) const {
    return score(lhs, rhs);
  }

  virtual float dist(const FeatureVector& lhs, const FeatureVector& rhs) const = 0;

  virtual float score(const FeatureVector& lhs, const FeatureVector& rhs) const = 0;
};

struct CosineDistance : EmbeddingDistance {
  struct Config {
    float tolerance = 1.0e-9f;
  };

  explicit CosineDistance() : CosineDistance(Config()) {}

  explicit CosineDistance(const Config& config) : config(config::checkValid(config)) {}

  float dist(const FeatureVector& lhs, const FeatureVector& rhs) const override;

  float score(const FeatureVector& lhs, const FeatureVector& rhs) const override;

  const Config config;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<EmbeddingDistance, CosineDistance, Config>(
          "cosine");
};

inline void declare_config(CosineDistance::Config& config) {
  config::name("CosineDistance::Config");
  config::field(config.tolerance, "tolerance");
  config::check(config.tolerance, config::GT, 0.0, "tolerance");
}

struct L1Norm : public EmbeddingDistance {
  struct Config {};

  explicit L1Norm(const Config& = {}) {}

  float dist(const FeatureVector& lhs, const FeatureVector& rhs) const override;

  float score(const FeatureVector& lhs, const FeatureVector& rhs) const override;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<EmbeddingDistance, L1Norm, Config>("l1");
};

inline void declare_config(L1Norm::Config&) { config::name("L1Norm::Config"); }

struct L2Norm : public EmbeddingDistance {
  struct Config {};

  explicit L2Norm(const Config& = {}) {}

  float dist(const FeatureVector& lhs, const FeatureVector& rhs) const override;

  float score(const FeatureVector& lhs, const FeatureVector& rhs) const override;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<EmbeddingDistance, L2Norm, Config>("l2");
};

inline void declare_config(L2Norm::Config&) { config::name("L2Norm::Config"); }

struct LerfScore : public EmbeddingDistance {
  struct Config {
    config::VirtualConfig<EmbeddingGroup> cannonical_features;
    float tolerance = 1.0e-9f;
  };

  explicit LerfScore(const Config& config);

  float dist(const FeatureVector& lhs, const FeatureVector& rhs) const override;

  float score(const FeatureVector& lhs, const FeatureVector& rhs) const override;

  const Config config;

 private:
  std::unique_ptr<EmbeddingGroup> cannonical_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<EmbeddingDistance, LerfScore, Config>("lerf");
};

void declare_config(LerfScore::Config& config);

}  // namespace hydra
