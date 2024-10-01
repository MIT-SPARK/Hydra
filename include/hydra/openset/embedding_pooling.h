#pragma once
#include <config_utilities/factory.h>

#include <Eigen/Dense>

#include "hydra/openset/openset_types.h"

namespace hydra {

struct FeaturePooling {
  using Ptr = std::unique_ptr<FeaturePooling>;
  using WeightVector = Eigen::VectorXf;
  virtual ~FeaturePooling() = default;
  virtual FeatureVector pool(const FeatureGroup& features,
                             const WeightVector& weights) const = 0;
};

struct MeanPooling : FeaturePooling {
  struct Config {};
  explicit MeanPooling(const Config& = {}) {}
  FeatureVector pool(const FeatureGroup& features,
                     const WeightVector& weights) const override;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<FeaturePooling, MeanPooling, Config>("mean");
};

void declare_config(MeanPooling::Config&);

struct WeightedMeanPooling : FeaturePooling {
  struct Config {};
  explicit WeightedMeanPooling(const Config& = {}) {}
  FeatureVector pool(const FeatureGroup& features,
                     const WeightVector& weights) const override;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<FeaturePooling, WeightedMeanPooling, Config>(
          "weighted_mean");
};

void declare_config(WeightedMeanPooling::Config&);

struct MaxPooling : FeaturePooling {
  struct Config {};
  explicit MaxPooling(const Config& = {}) {}
  FeatureVector pool(const FeatureGroup& features,
                     const WeightVector& weights) const override;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<FeaturePooling, MaxPooling, Config>("max");
};

void declare_config(MaxPooling::Config&);

}  // namespace hydra
