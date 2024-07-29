#pragma once
#include <config_utilities/factory.h>

#include <Eigen/Dense>

namespace hydra {

struct FeaturePooling {
  using Ptr = std::unique_ptr<FeaturePooling>;
  virtual ~FeaturePooling() = default;
  virtual Eigen::VectorXf pool(const Eigen::MatrixXf& features,
                               const Eigen::VectorXf& weights) const = 0;
};

struct MeanPooling : FeaturePooling {
  struct Config {};
  explicit MeanPooling(const Config& = {}) {}
  Eigen::VectorXf pool(const Eigen::MatrixXf& features,
                       const Eigen::VectorXf& weights) const override;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<FeaturePooling, MeanPooling, Config>("mean");
};

void declare_config(MeanPooling::Config&);

struct WeightedMeanPooling : FeaturePooling {
  struct Config {};
  explicit WeightedMeanPooling(const Config& = {}) {}
  Eigen::VectorXf pool(const Eigen::MatrixXf& features,
                       const Eigen::VectorXf& weights) const override;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<FeaturePooling, WeightedMeanPooling, Config>(
          "weighted_mean");
};

void declare_config(WeightedMeanPooling::Config&);

struct MaxPooling : FeaturePooling {
  struct Config {};
  explicit MaxPooling(const Config& = {}) {}
  Eigen::VectorXf pool(const Eigen::MatrixXf& features,
                       const Eigen::VectorXf& weights) const override;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<FeaturePooling, MaxPooling, Config>("max");
};

void declare_config(MaxPooling::Config&);

}  // namespace hydra
