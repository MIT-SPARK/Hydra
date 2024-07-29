#include "hydra/openset/embedding_pooling.h"

#include <config_utilities/config.h>

namespace hydra {

void declare_config(MeanPooling::Config&) { config::name("MeanPooling::Config"); }

Eigen::VectorXf MeanPooling::pool(const Eigen::MatrixXf& features,
                                  const Eigen::VectorXf&) const {
  if (features.cols() == 0) {
    return Eigen::VectorXf();
  }

  return features.rowwise().mean();
}

void declare_config(WeightedMeanPooling::Config&) {
  config::name("WeightedMeanPooling::Config");
}

Eigen::VectorXf WeightedMeanPooling::pool(const Eigen::MatrixXf& features,
                                          const Eigen::VectorXf& weights) const {
  if (features.cols() == 0 || weights.rows() != features.cols()) {
    return Eigen::VectorXf();
  }

  const Eigen::VectorXf weighted = features * weights;
  return weighted / features.cols();
}

void declare_config(MaxPooling::Config&) { config::name("MaxPooling::Config"); }
Eigen::VectorXf MaxPooling::pool(const Eigen::MatrixXf& features,
                                 const Eigen::VectorXf& weights) const {
  if (features.cols() == 0 || weights.rows() != features.cols()) {
    return Eigen::VectorXf();
  }

  int min_row = 0;
  int min_col = 0;
  weights.maxCoeff(&min_row, &min_col);
  return features.col(min_row);
}

}  // namespace hydra
