#include "hydra/openset/embedding_pooling.h"

#include <config_utilities/config.h>

namespace hydra {

void declare_config(MeanPooling::Config&) { config::name("MeanPooling::Config"); }

FeatureVector MeanPooling::pool(const FeatureGroup& features,
                                const WeightVector&) const {
  if (features.cols() == 0) {
    return FeatureVector();
  }

  return features.rowwise().mean();
}

void declare_config(WeightedMeanPooling::Config&) {
  config::name("WeightedMeanPooling::Config");
}

FeatureVector WeightedMeanPooling::pool(const FeatureGroup& features,
                                        const WeightVector& weights) const {
  if (features.cols() == 0 || weights.rows() != features.cols()) {
    return FeatureVector();
  }

  const FeatureVector weighted = features * weights;
  return weighted / features.cols();
}

void declare_config(MaxPooling::Config&) { config::name("MaxPooling::Config"); }

FeatureVector MaxPooling::pool(const FeatureGroup& features,
                               const WeightVector& weights) const {
  if (features.cols() == 0 || weights.rows() != features.cols()) {
    return FeatureVector();
  }

  int min_row = 0;
  int min_col = 0;
  weights.maxCoeff(&min_row, &min_col);
  return features.col(min_row);
}

}  // namespace hydra
