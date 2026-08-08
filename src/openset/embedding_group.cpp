#include "hydra/openset/embedding_group.h"

#include <config_utilities/types/eigen_matrix.h>
#include <glog/logging.h>

#include "hydra/openset/embedding_distances.h"
#include "hydra/utils/printing.h"

namespace hydra {
namespace {

// NOTE(nathan) this looks weird but the EmbeddingGroup struct is a config and class
static const auto reg =
    config::RegistrationWithConfig<EmbeddingGroup, EmbeddingGroup, EmbeddingGroup>(
        "from_config");

struct EmbeddingConverter {
  using InterT = std::vector<std::vector<float>>;
  using ValueT = std::vector<FeatureVector>;

  static InterT toIntermediate(const ValueT& value, std::string&) {
    InterT other;
    for (const auto& feature : value) {
      auto& vec = other.emplace_back();
      vec.resize(feature.rows());
      for (Eigen::Index r = 0; r < feature.rows(); ++r) {
        vec[r] = feature(r);
      }
    }

    return other;
  }

  static void fromIntermediate(const InterT& other, ValueT& value, std::string& error) {
    if (other.empty()) {
      return;
    }

    size_t dim = 0;
    for (const auto& vec : other) {
      if (dim == 0) {
        dim = vec.size();
      } else if (dim != vec.size()) {
        error = "Embedding dimensions do not match: " + std::to_string(dim) +
                " != " + std::to_string(vec.size());
        value.clear();
        return;
      }

      auto& feature = value.emplace_back(FeatureVector::Zero(dim));
      for (size_t i = 0; i < vec.size(); ++i) {
        feature(i) = vec[i];
      }
    }
  }
};

}  // namespace

using ScoreResult = EmbeddingGroup::ScoreResult;

void declare_config(EmbeddingGroup& group) {
  using namespace config;
  name("EmbeddingGroup");
  // required to handle dynamic eigen types and enforce size constraints
  field<EmbeddingConverter>(group.embeddings, "embeddings");
  field(group.names, "names");
  checkCondition(group.names.empty() || (group.names.size() == group.embeddings.size()),
                 "names must be same size as embeddings if specified");
}

EmbeddingGroup::~EmbeddingGroup() {}

bool EmbeddingGroup::empty() const { return embeddings.empty(); }

Eigen::VectorXf EmbeddingGroup::getDistances(const EmbeddingDistance& dist,
                                             const FeatureVector& embedding) const {
  Eigen::VectorXf distances(embeddings.size());
  for (size_t i = 0; i < embeddings.size(); ++i) {
    distances(i) = dist.dist(embeddings[i], embedding);
  }

  return distances;
}

Eigen::VectorXf EmbeddingGroup::getScores(const EmbeddingDistance& dist,
                                          const FeatureVector& embedding) const {
  const auto fmt = getDefaultFormat();
  VLOG(30) << "====================================================================";
  VLOG(30) << "Embedding: " << embedding.format(fmt);
  VLOG(30) << "====================================================================";
  Eigen::VectorXf scores(embeddings.size());
  for (size_t i = 0; i < embeddings.size(); ++i) {
    scores(i) = dist.score(embeddings[i], embedding);
    VLOG(30) << "(i=" << i << "): " << scores(i) << " @ " << embeddings[i].format(fmt);
  }

  return scores;
}

ScoreResult EmbeddingGroup::getBestScore(const EmbeddingDistance& dist,
                                         const FeatureVector& embedding) const {
  ScoreResult result;
  for (size_t i = 0; i < embeddings.size(); ++i) {
    const auto score = dist.score(embeddings[i], embedding);
    if (score > result.score) {
      result.score = score;
      result.index = i;
    }
  }

  return result;
}

}  // namespace hydra
