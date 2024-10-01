#include "hydra/openset/embedding_group.h"

#include <glog/logging.h>

#include "hydra/openset/embedding_distances.h"
#include "hydra/utils/printing.h"

namespace hydra {

using ScoreResult = EmbeddingGroup::ScoreResult;

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
