#pragma once

#include <unordered_map>

#include "hydra/openset/clustering/vmf_clustering.h"

namespace hydra::vmf {

// Internal numerical workspace for clustering and score regression tests.
struct ScoreEntry {
  Eigen::VectorXf scores;
  float kappa;
};

using ScoreWorkspace = std::unordered_map<spark_dsg::NodeId, ScoreEntry>;

ScoreWorkspace computeScores(const spark_dsg::SceneGraphLayer& layer,
                             const EmbeddingGroup& queries,
                             const EmbeddingDistance& metric,
                             bool allow_empty_scores);

ScoreWorkspace propagateScores(const spark_dsg::SceneGraphLayer& layer,
                               const ScoreWorkspace& initial,
                               const VmfClustering::Config& config);

}  // namespace hydra::vmf
