#pragma once

#include <hydra/openset/embedding_distances.h>
#include <hydra/openset/embedding_group.h>
#include <spark_dsg/dynamic_scene_graph.h>

#include <Eigen/Dense>

#include "clio/clustering_workspace.h"
#include "clio/scene_graph_types.h"

namespace clio {

struct PyGivenXConfig {
  float score_threshold = 0.23f;
  size_t top_k = 2;
  bool cumulative = true;
  bool null_task_preprune = true;
};

Eigen::MatrixXd computeIBpyGivenX(const ClusteringWorkspace& ws,
                                  const hydra::EmbeddingGroup& tasks,
                                  const hydra::EmbeddingDistance& metric,
                                  const PyGivenXConfig& config);

Eigen::VectorXd computeIBpx(const ClusteringWorkspace& ws);

Eigen::VectorXd computeIBpy(const hydra::EmbeddingGroup& tasks);

double computeDeltaWeight(const spark_dsg::SceneGraphLayer& layer,
                          const std::vector<NodeId>& nodes);

}  // namespace clio
