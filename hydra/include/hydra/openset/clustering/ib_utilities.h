#pragma once

#include <spark_dsg/scene_graph_layer.h>

#include <Eigen/Dense>

#include "hydra/openset/embedding_distances.h"

namespace hydra {

Eigen::MatrixXd computeIBpyGivenX(const ClusteringWorkspace& ws,
                                  const EmbeddingGroup& tasks,
                                  const EmbeddingDistance& metric,
                                  const PyGivenXConfig& config);

}  // namespace hydra
