#pragma once

#include <spark_dsg/mesh.h>

#include "hydra/frontend/clustering_types.h"

namespace hydra::clustering {

struct VoxelClusteringConfig {
  double cluster_tolerance = 0.25;
  size_t min_cluster_size = 40;
  //! Maximum number of samples in a cluster (0 disables the upper limit).
  size_t max_cluster_size = 0;
};

void declare_config(VoxelClusteringConfig& config);

// Indices must be unique, in range, finite, and belong to a single label.
// Components use strict squared distance < tolerance^2, like PCL/FLANN.
// Output indices refer to the supplied mesh, sorted within each component.
Clusters findClusters(const VoxelClusteringConfig& config,
                      const spark_dsg::Mesh& mesh,
                      const std::vector<size_t>& indices);

}  // namespace hydra::clustering
