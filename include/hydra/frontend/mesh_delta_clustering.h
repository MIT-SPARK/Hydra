#pragma once
#include <map>
#include <set>
#include <unordered_set>
#include <vector>

#include "kimera_pgmo/mesh_delta.h"

namespace hydra::clustering {

using Clusters = std::vector<std::vector<size_t>>;
using LabelIndices = std::map<uint32_t, std::vector<size_t>>;
using LabelSet = std::set<uint32_t>;

struct ClusteringConfig {
  double cluster_tolerance = 0.25;
  size_t min_cluster_size = 40;
  size_t max_cluster_size = 100000;
};

void declare_config(ClusteringConfig& config);

std::string printLabels(const LabelSet& labels);

LabelIndices getLabelIndices(const LabelSet& desired_labels,
                             const kimera_pgmo::MeshDelta& delta,
                             const std::unordered_set<size_t>* invalid = nullptr);

Clusters findClusters(const ClusteringConfig& config,
                      const kimera_pgmo::MeshDelta& delta,
                      const std::vector<size_t>& indices);

}  // namespace hydra::clustering
