#pragma once
#include <config_utilities/virtual_config.h>
#include <hydra/openset/embedding_group.h>

#include <Eigen/Dense>

#include "clio/cluster.h"
#include "clio/clustering_workspace.h"
#include "clio/ib_edge_selector.h"
#include "clio/scene_graph_types.h"

namespace clio {

void clusterAgglomerative(ClusteringWorkspace& ws,
                          const hydra::EmbeddingGroup& tasks,
                          EdgeSelector& edge_selector,
                          const hydra::EmbeddingDistance& metric,
                          bool reweight = false,
                          double I_xy = -1,
                          double delta_weight = 1,
                          int verbosity = 5);

class AgglomerativeClustering {
 public:
  using Clusters = std::vector<Cluster::Ptr>;
  using NodeEmbeddingMap = std::map<NodeId, Eigen::VectorXf>;

  struct Config {
    config::VirtualConfig<hydra::EmbeddingGroup> tasks;
    config::VirtualConfig<hydra::EmbeddingDistance> metric{
        hydra::CosineDistance::Config()};
    IBEdgeSelector::Config selector;
    bool filter_regions = false;
  } const config;

  AgglomerativeClustering(const Config& config);

  Clusters cluster(const spark_dsg::SceneGraphLayer& layer,
                   const NodeEmbeddingMap& embeddings) const;

  Clusters getClusters(const ClusteringWorkspace& workspace,
                       const NodeEmbeddingMap& features) const;

 private:
  hydra::EmbeddingGroup::Ptr tasks_;
  std::unique_ptr<hydra::EmbeddingDistance> metric_;
  EdgeSelector::Ptr edge_selector_;
};

void declare_config(AgglomerativeClustering::Config& config);

}  // namespace clio
