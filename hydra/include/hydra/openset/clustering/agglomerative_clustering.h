#pragma once
#include <config_utilities/virtual_config.h>
#include <spark_dsg/scene_graph_layer.h>

#include <Eigen/Dense>
#include <map>

#include "hydra/openset/embedding_distances.h"
#include "hydra/utils/logging.h"

namespace hydra {

class AgglomerativeClustering {
 public:
  struct ClusteringConfig {
    float score_threshold = 0.23f;
    size_t top_k = 2;
    bool cumulative = true;
    bool null_task_preprune = true;
    double max_delta = 1.0e-3;
  };

  struct Config : VerbosityConfig {
    ClusteringConfig clustering;
    config::VirtualConfig<EmbeddingGroup> tasks;
    config::VirtualConfig<EmbeddingDistance> metric{CosineDistance::Config()};
    bool filter_clusters = false;
  } const config;

  struct Workspace {
    using NodeEmbeddings = FeatureMap<spark_dsg::NodeId>;
    using ClusterIndices = std::vector<std::vector<spark_dsg::NodeId>>;

    Workspace(const ClusteringConfig& config,
              const spark_dsg::EdgeContainer::Edges& edges,
              const NodeEmbeddings& node_embeddings,
              const EmbeddingGroup& tasks,
              const EmbeddingDistance& metric);

    void reweight(double I_xy, double delta_weight);

    double score(const spark_dsg::EdgeKey& edge) const;

    bool merge(spark_dsg::EdgeKey to_merge,
               std::list<spark_dsg::EdgeKey>& updated,
               bool force = false);

    size_t size() const;

    size_t featureDim() const;

    std::string summary() const;

    ClusterIndices getClusters() const;

    static Eigen::MatrixXd compute_py_x(const ClusteringConfig& config,
                                        const std::vector<FeatureVector>& features,
                                        const EmbeddingGroup& tasks,
                                        const EmbeddingDistance& metric);

    const ClusteringConfig config;

    std::vector<FeatureVector> features;
    std::map<size_t, spark_dsg::NodeId> node_lookup;
    std::map<spark_dsg::NodeId, size_t> order;
    std::map<spark_dsg::EdgeKey, double> edges;
    std::vector<size_t> assignments;

    // p(x), p(z), p(y)
    Eigen::VectorXd px;
    Eigen::VectorXd pz;
    Eigen::VectorXd py;

    // p(z|x), p(y|x), p(y|z)
    Eigen::MatrixXd pz_x;  // NxN
    Eigen::MatrixXd py_x;  // 2xN
    Eigen::MatrixXd py_z;  // 2xN

    // mutual information caches
    double I_xy;
    double I_zy_prev;
    double delta_weight = 1.0;
    std::vector<double> deltas;
  };

  struct Cluster {
    using Ptr = std::shared_ptr<Cluster>;
    std::set<uint64_t> nodes;
    double score;
    Eigen::VectorXf feature;
    size_t best_task_index;
    std::string best_task_name;
  };
  using Clusters = std::vector<Cluster::Ptr>;
  using NodeEmbeddingMap = std::map<spark_dsg::NodeId, Eigen::VectorXf>;

  AgglomerativeClustering(const Config& config);

  Clusters cluster(const spark_dsg::SceneGraphLayer& layer,
                   const NodeEmbeddingMap& embeddings) const;

  static void cluster(Workspace& workspace);

 protected:
  EmbeddingGroup::Ptr tasks_;
  std::unique_ptr<EmbeddingDistance> metric_;
};

void declare_config(AgglomerativeClustering::ClusteringConfig& config);
void declare_config(AgglomerativeClustering::Config& config);

}  // namespace hydra
