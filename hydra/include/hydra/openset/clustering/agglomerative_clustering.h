#pragma once
#include <config_utilities/virtual_config.h>
#include <spark_dsg/scene_graph_layer.h>

#include <Eigen/Dense>
#include <map>

#include "hydra/openset/embedding_distances.h"

namespace hydra {

class AgglomerativeClustering {
 public:
  struct Workspace {
    using NodeEmbeddings = std::map<spark_dsg::NodeId, Eigen::VectorXf>;
    Workspace(const spark_dsg::SceneGraphLayer& layer,
              const NodeEmbeddings& node_embeddings);

    Workspace(const spark_dsg::SceneGraphLayer& layer,
              const std::vector<spark_dsg::NodeId>& nodes);

    Workspace(const spark_dsg::SceneGraphLayer& layer);

    size_t size() const;
    size_t featureDim() const;

    void setup(const EmbeddingGroup& tasks,
               const EmbeddingDistance& metric,
               bool reweight = false,
               double I_xy = -1.0,
               double delta_weight = 1.0);
    double score(const spark_dsg::EdgeKey& edge) const;
    bool merge(spark_dsg::EdgeKey to_merge,
               double max_delta,
               std::list<spark_dsg::EdgeKey>& updated);
    std::string summary(double max_delta) const;

    std::vector<std::vector<spark_dsg::NodeId>> getClusters() const;

    std::map<size_t, Eigen::VectorXf> features;
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

  struct Config {
    config::VirtualConfig<EmbeddingGroup> tasks;
    config::VirtualConfig<EmbeddingDistance> metric{CosineDistance::Config()};
    float score_threshold = 0.23f;
    size_t top_k = 2;
    bool cumulative = true;
    bool null_task_preprune = true;
    double max_delta = 1.0e-3;
    double tolerance = -1.0e-18;
    bool filter_clusters = false;
  } const config;

  AgglomerativeClustering(const Config& config);

  Clusters cluster(const spark_dsg::SceneGraphLayer& layer,
                   const NodeEmbeddingMap& embeddings) const;

  static void cluster(Workspace& workspace,
                      const EmbeddingGroup& tasks,
                      const EmbeddingDistance& metric,
                      bool reweight = false,
                      double I_xy = -1,
                      double delta_weight = 1,
                      int verbosity = 5);

 protected:
  static bool updateFromEdge(spark_dsg::EdgeKey edge);

  static std::string summarize();

 protected:
  EmbeddingGroup::Ptr tasks_;
  std::unique_ptr<EmbeddingDistance> metric_;
};

void declare_config(AgglomerativeClustering::Config& config);

}  // namespace hydra
