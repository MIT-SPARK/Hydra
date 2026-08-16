#pragma once
#include <config_utilities/virtual_config.h>
#include <spark_dsg/scene_graph_layer.h>

#include <Eigen/Dense>
#include <list>
#include <map>

#include "hydra/openset/embedding_distances.h"

namespace hydra {

struct ClusteringWorkspace {
  using NodeEmbeddings = std::map<spark_dsg::NodeId, Eigen::VectorXf>;
  std::map<size_t, Eigen::VectorXf> features;
  std::map<size_t, spark_dsg::NodeId> node_lookup;
  std::map<spark_dsg::NodeId, size_t> order;
  std::map<spark_dsg::EdgeKey, double> edges;
  std::vector<size_t> assignments;

  ClusteringWorkspace(const spark_dsg::SceneGraphLayer& layer,
                      const NodeEmbeddings& node_embeddings);

  ClusteringWorkspace(const spark_dsg::SceneGraphLayer& layer,
                      const std::vector<spark_dsg::NodeId>& nodes);

  ClusteringWorkspace(const spark_dsg::SceneGraphLayer& layer);

  size_t size() const;

  size_t featureDim() const;

  std::list<spark_dsg::EdgeKey> addMerge(spark_dsg::EdgeKey to_merge);

  std::vector<std::vector<spark_dsg::NodeId>> getClusters() const;
};

class AgglomerativeClustering {
 public:
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
  using WeightedEdge = std::pair<spark_dsg::EdgeKey, double>;
  using ScoreFunc = std::function<double(const Eigen::VectorXd&)>;

  struct Config {
    config::VirtualConfig<EmbeddingGroup> tasks;
    config::VirtualConfig<EmbeddingDistance> metric{CosineDistance::Config()};
    PyGivenXConfig py_x;
    double max_delta = 1.0e-3;
    double tolerance = -1.0e-18;
    bool filter_regions = false;
  } const config;

  AgglomerativeClustering(const Config& config);

  Clusters cluster(const spark_dsg::SceneGraphLayer& layer,
                   const NodeEmbeddingMap& embeddings) const;

  Clusters getClusters(const ClusteringWorkspace& workspace,
                       const NodeEmbeddingMap& features) const;

  static void cluster(ClusteringWorkspace& ws,
                      const EmbeddingGroup& tasks,
                      const EmbeddingDistance& metric,
                      bool reweight = false,
                      double I_xy = -1,
                      double delta_weight = 1,
                      int verbosity = 5);

 protected:
  static void setup(const ClusteringWorkspace& ws,
                    const EmbeddingGroup& tasks,
                    const EmbeddingDistance& metric);

  static double scoreEdge(spark_dsg::EdgeKey edge);

  static bool updateFromEdge(spark_dsg::EdgeKey edge);

  static bool compareEdges(const WeightedEdge& lhs, const WeightedEdge& rhs);

  static void onlineReweighting(double Ixy, double delta_weight);

  static std::string summarize();

 protected:
  EmbeddingGroup::Ptr tasks_;
  std::unique_ptr<EmbeddingDistance> metric_;

  // p(x), p(z), p(y)
  Eigen::VectorXd px_;
  Eigen::VectorXd pz_;
  Eigen::VectorXd py_;
  // p(z|x), p(y|x), p(y|z)
  Eigen::MatrixXd pz_x_;  // NxN
  Eigen::MatrixXd py_x_;  // 2xN
  Eigen::MatrixXd py_z_;  // 2xN
  // mutual information caches
  double I_xy_;
  double I_zy_prev_;
  double delta_weight_ = 1.0;
  std::vector<double> deltas_;
};

void declare_config(AgglomerativeClustering::Config& config);

}  // namespace hydra
