#pragma once
#include <Eigen/Dense>

#include "clio/ib_utils.h"

namespace clio {

class IBEdgeSelector {
 public:
  using WeightedEdge = std::pair<spark_dsg::EdgeKey, double>;
  using ScoreFunc = std::function<double(const Eigen::VectorXd&)>;
  using Ptr = std::unique_ptr<IBEdgeSelector>;

  struct Config {
    double max_delta = 1.0e-3;
    double tolerance = -1.0e-18;
    PyGivenXConfig py_x;
  };

  explicit IBEdgeSelector(const Config& config);

  virtual ~IBEdgeSelector() = default;

  void setup(const ClusteringWorkspace& ws,
             const hydra::EmbeddingGroup& tasks,
             const hydra::EmbeddingDistance& metric);

  double scoreEdge(spark_dsg::EdgeKey edge);

  bool updateFromEdge(spark_dsg::EdgeKey edge);

  bool compareEdges(const WeightedEdge& lhs, const WeightedEdge& rhs) const;

  void onlineReweighting(double Ixy, double delta_weight);

  const Config config;

  std::string summarize() const;

 protected:
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

void declare_config(IBEdgeSelector::Config& config);

}  // namespace clio
