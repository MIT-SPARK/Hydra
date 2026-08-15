#pragma once
#include <Eigen/Dense>
#include <memory>
#include <set>

namespace clio {

struct Cluster {
  using Ptr = std::shared_ptr<Cluster>;
  std::set<uint64_t> nodes;
  double score;
  Eigen::VectorXf feature;
  size_t best_task_index;
  std::string best_task_name;
};

}  // namespace clio
