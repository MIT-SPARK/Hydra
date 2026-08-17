#include "hydra/utils/probability_utilities.h"

namespace hydra {

// -sum(p(x)log(p(x))) over all x
double shannonEntropy(const Eigen::Ref<const Eigen::VectorXd>& p, double tolerance) {
  double entropy = 0.0;
  for (int i = 0; i < p.rows(); ++i) {
    const double p_i = p(i, 0);
    if (p_i < tolerance) {
      continue;
    }

    entropy += p_i * std::log2(p_i);
  }

  return -entropy;
}

// compute the Jensen-Shannon divergence of p(a|b) (uses prior p(b))
double jensenShannonDivergence(const Eigen::Ref<const Eigen::MatrixXd>& pa_b,
                               const Eigen::Ref<const Eigen::VectorXd>& p_b,
                               double tolerance) {
  // M = sum_i=0^|{1, 2, ... m}| p_i(x) p(y|x=i)
  // p(a|b) * p(b) = \sum_i=0^|a| p(a|b=i)p(b=i)
  double total_entropy = 0.0;
  const Eigen::VectorXd M = pa_b * p_b;
  for (int i = 0; i < pa_b.cols(); ++i) {
    total_entropy += p_b(i) * shannonEntropy(pa_b.col(i), tolerance);
  }

  return shannonEntropy(M, tolerance) - total_entropy;
}

// compute the mutual information between two distributions
double mutualInformation(const Eigen::Ref<const Eigen::VectorXd>& pa,
                         const Eigen::Ref<const Eigen::VectorXd>& pb,
                         const Eigen::Ref<const Eigen::MatrixXd>& pa_b,
                         double tolerance) {
  if (pa_b.cols() != pb.rows() || pa_b.rows() != pa.rows()) {
    throw std::domain_error("Invalid argument: conditional probability p(a|b) of [" +
                            std::to_string(pa_b.rows()) + ", " +
                            std::to_string(pa_b.cols()) + "] doesn't match p(a) (" +
                            std::to_string(pa.rows()) + " elements) or p(b) (" +
                            std::to_string(pb.rows()) + " elements");
  }

  double total = 0.0;
  for (int b = 0; b < pa_b.cols(); ++b) {
    for (int a = 0; a < pa_b.rows(); ++a) {
      const auto p_joint = pa_b(a, b);
      // avoid log blowing up for events that can't occur
      if (p_joint < tolerance || pa(a) < tolerance) {
        continue;
      }

      total += pb(b) * p_joint * std::log2(p_joint / pa(a));
    }
  }

  return total;
}

}  // namespace hydra
