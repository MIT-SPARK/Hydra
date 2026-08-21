#pragma once
#include <Eigen/Dense>

namespace hydra {

/**
 * @brief Compute Shannon entropy for a giving PMF
 * @param p PMF of the distribution
 * @param tolerance Threshold for near-zero probability
 * @returns Shannon entropy of the distribution
 */
double shannonEntropy(const Eigen::Ref<const Eigen::VectorXd>& p,
                      double tolerance = 1.0e-9);

/**
 * @brief Compute the JS divergence of N distributions
 * @param p MxN matrix where each column is the PMF for one of the distributions
 * @param priors Weights between the N distributions
 * @param tolerance Threshold for near-zero probability
 * @returns JS divergence of the distributions
 */
double jensenShannonDivergence(const Eigen::Ref<const Eigen::MatrixXd>& p,
                               const Eigen::Ref<const Eigen::VectorXd>& priors,
                               double tolerance = 1.0e-9);

/**
 * @brief Compute the mutual information between two distributions
 * @param pa Marginal of the first distribution p(a)
 * @param pb Marginal of the second distribution p(b)
 * @param pa_b Conditional probability distribution p(a|b)
 * @param tolerance Threshold for near-zero probability
 * @returns mutual information between the two distributions
 */
double mutualInformation(const Eigen::Ref<const Eigen::VectorXd>& pa,
                         const Eigen::Ref<const Eigen::VectorXd>& pb,
                         const Eigen::Ref<const Eigen::MatrixXd>& pa_b,
                         double tolerance = 1.0e-9);

}  // namespace hydra
