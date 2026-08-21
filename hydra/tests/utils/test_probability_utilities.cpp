#include <gtest/gtest.h>
#include <hydra/utils/printing.h>
#include <hydra/utils/probability_utilities.h>

namespace hydra {

Eigen::VectorXd getBinaryProbability(double p_true) {
  Eigen::VectorXd p(2);
  p << p_true, 1.0 - p_true;
  return p;
}

TEST(ProbabilityUtilities, TestShannonEntropy) {
  const auto fmt = getDefaultFormat();
  {  // test case 1: no entropy
    Eigen::VectorXd p = getBinaryProbability(0.0);
    EXPECT_NEAR(shannonEntropy(p), 0.0, 1.0e-9) << "input: " << p.format(fmt);
  }

  {  // test case 2: also no entropy
    Eigen::VectorXd p = getBinaryProbability(1.0);
    EXPECT_NEAR(shannonEntropy(p), 0.0, 1.0e-9) << "input: " << p.format(fmt);
  }

  {  // test case 3: max entropy (of 1 bit)
    Eigen::VectorXd p = getBinaryProbability(0.5);
    EXPECT_NEAR(shannonEntropy(p), 1.0, 1.0e-9) << "input: " << p.format(fmt);
  }
}

TEST(ProbabilityUtilities, TestJSDivergence) {
  {  // test case 1: complete divergence
    Eigen::MatrixXd dists = Eigen::MatrixXd::Identity(2, 2);
    Eigen::VectorXd priors = getBinaryProbability(0.5);
    EXPECT_NEAR(jensenShannonDivergence(dists, priors), 1.0, 1.0e-9);
  }

  {  // test case 2: no divergence
    Eigen::MatrixXd dists(2, 2);
    dists << 0.5, 0.5, 0.5, 0.5;
    Eigen::VectorXd priors = getBinaryProbability(0.5);
    EXPECT_NEAR(jensenShannonDivergence(dists, priors), 0.0, 1.0e-9);
  }

  {  // test case 3: no divergence (deterministic prior)
    Eigen::MatrixXd dists = Eigen::MatrixXd::Identity(5, 5);
    Eigen::VectorXd priors = Eigen::VectorXd::Zero(5);
    priors(3) = 1.0;
    EXPECT_NEAR(jensenShannonDivergence(dists, priors), 0.0, 1.0e-9);
  }

  {  // test case 4: complete divergence (uniform prior)
    Eigen::MatrixXd dists = Eigen::MatrixXd::Identity(5, 5);
    Eigen::VectorXd priors = Eigen::VectorXd::Constant(5, 0.2);
    EXPECT_NEAR(jensenShannonDivergence(dists, priors), std::log2(5), 1.0e-9);
  }

  {  // test case 5: interesting probabilities
    Eigen::MatrixXd dists = Eigen::MatrixXd::Zero(3, 2);
    dists << 3.0 / 8.0, 2.0 / 8.0, 4.0 / 8.0, 3.0 / 8.0, 1.0 / 8.0, 3.0 / 8.0;
    Eigen::VectorXd priors = getBinaryProbability(0.5);
    // D(p1||M) = 0.06996044115887057
    // D(p2||M) = 0.055481756047425
    EXPECT_NEAR(jensenShannonDivergence(dists, priors), 0.06272109860314778, 1.0e-9);
  }
}

TEST(ProbabilityUtilities, TestMutualInformation) {
  const auto fmt = getDefaultFormat();
  {  // test case 1: mutual info of 1
    Eigen::MatrixXd pa_b = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd p_a = getBinaryProbability(0.5);
    Eigen::MatrixXd p_b = getBinaryProbability(0.5);
    EXPECT_NEAR(mutualInformation(p_a, p_b, pa_b), 1.0, 1.0e-9)
        << "p(a): " << p_a.format(fmt) << ", p(b): " << p_b.format(fmt)
        << ", p(a|b): " << pa_b.format(fmt);
  }

  {  // test case 2: mutual info of 0
    Eigen::MatrixXd pa_b(2, 2);
    pa_b << 0.5, 0.5, 0.5, 0.5;
    Eigen::MatrixXd p_a = getBinaryProbability(0.5);
    Eigen::MatrixXd p_b = getBinaryProbability(0.5);
    EXPECT_NEAR(mutualInformation(p_a, p_b, pa_b), 0.0, 1.0e-9)
        << "p(a): " << p_a.format(fmt) << ", p(b): " << p_b.format(fmt)
        << ", p(a|b): " << pa_b.format(fmt);
  }

  {  // test case 2: mutual info for bad distribution
    Eigen::MatrixXd pa_b(2, 2);
    pa_b << 0.5, 0.5, 0.5, 0.5;
    Eigen::MatrixXd p_a = getBinaryProbability(1.0);
    Eigen::MatrixXd p_b = getBinaryProbability(0.5);
    // TODO(nathan) technically, mutual information is non-negative, but the joint
    // distribution we're providing isn't valid
    EXPECT_NEAR(mutualInformation(p_a, p_b, pa_b), -0.5, 1.0e-9)
        << "p(a): " << p_a.format(fmt) << ", p(b): " << p_b.format(fmt)
        << ", p(a|b): " << pa_b.format(fmt);
  }
}

}  // namespace hydra
