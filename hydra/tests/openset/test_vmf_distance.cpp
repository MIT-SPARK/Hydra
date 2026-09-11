#include <gtest/gtest.h>

#include "hydra/openset/vmf_distance.h"

namespace hydra {
namespace {

Eigen::VectorXf makeUnit(int d, int axis) {
  Eigen::VectorXf v = Eigen::VectorXf::Zero(d);
  v(axis) = 1.0f;
  return v;
}

}  // namespace

TEST(VmfDistance, ZeroObservationsAreInvalid) {
  Eigen::VectorXf sum = Eigen::VectorXf::Zero(0);
  const auto stats = computeVmfStats(sum, 0u);
  EXPECT_FALSE(stats.valid);
  EXPECT_EQ(stats.n, 0u);
}

TEST(VmfDistance, SingleObservationClampsKappaToMax) {
  Eigen::VectorXf sum = makeUnit(8, 0);
  const auto stats = computeVmfStats(sum, 1u, 500.0f);
  EXPECT_TRUE(stats.valid);
  EXPECT_EQ(stats.n, 1u);
  EXPECT_FLOAT_EQ(stats.r_bar, 1.0f);
  EXPECT_FLOAT_EQ(stats.kappa, 500.0f);
}

TEST(VmfDistance, IdenticalDistributionsHaveZeroDistance) {
  Eigen::VectorXf sum = makeUnit(16, 3) * 5.0f;
  const auto a = computeVmfStats(sum, 5u, 500.0f);
  const auto b = computeVmfStats(sum, 5u, 500.0f);
  EXPECT_NEAR(vmfDistance(a, b), 0.0f, 1e-6f);
  EXPECT_NEAR(vmfScore(a, b), 1.0f, 1e-6f);
}

TEST(VmfDistance, AntipodalDistributionsHaveUnitDistance) {
  Eigen::VectorXf mu_a = makeUnit(16, 0);
  Eigen::VectorXf mu_b = -mu_a;
  // Bound kappa to a finite value so n=1 doesn't push it to kappa_max.
  // Use n=10 with r_bar==1 so kappa is again kappa_max but the antipodal
  // direction still yields ||k*mu_a + k*(-mu_a)|| / (2k) = 0 -> distance 1.
  const auto a = computeVmfStats(mu_a * 10.0f, 10u, 500.0f);
  const auto b = computeVmfStats(mu_b * 10.0f, 10u, 500.0f);
  EXPECT_NEAR(vmfDistance(a, b), 1.0f, 1e-6f);
}

TEST(VmfDistance, KappaParityWithPythonReferenceAtRbar07) {
  // kappa = 0.7 * (1024 - 0.49) / (1 - 0.49) = 0.7 * 1023.51 / 0.51 = ~1404.66
  // Clipped to kappa_max=500 -> 500. Test at kappa_max=5000 to avoid the clip.
  const float k = estimateVmfKappa(0.7f, 1024, 10u, 5000.0f);
  const float expected = 0.7f * (1024.0f - 0.49f) / (1.0f - 0.49f);
  EXPECT_NEAR(k, expected, 1e-2f);
}

TEST(VmfDistance, DimensionMismatchReturnsZero) {
  Eigen::VectorXf sum_a = makeUnit(8, 0) * 4.0f;
  Eigen::VectorXf sum_b = makeUnit(16, 0) * 4.0f;
  const auto a = computeVmfStats(sum_a, 4u);
  const auto b = computeVmfStats(sum_b, 4u);
  EXPECT_FLOAT_EQ(vmfDistance(a, b), 0.0f);
}

TEST(VmfDistance, RbarReflectsConcentration) {
  // Two parallel observations: r_bar = 1.
  Eigen::VectorXf parallel_sum = makeUnit(8, 0) * 2.0f;
  EXPECT_FLOAT_EQ(computeVmfStats(parallel_sum, 2u).r_bar, 1.0f);

  // Two orthogonal observations: ||sum||=sqrt(2), r_bar = sqrt(2)/2 ~= 0.707.
  Eigen::VectorXf orth_sum = makeUnit(8, 0) + makeUnit(8, 1);
  const auto orth = computeVmfStats(orth_sum, 2u);
  EXPECT_NEAR(orth.r_bar, std::sqrt(2.0f) / 2.0f, 1e-6f);
}

}  // namespace hydra
