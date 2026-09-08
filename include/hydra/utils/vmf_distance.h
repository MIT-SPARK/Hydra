/* -----------------------------------------------------------------------------
 * von Mises-Fisher distance utilities for per-place CLIP feature distributions.
 *
 * Sufficient statistics: feature_sum (sum of unit vectors) and observation_count.
 * Derived quantities (mu, r_bar, kappa) are computed on demand by computeVmfStats.
 *
 * Distance:  d(i, j) = 1 - || k_i * mu_i + k_j * mu_j || / (k_i + k_j)
 *   In [0, 1], Dijkstra-safe, concentration-weighted.
 *
 * Kappa MLE (Banerjee et al. 2005, "Clustering on the Unit Hypersphere using
 * von Mises-Fisher Distributions"):
 *   kappa ~= r_bar * (D - r_bar^2) / (1 - r_bar^2)
 * -------------------------------------------------------------------------- */
#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace hydra {

struct VmfStats {
  Eigen::VectorXf mu;
  float r_bar = 0.0f;
  float kappa = 0.0f;
  uint32_t n = 0u;
  bool valid = false;
};

inline float estimateVmfKappa(float r_bar,
                              int d,
                              uint32_t n,
                              float kappa_max = 500.0f) {
  if (n <= 1u) {
    return kappa_max;
  }
  const float denom = 1.0f - r_bar * r_bar;
  if (denom < 1e-12f) {
    return kappa_max;
  }
  const float k = r_bar * (static_cast<float>(d) - r_bar * r_bar) / denom;
  return std::min(std::max(k, 0.0f), kappa_max);
}

inline VmfStats computeVmfStats(const Eigen::VectorXf& feature_sum,
                                uint32_t observation_count,
                                float kappa_max = 500.0f) {
  VmfStats s;
  s.n = observation_count;
  if (observation_count == 0u || feature_sum.size() == 0) {
    return s;
  }
  const float norm = feature_sum.norm();
  if (norm < 1e-12f) {
    return s;
  }
  s.mu = feature_sum / norm;
  s.r_bar = std::min(norm / static_cast<float>(observation_count), 1.0f);
  s.kappa = estimateVmfKappa(s.r_bar, feature_sum.size(), observation_count, kappa_max);
  s.valid = true;
  return s;
}

inline float vmfDistance(const VmfStats& a, const VmfStats& b) {
  if (!a.valid || !b.valid) {
    return 0.0f;
  }
  if (a.mu.size() != b.mu.size()) {
    return 0.0f;
  }
  const float kappa_sum = a.kappa + b.kappa;
  if (kappa_sum < 1e-12f) {
    return 0.0f;
  }
  const float norm = (a.kappa * a.mu + b.kappa * b.mu).norm();
  return std::max(1.0f - norm / kappa_sum, 0.0f);
}

inline float vmfScore(const VmfStats& a, const VmfStats& b) {
  return 1.0f - vmfDistance(a, b);
}

}  // namespace hydra
