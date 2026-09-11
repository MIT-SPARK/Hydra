/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
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
#include <cstdint>

namespace hydra {

struct VmfStats {
  Eigen::VectorXf mu;
  float r_bar = 0.0f;
  float kappa = 0.0f;
  uint32_t n = 0u;
  bool valid = false;
};

float estimateVmfKappa(float r_bar, size_t d, uint32_t n, float kappa_max = 500.0f);

VmfStats computeVmfStats(const Eigen::VectorXf& feature_sum,
                         uint32_t observation_count,
                         float kappa_max = 500.0f);

float vmfDistance(const VmfStats& a, const VmfStats& b);

float vmfScore(const VmfStats& a, const VmfStats& b);

}  // namespace hydra
