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
#include "hydra/openset/vmf_distance.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace hydra {

float estimateVmfKappa(float r_bar, size_t d, uint32_t n, float kappa_max) {
  if (n <= 1) {
    return kappa_max;
  }

  const auto denom = 1.0f - r_bar * r_bar;
  if (denom < 1.0e-12f) {
    return kappa_max;
  }

  const auto k = r_bar * (d - r_bar * r_bar) / denom;
  return std::clamp(k, 0.0f, kappa_max);
}

VmfStats computeVmfStats(const Eigen::VectorXf& feature_sum,
                         size_t observation_count,
                         float kappa_max) {
  VmfStats s;
  s.n = observation_count;
  if (observation_count == 0u || feature_sum.size() == 0) {
    return s;
  }

  const float norm = feature_sum.norm();
  if (norm < 1.0e-12f) {
    return s;
  }

  s.mu = feature_sum / norm;
  s.r_bar = std::min(norm / observation_count, 1.0f);
  s.kappa = estimateVmfKappa(s.r_bar, feature_sum.size(), observation_count, kappa_max);
  s.valid = true;
  return s;
}

float vmfDistance(const VmfStats& a, const VmfStats& b) {
  if (!a.valid || !b.valid) {
    return 0.0f;
  }

  if (a.mu.size() != b.mu.size()) {
    return 0.0f;
  }

  const auto kappa_sum = a.kappa + b.kappa;
  if (kappa_sum < 1.0e-12f) {
    return 0.0f;
  }

  const float norm = (a.kappa * a.mu + b.kappa * b.mu).norm();
  return std::max(1.0f - norm / kappa_sum, 0.0f);
}

float vmfScore(const VmfStats& a, const VmfStats& b) {
  return 1.0f - vmfDistance(a, b);
}

}  // namespace hydra
