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
#pragma once
#include <config_utilities/factory.h>

#include <cstdint>

#include "hydra/reconstruction/voxel_types.h"

namespace hydra {

struct SemanticIntegrator {
  virtual ~SemanticIntegrator() = default;

  /**
   * @brief Check whether the point should be integrated based on the label
   * Note: this discards points that should not be contained in the map, e.g., dynamic
   * points or labels like 'sky'
   */
  virtual bool canIntegrate(uint32_t label) const = 0;

  /**
   * @brief Check whether this is a supported label for updating the voxel semantics
   * Note: this is different than canIntegrate and checks if this is a semantic label
   * that we are capable of tracking independent of the point itself being integrated
   * into the tsdf
   */
  virtual bool isValidLabel(uint32_t label) const = 0;

  virtual void updateLikelihoods(uint32_t label, SemanticVoxel& voxel) const = 0;
};

// Implementation based in part on Kimera-Semantics
class MLESemanticIntegrator : public SemanticIntegrator {
 public:
  struct Config {
    /// Measurement probability
    double label_confidence = 0.9;
  } const config;

  explicit MLESemanticIntegrator(const Config& config);

  bool canIntegrate(uint32_t label) const override;

  bool isValidLabel(uint32_t label) const override;

  void updateLikelihoods(uint32_t label, SemanticVoxel& voxel) const override;

 protected:
  size_t total_labels_;
  std::set<uint32_t> dynamic_labels_;
  std::set<uint32_t> invalid_labels_;

  float init_likelihood_;
  Eigen::MatrixXf observation_likelihoods_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<SemanticIntegrator,
                                     MLESemanticIntegrator,
                                     MLESemanticIntegrator::Config>(
          "MLESemanticIntegrator");
};

void declare_config(MLESemanticIntegrator::Config& config);

}  // namespace hydra
