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

#include <cstdint>
#include <set>

#include "hydra/reconstruction/voxel_types.h"

namespace hydra {

struct SemanticIntegrator {
  virtual ~SemanticIntegrator() = default;

  /**
   * @brief Update voxel likelihoods given an observation of a label
   * @param label Observed input label
   * @param weight Reconstruction weight assigned to the measurement for the voxel
   * @param voxel Semantic voxel to update
   */
  virtual void updateLikelihoods(uint32_t label,
                                 float weight,
                                 SemanticVoxel& voxel) const = 0;
};

/*
 * @brief Full maximum-likelihood integrator
 *
 * Based roughly on Kimera-Semantics and https://arxiv.org/pdf/1609.05130
 */
class MLESemanticIntegrator : public SemanticIntegrator {
 public:
  struct Config {
    //! @brief Measurement probability
    double label_confidence = 0.9;
  } const config;

  explicit MLESemanticIntegrator(const Config& config);

  void updateLikelihoods(uint32_t label,
                         float weight,
                         SemanticVoxel& voxel) const override;

 protected:
  size_t total_labels_;

  float init_likelihood_;
  Eigen::MatrixXf observation_likelihoods_;
};

void declare_config(MLESemanticIntegrator::Config& config);

class BinarySemanticIntegrator : public hydra::SemanticIntegrator {
 public:
  struct Config {};

  explicit BinarySemanticIntegrator(const Config& /* config */){};

  void updateLikelihoods(uint32_t label,
                         float weight,
                         SemanticVoxel& voxel) const override;
};

void declare_config(BinarySemanticIntegrator::Config& config);

/**
 * @brief Integrator that tracks up to K labels
 *
 * Rough implementation of a truncated likelihood integrator.
 */
class FirstKSemanticIntegrator : public SemanticIntegrator {
 public:
  struct Config {
    //! Maximum number of classes to track
    size_t k = 5;
    //! Minimum weight for a class to be valid
    float min_weight = 0.0f;
    //! Maximum weight for any class
    float max_weight = 0.0f;
  } const config;

  explicit FirstKSemanticIntegrator(const Config& config);

  void updateLikelihoods(uint32_t label,
                         float weight,
                         SemanticVoxel& voxel) const override;
};

void declare_config(FirstKSemanticIntegrator::Config& config);

/**
 * @brief Integrator that tracks the single most-likely estimate
 *
 * based on: https://ieeexplore.ieee.org/abstract/document/8967890
 */
class SingleLabelIntegrator : public SemanticIntegrator {
 public:
  struct Config {
  } const config;

  explicit SingleLabelIntegrator(const Config& config = {});

  void updateLikelihoods(uint32_t label,
                         float weight,
                         SemanticVoxel& voxel) const override;
};

void declare_config(SingleLabelIntegrator::Config& config);

}  // namespace hydra
