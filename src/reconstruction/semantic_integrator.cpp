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
#include "hydra/reconstruction/semantic_integrator.h"

#include <config_utilities/config.h>
#include <glog/logging.h>

#include <cmath>
#include <cstdint>

#include "hydra/common/global_info.h"

namespace hydra {

MLESemanticIntegrator::MLESemanticIntegrator(const Config& config) : config(config) {
  total_labels_ = GlobalInfo::instance().getTotalLabels();
  init_likelihood_ = std::log(1.0f / static_cast<float>(total_labels_));

  const auto label_config = GlobalInfo::instance().getLabelSpaceConfig();
  dynamic_labels_ = label_config.dynamic_labels;
  invalid_labels_ = label_config.invalid_labels;

  const auto match_likelihood = std::log(config.label_confidence);
  const auto nonmatch_likelihood = std::log(1.0f - config.label_confidence);
  observation_likelihoods_ =
      Eigen::MatrixXf::Constant(total_labels_, total_labels_, nonmatch_likelihood);
  observation_likelihoods_.diagonal().setConstant(match_likelihood);
}

bool MLESemanticIntegrator::canIntegrate(uint32_t label) const {
  if (dynamic_labels_.count(label)) {
    return false;
  }

  if (invalid_labels_.count(label)) {
    return false;
  }

  return true;
}

bool MLESemanticIntegrator::isValidLabel(uint32_t label) const {
  if (label >= total_labels_) {
    if (label != std::numeric_limits<uint32_t>::max()) {
      LOG_FIRST_N(ERROR, 100) << "Encountered invalid label: " << label << " (warning "
                              << google::COUNTER << " / 100)";
    }
    return false;
  }

  return canIntegrate(label);
}

void MLESemanticIntegrator::updateLikelihoods(uint32_t label,
                                              SemanticVoxel& voxel) const {
  if (voxel.empty) {
    voxel.empty = false;
    voxel.semantic_likelihoods.setConstant(total_labels_, init_likelihood_);
  }
  voxel.semantic_likelihoods += observation_likelihoods_.col(label);
  voxel.semantic_likelihoods.maxCoeff(&voxel.semantic_label);
}

void declare_config(MLESemanticIntegrator::Config& config) {
  using namespace config;
  name("MLESemanticIntegrator::Config");
  field(config.label_confidence, "label_confidence");
  checkInRange(config.label_confidence,
               0.0,
               1.0,
               "label_confidence is valid probability",
               false,
               true);
}

}  // namespace hydra
