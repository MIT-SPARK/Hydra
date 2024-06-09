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
#include <gtest/gtest.h>
#include <hydra/common/global_info.h>
#include <hydra/reconstruction/semantic_integrator.h>

#include "hydra_test/config_guard.h"

namespace hydra {

std::unique_ptr<SemanticIntegrator> createIntegrator(
    size_t num_labels,
    const std::set<uint32_t>& dynamic_labels = {},
    const std::set<uint32_t>& invalid_labels = {},
    double label_confidence = 0.9) {
  PipelineConfig pipeline_config;
  pipeline_config.label_space.total_labels = num_labels;
  pipeline_config.label_space.dynamic_labels = dynamic_labels;
  pipeline_config.label_space.invalid_labels = invalid_labels;
  GlobalInfo::init(pipeline_config);

  MLESemanticIntegrator::Config config;
  config.label_confidence = label_confidence;
  return std::make_unique<MLESemanticIntegrator>(config);
}

TEST(SemanticIntegrator, MLEValidLabelCorrect) {
  test::ConfigGuard guard(false);
  const auto integrator = createIntegrator(5, {1}, {4});
  EXPECT_FALSE(integrator->isValidLabel(1));
  EXPECT_FALSE(integrator->isValidLabel(4));
  EXPECT_FALSE(integrator->isValidLabel(5));
  EXPECT_TRUE(integrator->isValidLabel(0));
  EXPECT_TRUE(integrator->isValidLabel(2));
}

TEST(SemanticIntegrator, MLEIntegrationCorrect) {
  test::ConfigGuard guard(false);
  const auto integrator = createIntegrator(5, {}, {}, 0.8);
  SemanticVoxel voxel;
  EXPECT_TRUE(voxel.empty);
  integrator->updateLikelihoods(2, voxel);
  EXPECT_FALSE(voxel.empty);
  ASSERT_EQ(voxel.semantic_likelihoods.size(), 5);
  EXPECT_GT(voxel.semantic_likelihoods(2), voxel.semantic_likelihoods(0));
  EXPECT_GT(voxel.semantic_likelihoods(2), voxel.semantic_likelihoods(1));
  EXPECT_GT(voxel.semantic_likelihoods(2), voxel.semantic_likelihoods(3));
  EXPECT_GT(voxel.semantic_likelihoods(2), voxel.semantic_likelihoods(4));
  Eigen::VectorXf expected = Eigen::VectorXf::Zero(5);
  expected << 0.04, 0.04, 0.16, 0.04, 0.04;
  Eigen::VectorXf result_prob = voxel.semantic_likelihoods.array().exp();
  EXPECT_TRUE(result_prob.isApprox(expected));
}

}  // namespace hydra
