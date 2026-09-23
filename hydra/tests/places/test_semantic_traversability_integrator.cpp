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
#include <hydra/places/semantic_traversability_integrator.h>

#include "hydra_test/config_guard.h"

namespace hydra::places {

namespace {

using Integrator = ProjectiveSemanticTraversabilityIntegrator;
using FusionMode = Integrator::FusionMode;

// Exposes the protected state so tests can seed semantics without a sensor.
struct TestIntegrator : public Integrator {
  explicit TestIntegrator(const Config& config) : Integrator(config) {}
  using Integrator::applyToLayer;
  using Integrator::initialize;
  using Integrator::state_;
};

Integrator::Config makeConfig(FusionMode mode) {
  LabelSetPrior::Config prior;
  prior.traversable_labels = {1};
  prior.untraversable_labels = {2};
  prior.confidence = 0.9f;

  Integrator::Config config;
  config.fusion_mode = mode;
  config.weight_half = 1.0f;
  config.max_weight = 5.0f;
  config.prior = prior;
  return config;
}

Integrator::CellMeasurement makeMeasurement(int32_t label, float weight) {
  Integrator::CellMeasurement measurement;
  measurement.valid = true;
  measurement.label = label;
  measurement.weight = weight;
  return measurement;
}

TraversabilityVoxel makeVoxel(float traversability, float confidence) {
  TraversabilityVoxel voxel;
  voxel.traversability = traversability;
  voxel.confidence = confidence;
  return voxel;
}

}  // namespace

TEST(SemanticTraversability, LabelTablePrior) {
  LabelTablePrior::Config config;
  config.label_traversability = {{1, 0.8f}, {2, 0.0f}, {3, 1.0f}};
  config.default_traversability = 0.3f;

  const LabelTablePrior ignoring(config);
  EXPECT_NEAR(*ignoring.probability(1), 0.8f, 1.0e-6f);
  // Extremes are clamped to stay strictly inside (0, 1).
  EXPECT_GT(*ignoring.probability(2), 0.0f);
  EXPECT_LT(*ignoring.probability(3), 1.0f);
  EXPECT_FALSE(ignoring.probability(4));

  config.ignore_unknown_labels = false;
  const LabelTablePrior defaulting(config);
  EXPECT_NEAR(*defaulting.probability(4), 0.3f, 1.0e-6f);
}

TEST(SemanticTraversability, LabelSetPrior) {
  LabelSetPrior::Config config;
  config.traversable_labels = {1};
  config.untraversable_labels = {2};
  config.confidence = 0.8f;
  const LabelSetPrior prior(config);
  EXPECT_NEAR(*prior.probability(1), 0.8f, 1.0e-6f);
  EXPECT_NEAR(*prior.probability(2), 0.2f, 1.0e-6f);
  EXPECT_FALSE(prior.probability(3));
}

TEST(SemanticTraversability, UpdateAndEvaluateCell) {
  const auto guard = test::ConfigGuard::FixedLabels(5);
  const Integrator integrator(makeConfig(FusionMode::MIN));

  SemanticTraversabilityCell cell;
  EXPECT_FALSE(integrator.evaluate(cell));

  // Invalid measurements and labels the prior knows nothing about are not accumulated.
  auto invalid = makeMeasurement(1, 1.0f);
  invalid.valid = false;
  integrator.updateCell(invalid, cell);
  integrator.updateCell(makeMeasurement(3, 1.0f), cell);
  integrator.updateCell(makeMeasurement(-1, 1.0f), cell);
  EXPECT_TRUE(cell.semantics.empty);
  EXPECT_EQ(cell.weight, 0.0f);

  integrator.updateCell(makeMeasurement(2, 1.0f), cell);
  auto result = integrator.evaluate(cell);
  ASSERT_TRUE(result);
  EXPECT_NEAR(result->first, 0.1f, 1.0e-6f);
  EXPECT_NEAR(result->second, 0.5f, 1.0e-6f);  // w / (w + weight_half)

  // Weight saturates at max_weight.
  for (int i = 0; i < 10; ++i) {
    integrator.updateCell(makeMeasurement(2, 1.0f), cell);
  }
  EXPECT_EQ(cell.weight, 5.0f);
  result = integrator.evaluate(cell);
  ASSERT_TRUE(result);
  EXPECT_NEAR(result->second, 5.0f / 6.0f, 1.0e-6f);
}

TEST(SemanticTraversability, FusionModes) {
  const auto guard = test::ConfigGuard::FixedLabels(5);

  SemanticTraversabilityCell traversable;
  SemanticTraversabilityCell untraversable;
  {
    const Integrator integrator(makeConfig(FusionMode::MIN));
    integrator.updateCell(makeMeasurement(1, 1.0f), traversable);
    integrator.updateCell(makeMeasurement(2, 1.0f), untraversable);
  }

  // MIN: semantics can lower but never raise traversability.
  {
    const Integrator integrator(makeConfig(FusionMode::MIN));
    auto voxel = makeVoxel(0.5f, 0.2f);
    integrator.applyToVoxel(traversable, voxel);
    EXPECT_NEAR(voxel.traversability, 0.5f, 1.0e-6f);
    EXPECT_NEAR(voxel.confidence, 0.5f, 1.0e-6f);

    integrator.applyToVoxel(untraversable, voxel);
    EXPECT_NEAR(voxel.traversability, 0.1f, 1.0e-6f);
  }

  // PRODUCT: multiplies the probabilities.
  {
    const Integrator integrator(makeConfig(FusionMode::PRODUCT));
    auto voxel = makeVoxel(0.5f, 0.8f);
    integrator.applyToVoxel(traversable, voxel);
    EXPECT_NEAR(voxel.traversability, 0.45f, 1.0e-6f);
    EXPECT_NEAR(voxel.confidence, 0.8f, 1.0e-6f);
  }

  // WEIGHTED_MEAN: semantics can raise traversability.
  {
    const Integrator integrator(makeConfig(FusionMode::WEIGHTED_MEAN));
    auto voxel = makeVoxel(0.5f, 0.5f);
    integrator.applyToVoxel(traversable, voxel);
    EXPECT_NEAR(voxel.traversability, 0.7f, 1.0e-6f);
  }

  // Unobserved cells leave the geometric estimate untouched.
  {
    const Integrator integrator(makeConfig(FusionMode::PRODUCT));
    auto voxel = makeVoxel(0.5f, 0.2f);
    integrator.applyToVoxel(SemanticTraversabilityCell(), voxel);
    EXPECT_EQ(voxel, makeVoxel(0.5f, 0.2f));
  }
}

TEST(SemanticTraversability, OnlyUpdatedBlocksAreFused) {
  const auto guard = test::ConfigGuard::FixedLabels(5);
  TestIntegrator integrator(makeConfig(FusionMode::PRODUCT));

  TraversabilityLayer layer(0.1f, 2);
  auto& fresh = layer.allocateBlock(BlockIndex(0, 0, 0), 2);
  auto& stale = layer.allocateBlock(BlockIndex(1, 0, 0), 2);
  for (auto* block : {&fresh, &stale}) {
    for (auto& voxel : block->voxels) {
      voxel = makeVoxel(0.5f, 0.5f);
    }
  }

  // Seed semantic state for both blocks.
  integrator.initialize(layer);
  for (const auto& index : layer.allocatedBlockIndices()) {
    auto& block = integrator.state_->allocateBlock(index, 2);
    for (auto& cell : block.voxels) {
      integrator.updateCell(makeMeasurement(1, 1.0f), cell);
    }
  }

  // Repeated updates without labels: only blocks the estimator recomputed (marked
  // updated) get the semantics applied, and stale blocks are never fused again.
  // Input data without a label image (the sensor is never accessed).
  const ActiveWindowOutput msg(std::make_shared<InputData>(nullptr));
  int num_classified = 0;
  const auto classify = [&](TraversabilityVoxel&) { ++num_classified; };
  for (int i = 0; i < 3; ++i) {
    for (auto& voxel : fresh.voxels) {
      voxel = makeVoxel(0.5f, 0.5f);  // Mimic the estimator's reset.
    }
    fresh.updated = true;
    stale.updated = false;
    integrator.updateTraversability(msg, layer, classify);
  }

  EXPECT_EQ(num_classified, 3 * 4);
  for (const auto& voxel : fresh.voxels) {
    EXPECT_NEAR(voxel.traversability, 0.45f, 1.0e-6f);
  }
  for (const auto& voxel : stale.voxels) {
    EXPECT_NEAR(voxel.traversability, 0.5f, 1.0e-6f);
  }
}

}  // namespace hydra::places
