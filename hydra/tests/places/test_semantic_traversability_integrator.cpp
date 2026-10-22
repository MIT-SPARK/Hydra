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

namespace hydra::places {

namespace {

using Integrator = ProjectiveSemanticTraversabilityIntegrator;
using FusionMode = Integrator::FusionMode;

Integrator::Config makeConfig(FusionMode mode) {
  Integrator::Config config;
  config.label_traversability = {{1, 0.9f}, {2, 0.1f}};
  config.fusion_mode = mode;
  config.weight_half = 1.0f;
  config.max_weight = 5.0f;
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

TEST(SemanticTraversability, UpdateCell) {
  const Integrator integrator(makeConfig(FusionMode::MIN));

  // Running mean of the label probabilities, weighted by the measurement weights.
  TraversabilityVoxel voxel;
  integrator.updateCell(makeMeasurement(1, 1.0f), voxel);
  EXPECT_NEAR(voxel.semantic_traversability, 0.9f, 1.0e-6f);
  EXPECT_NEAR(voxel.semantic_weight, 1.0f, 1.0e-6f);

  integrator.updateCell(makeMeasurement(2, 3.0f), voxel);
  EXPECT_NEAR(voxel.semantic_traversability, (0.9f + 3.0f * 0.1f) / 4.0f, 1.0e-6f);
  EXPECT_NEAR(voxel.semantic_weight, 4.0f, 1.0e-6f);

  // Unlisted labels and invalid measurements are ignored.
  const auto before = voxel;
  integrator.updateCell(makeMeasurement(3, 1.0f), voxel);
  auto invalid = makeMeasurement(1, 1.0f);
  invalid.valid = false;
  integrator.updateCell(invalid, voxel);
  EXPECT_EQ(voxel.semantic_traversability, before.semantic_traversability);
  EXPECT_EQ(voxel.semantic_weight, before.semantic_weight);

  // Weight saturates at max_weight.
  integrator.updateCell(makeMeasurement(1, 10.0f), voxel);
  EXPECT_NEAR(voxel.semantic_weight, 5.0f, 1.0e-6f);
}

TEST(SemanticTraversability, ResetPreservesSemantics) {
  auto voxel = makeVoxel(0.7f, 0.8f);
  voxel.height = 1.0f;
  voxel.state = spark_dsg::TraversabilityState::TRAVERSABLE;
  voxel.semantic_traversability = 0.2f;
  voxel.semantic_weight = 3.0f;

  voxel.resetGeometry();
  EXPECT_EQ(voxel, TraversabilityVoxel());
  EXPECT_FALSE(voxel.height);
  EXPECT_EQ(voxel.semantic_traversability, 0.2f);
  EXPECT_EQ(voxel.semantic_weight, 3.0f);
}

TEST(SemanticTraversability, FusionModes) {
  TraversabilityVoxel traversable;
  TraversabilityVoxel untraversable;
  {
    const Integrator integrator(makeConfig(FusionMode::MIN));
    integrator.updateCell(makeMeasurement(1, 1.0f), traversable);
    integrator.updateCell(makeMeasurement(2, 1.0f), untraversable);
  }

  const auto withSemantics =
      [](float traversability, float confidence, const TraversabilityVoxel& semantics) {
        auto voxel = makeVoxel(traversability, confidence);
        voxel.semantic_traversability = semantics.semantic_traversability;
        voxel.semantic_weight = semantics.semantic_weight;
        return voxel;
      };

  // MIN: semantics can lower but never raise traversability.
  {
    const Integrator integrator(makeConfig(FusionMode::MIN));
    auto voxel = withSemantics(0.5f, 0.2f, traversable);
    integrator.applyToVoxel(voxel);
    EXPECT_NEAR(voxel.traversability, 0.5f, 1.0e-6f);
    EXPECT_NEAR(voxel.confidence, 0.5f, 1.0e-6f);

    voxel = withSemantics(0.5f, 0.2f, untraversable);
    integrator.applyToVoxel(voxel);
    EXPECT_NEAR(voxel.traversability, 0.1f, 1.0e-6f);
  }

  // PRODUCT: multiplies the probabilities.
  {
    const Integrator integrator(makeConfig(FusionMode::PRODUCT));
    auto voxel = withSemantics(0.5f, 0.8f, traversable);
    integrator.applyToVoxel(voxel);
    EXPECT_NEAR(voxel.traversability, 0.45f, 1.0e-6f);
    EXPECT_NEAR(voxel.confidence, 0.8f, 1.0e-6f);
  }

  // WEIGHTED_MEAN: semantics can raise traversability.
  {
    const Integrator integrator(makeConfig(FusionMode::WEIGHTED_MEAN));
    auto voxel = withSemantics(0.5f, 0.5f, traversable);
    integrator.applyToVoxel(voxel);
    EXPECT_NEAR(voxel.traversability, 0.7f, 1.0e-6f);
  }

  // Unobserved voxels leave the geometric estimate untouched.
  {
    const Integrator integrator(makeConfig(FusionMode::PRODUCT));
    auto voxel = makeVoxel(0.5f, 0.2f);
    integrator.applyToVoxel(voxel);
    EXPECT_EQ(voxel, makeVoxel(0.5f, 0.2f));
  }
}

TEST(SemanticTraversability, OnlyUpdatedBlocksAreFused) {
  Integrator integrator(makeConfig(FusionMode::PRODUCT));

  TraversabilityLayer layer(0.1f, 2);
  auto& fresh = layer.allocateBlock(BlockIndex(0, 0, 0), 2);
  auto& stale = layer.allocateBlock(BlockIndex(1, 0, 0), 2);
  for (auto* block : {&fresh, &stale}) {
    for (auto& voxel : block->voxels) {
      voxel = makeVoxel(0.5f, 0.5f);
      integrator.updateCell(makeMeasurement(1, 1.0f), voxel);
    }
  }

  // Repeated updates without labels: only blocks the estimator recomputed (marked
  // updated) get the semantics applied, and stale blocks are never fused again.
  // Input data without a label image (the sensor is never accessed).
  const ActiveWindowOutput msg(std::make_shared<InputData>(nullptr));
  for (int i = 0; i < 3; ++i) {
    // Mimic the estimator: reset the geometry, keeping the semantics.
    for (auto& voxel : fresh.voxels) {
      voxel.resetGeometry();
      voxel.traversability = 0.5f;
      voxel.confidence = 0.5f;
    }
    fresh.updated = true;
    stale.updated = false;
    integrator.apply(msg, layer);
  }

  // Fused once per recompute: 0.5 * 0.9 = 0.45.
  for (const auto& voxel : fresh.voxels) {
    EXPECT_NEAR(voxel.traversability, 0.45f, 1.0e-6f);
  }
  for (const auto& voxel : stale.voxels) {
    EXPECT_NEAR(voxel.traversability, 0.5f, 1.0e-6f);
  }
}

}  // namespace hydra::places
