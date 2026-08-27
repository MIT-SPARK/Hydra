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
#include "hydra/places/semantic_traversability_integrator.h"

#include <config_utilities/config.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

#include <algorithm>
#include <cmath>
#include <future>
#include <limits>

#include "hydra/common/global_info.h"
#include "hydra/reconstruction/index_getter.h"

namespace hydra::places {
namespace {

static const auto registration_projective =
    config::RegistrationWithConfig<SemanticTraversabilityIntegrator,
                                   ProjectiveSemanticTraversabilityIntegrator,
                                   ProjectiveSemanticTraversabilityIntegrator::Config>(
        "ProjectiveSemanticTraversabilityIntegrator");

static const auto registration_table =
    config::RegistrationWithConfig<TraversabilityPrior,
                                   LabelTablePrior,
                                   LabelTablePrior::Config>("LabelTablePrior");

static const auto registration_set =
    config::RegistrationWithConfig<TraversabilityPrior,
                                   LabelSetPrior,
                                   LabelSetPrior::Config>("LabelSetPrior");

//! Keep probabilities strictly inside (0, 1) so that downstream log-odds style updates
//! in any SemanticIntegrator remain finite.
constexpr float kMinProbability = 1.0e-4f;
constexpr float kMaxProbability = 1.0f - kMinProbability;

float clampProbability(float p) {
  return std::clamp(p, kMinProbability, kMaxProbability);
}

}  // namespace

// ---------------------------------------------------------------------------------
// Priors
// ---------------------------------------------------------------------------------

LabelTablePrior::LabelTablePrior(const Config& config)
    : config(config::checkValid(config)) {}

std::optional<float> LabelTablePrior::probability(uint32_t label) const {
  const auto iter = config.label_traversability.find(label);
  if (iter != config.label_traversability.end()) {
    return clampProbability(iter->second);
  }

  if (config.ignore_unknown_labels) {
    return std::nullopt;
  }

  return clampProbability(config.default_traversability);
}

void declare_config(LabelTablePrior::Config& config) {
  using namespace config;
  name("LabelTablePrior::Config");
  field(config.label_traversability, "label_traversability");
  field(config.default_traversability, "default_traversability");
  field(config.ignore_unknown_labels, "ignore_unknown_labels");
  checkInRange(config.default_traversability, 0.0f, 1.0f, "default_traversability");
}

LabelSetPrior::LabelSetPrior(const Config& config)
    : config(config::checkValid(config)) {}

std::optional<float> LabelSetPrior::probability(uint32_t label) const {
  if (config.traversable_labels.count(label)) {
    return clampProbability(config.confidence);
  }

  if (config.untraversable_labels.count(label)) {
    return clampProbability(1.0f - config.confidence);
  }

  return std::nullopt;
}

void declare_config(LabelSetPrior::Config& config) {
  using namespace config;
  name("LabelSetPrior::Config");
  field(config.traversable_labels, "traversable_labels");
  field(config.untraversable_labels, "untraversable_labels");
  field(config.confidence, "confidence");
  checkInRange(config.confidence, 0.5f, 1.0f, "confidence");
}

// ---------------------------------------------------------------------------------
// Projective integrator
// ---------------------------------------------------------------------------------

void declare_config(ProjectiveSemanticTraversabilityIntegrator::Config& config) {
  using namespace config;
  using FusionMode = ProjectiveSemanticTraversabilityIntegrator::FusionMode;
  name("ProjectiveSemanticTraversabilityIntegrator::Config");
  field(config.max_surface_distance, "max_surface_distance", "m");
  field(config.min_incidence_cosine, "min_incidence_cosine");
  field(config.min_measurement_weight, "min_measurement_weight");
  field(config.max_weight, "max_weight");
  field(config.weight_half, "weight_half");
  enum_field(config.fusion_mode,
             "fusion_mode",
             {{FusionMode::MIN, "MIN"},
              {FusionMode::PRODUCT, "PRODUCT"},
              {FusionMode::WEIGHTED_MEAN, "WEIGHTED_MEAN"}});
  field(config.num_threads, "num_threads");
  field(config.interpolation_method, "interpolation_method");
  field(config.semantic_integrator, "semantic_integrator");
  field(config.prior, "prior");

  checkCondition(config.max_surface_distance != 0.0f,
                 "'max_surface_distance' must be nonzero");
  checkInRange(config.min_incidence_cosine, 0.0f, 1.0f, "min_incidence_cosine");
  check(config.max_weight, GT, 0.0f, "max_weight");
  check(config.weight_half, GT, 0.0f, "weight_half");
  check(config.min_measurement_weight, GE, 0.0f, "min_measurement_weight");
  check(config.num_threads, GT, 0, "num_threads");
  checkCondition(config.prior.isSet(), "'prior' must be specified");
}

ProjectiveSemanticTraversabilityIntegrator::
    ProjectiveSemanticTraversabilityIntegrator(const Config& config)
    : config(config::checkValid(config)),
      interpolator_(config.interpolation_method.create()),
      semantic_integrator_(config.semantic_integrator.create()),
      prior_(config.prior.create()) {}

void ProjectiveSemanticTraversabilityIntegrator::initialize(
    const TraversabilityLayer& layer) {
  if (state_) {
    return;
  }

  state_ = std::make_unique<SemanticTraversabilityLayer>(layer.voxel_size,
                                                         layer.voxels_per_side);
  // Negative values are multiples of the voxel size, mirroring ProjectiveIntegrator's
  // convention for weight_dropoff_epsilon and extra_integration_distance.
  max_surface_distance_m_ = config.max_surface_distance < 0.0f
                                ? -config.max_surface_distance * layer.voxel_size
                                : config.max_surface_distance;
}

void ProjectiveSemanticTraversabilityIntegrator::pruneArchived(
    const TraversabilityLayer& layer) {
  BlockIndices to_remove;
  for (const auto& index : state_->allocatedBlockIndices()) {
    if (!layer.hasBlock(index)) {
      to_remove.push_back(index);
    }
  }

  state_->removeBlocks(to_remove);
}

ProjectiveSemanticTraversabilityIntegrator::CellMeasurement
ProjectiveSemanticTraversabilityIntegrator::getCellMeasurement(
    const InputData& data, const Point& p_S, float incidence) const {
  CellMeasurement measurement;

  const auto range = p_S.norm();
  if (!data.inRange(range)) {
    return measurement;
  }

  // Reject grazing views before doing any image work.
  if (incidence < config.min_incidence_cosine) {
    return measurement;
  }

  // Equivalent to ProjectiveIntegrator::interpolatePoint.
  float u, v;
  if (!data.getSensor().projectPointToImagePlane(p_S, u, v)) {
    return measurement;
  }

  measurement.interpolation_weights =
      interpolator_->computeWeights(u, v, data.range_image);
  if (!measurement.interpolation_weights.valid) {
    return measurement;
  }

  // Visibility gate. This is the floor-cell analogue of ProjectiveIntegrator::computeSDF
  // plus the truncation band check:
  //   sdf < 0 -> something is in front of the cell, so the label belongs to an occluder
  //   sdf > 0 -> the measured surface is behind the cell, so we are seeing through it
  // Only |sdf| close to zero means this pixel is actually looking at this floor cell.
  const auto d_surface = interpolator_->interpolateRange(
      data.range_image, measurement.interpolation_weights);
  const auto sdf = d_surface - range;
  if (!std::isfinite(sdf) || std::abs(sdf) > max_surface_distance_m_) {
    return measurement;
  }

  measurement.label =
      interpolator_->interpolateID(data.label_image, measurement.interpolation_weights);

  // Same shape as ProjectiveIntegrator::computeWeight, without the TSDF-specific
  // dropoff behind the surface, plus a foreshortening term: a grazing ray spreads one
  // pixel across many floor cells and should count for less.
  const auto& sensor = data.getSensor();
  const auto depth = sensor.getPointDepth(p_S);
  if (depth <= 0.0f) {
    return measurement;
  }

  auto weight = sensor.computeRayDensity(state_->voxel_size, depth);
  weight /= std::pow(depth, 2.0f);
  weight *= incidence;
  if (!std::isfinite(weight) || weight < config.min_measurement_weight) {
    return measurement;
  }

  measurement.weight = weight;
  measurement.valid = true;
  return measurement;
}

void ProjectiveSemanticTraversabilityIntegrator::updateCell(
    const CellMeasurement& measurement, SemanticTraversabilityCell& cell) const {
  if (!measurement.valid || measurement.label < 0) {
    return;
  }

  // Only accumulate labels the prior has an opinion about, so that unknown classes do
  // not dilute the belief.
  if (!prior_->probability(static_cast<uint32_t>(measurement.label))) {
    return;
  }

  semantic_integrator_->updateLikelihoods(
      static_cast<uint32_t>(measurement.label), measurement.weight, cell.semantics);
  cell.weight = std::min(cell.weight + measurement.weight, config.max_weight);
}

std::optional<std::pair<float, float>>
ProjectiveSemanticTraversabilityIntegrator::evaluate(
    const SemanticTraversabilityCell& cell) const {
  if (cell.semantics.empty || cell.weight <= 0.0f) {
    return std::nullopt;
  }

  const auto probability = prior_->probability(cell.semantics.semantic_label);
  if (!probability) {
    return std::nullopt;
  }

  // Saturating confidence: weight_half is the accumulated weight at which we are half
  // confident, which is a single interpretable knob.
  const auto confidence = cell.weight / (cell.weight + config.weight_half);
  return std::make_pair(*probability, confidence);
}

void ProjectiveSemanticTraversabilityIntegrator::applyToVoxel(
    const SemanticTraversabilityCell& cell, TraversabilityVoxel& voxel) const {
  const auto result = evaluate(cell);
  if (!result) {
    return;  // no semantic information: leave the geometric estimate untouched
  }

  const auto p_semantic = result->first;
  const auto confidence_semantic = result->second;

  switch (config.fusion_mode) {
    case FusionMode::MIN:
      // Semantics may veto a geometrically flat but undrivable surface (water, glass)
      // but can never override a geometric obstacle.
      voxel.traversability = std::min(voxel.traversability, p_semantic);
      break;
    case FusionMode::PRODUCT:
      voxel.traversability *= p_semantic;
      break;
    case FusionMode::WEIGHTED_MEAN: {
      const auto w_geometric = voxel.confidence;
      const auto w_semantic = confidence_semantic;
      const auto total = w_geometric + w_semantic;
      if (total > 0.0f) {
        voxel.traversability =
            (voxel.traversability * w_geometric + p_semantic * w_semantic) / total;
      }
      break;
    }
  }

  voxel.confidence = std::max(voxel.confidence, confidence_semantic);
}

void ProjectiveSemanticTraversabilityIntegrator::updateBlock(
    const BlockIndex& block_index,
    const InputData& data,
    const Eigen::Isometry3f& sensor_T_world,
    const Point& sensor_p_world,
    const TraversabilityLayer& layer) {
  const auto geometry_block = layer.getBlockPtr(block_index);
  if (!geometry_block) {
    return;
  }

  // NOTE: the state block must already exist. Allocation mutates the layer's hash map
  // and therefore cannot happen inside the parallel section.
  const auto state_block_ptr = state_->getBlockPtr(block_index);
  if (!state_block_ptr) {
    return;
  }

  auto& state_block = *state_block_ptr;
  const auto voxels_per_side = static_cast<int>(state_->voxels_per_side);
  const auto voxel_size = state_->voxel_size;

  for (int x = 0; x < voxels_per_side; ++x) {
    for (int y = 0; y < voxels_per_side; ++y) {
      const auto& geometry_voxel = geometry_block->voxel(x, y);
      if (!geometry_voxel.height) {
        continue;  // no surface found for this column in the current frame
      }

      // Lift the cell to a world point. The +0.5 offsets are required because
      // spatial_hash uses the cell-center convention (see spatial_hash/grid.h).
      const auto global = geometry_block->globalFromLocalIndex(Index2D(x, y));
      const Point p_world((global.x() + 0.5f) * voxel_size,
                          (global.y() + 0.5f) * voxel_size,
                          *geometry_voxel.height);

      // Incidence relative to the vertical, computed in the world frame where the floor
      // normal is simply +z.
      const auto ray_world = p_world - sensor_p_world;
      const auto ray_norm = ray_world.norm();
      if (ray_norm <= 0.0f) {
        continue;
      }

      const auto incidence = std::abs(ray_world.z()) / ray_norm;
      const auto measurement =
          getCellMeasurement(data, sensor_T_world * p_world, incidence);
      if (!measurement.valid) {
        continue;
      }

      updateCell(measurement, state_block.voxel(x, y));
      state_block.updated = true;
    }
  }
}

void ProjectiveSemanticTraversabilityIntegrator::applyToLayer(
    TraversabilityLayer& layer, const Classifier& reclassify) const {
  for (auto& geometry_block : layer) {
    const auto state_block = state_->getBlockPtr(geometry_block.index);
    if (!state_block) {
      continue;
    }

    for (size_t i = 0; i < geometry_block.voxels.size(); ++i) {
      auto& voxel = geometry_block.voxels[i];
      applyToVoxel(state_block->voxels[i], voxel);
      if (reclassify) {
        reclassify(voxel);
      }
    }
  }
}

void ProjectiveSemanticTraversabilityIntegrator::updateTraversability(
    const ActiveWindowOutput& msg,
    TraversabilityLayer& layer,
    const Classifier& reclassify) {
  if (!msg.sensor_data) {
    LOG_FIRST_N(WARNING, 5)
        << "No sensor data in active window output; skipping semantic traversability.";
    return;
  }

  const auto& data = *msg.sensor_data;
  if (data.label_image.empty()) {
    LOG_FIRST_N(WARNING, 5) << "No label image in input data; semantic traversability "
                               "requires a semantic input source.";
    return;
  }

  initialize(layer);
  pruneArchived(layer);

  const auto world_T_sensor = data.getSensorPose().cast<float>();
  const Eigen::Isometry3f sensor_T_world = world_T_sensor.inverse();
  const Point sensor_p_world = world_T_sensor.translation();

  const auto block_indices = layer.allocatedBlockIndices();

  // Allocate all state blocks up front: the parallel section below only reads the
  // layer's block map, so every block it touches must already be present.
  for (const auto& index : block_indices) {
    state_->allocateBlock(index, state_->voxels_per_side);
  }

  if (config.num_threads <= 1) {
    for (const auto& index : block_indices) {
      updateBlock(index, data, sensor_T_world, sensor_p_world, layer);
    }
  } else {
    IndexGetter<BlockIndex> index_getter(block_indices);
    std::vector<std::future<void>> threads;
    for (int i = 0; i < config.num_threads; ++i) {
      threads.emplace_back(std::async(std::launch::async, [&]() {
        BlockIndex block_index;
        while (index_getter.getNextIndex(block_index)) {
          updateBlock(block_index, data, sensor_T_world, sensor_p_world, layer);
        }
      }));
    }

    for (auto& thread : threads) {
      thread.get();
    }
  }

  applyToLayer(layer, reclassify);
}

}  // namespace hydra::places
