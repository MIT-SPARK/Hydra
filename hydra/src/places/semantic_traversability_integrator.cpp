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

namespace hydra::places {
namespace {

static const auto registration_projective =
    config::RegistrationWithConfig<PersistentTraversabilityProcessor,
                                   ProjectiveSemanticTraversabilityIntegrator,
                                   ProjectiveSemanticTraversabilityIntegrator::Config>(
        "ProjectiveSemanticTraversabilityIntegrator");

}  // namespace

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
  field(config.interpolation_method, "interpolation_method");
  field(config.label_traversability, "label_traversability");

  checkCondition(config.max_surface_distance != 0.0f,
                 "'max_surface_distance' must be nonzero");
  checkInRange(config.min_incidence_cosine, 0.0f, 1.0f, "min_incidence_cosine");
  check(config.min_measurement_weight, GE, 0.0f, "min_measurement_weight");
  check(config.max_weight, GT, 0.0f, "max_weight");
  check(config.weight_half, GT, 0.0f, "weight_half");
  checkCondition(!config.label_traversability.empty(),
                 "'label_traversability' must be specified");
  for (const auto& [label, probability] : config.label_traversability) {
    checkInRange(probability, 0.0f, 1.0f, "label_traversability");
  }
}

ProjectiveSemanticTraversabilityIntegrator::ProjectiveSemanticTraversabilityIntegrator(
    const Config& config)
    : config(config::checkValid(config)),
      interpolator_(config.interpolation_method.create()) {}

ProjectiveSemanticTraversabilityIntegrator::CellMeasurement
ProjectiveSemanticTraversabilityIntegrator::getCellMeasurement(const InputData& data,
                                                               const Point& p_S,
                                                               float incidence,
                                                               float voxel_size) const {
  CellMeasurement measurement;

  const auto range = p_S.norm();
  if (!data.inRange(range)) {
    return measurement;
  }

  // Reject grazing views before doing any image work.
  if (incidence < config.min_incidence_cosine) {
    return measurement;
  }

  const auto& sensor = data.getSensor();
  float u, v;
  if (!sensor.projectPointToImagePlane(p_S, u, v)) {
    return measurement;
  }

  measurement.interpolation_weights =
      interpolator_->computeWeights(u, v, data.range_image);
  if (!measurement.interpolation_weights.valid) {
    return measurement;
  }

  // Visibility gate, the floor-cell analogue of the TSDF truncation band:
  //   sdf < 0 -> something is in front of the cell, so the label belongs to an occluder
  //   sdf > 0 -> the measured surface is behind the cell, so we are seeing through it
  // Only |sdf| close to zero means this pixel is actually looking at this floor cell.
  const auto measured_range = interpolator_->interpolateRange(
      data.range_image, measurement.interpolation_weights);
  const auto sdf = measured_range - range;
  if (!std::isfinite(sdf) || std::abs(sdf) > max_surface_distance_m_) {
    return measurement;
  }

  // Same shape as ProjectiveIntegrator::computeWeight without the dropoff behind the
  // surface, plus a foreshortening term: a grazing ray spreads one pixel across many
  // floor cells and should count for less.
  const auto depth = sensor.getPointDepth(p_S);
  if (depth <= 0.0f) {
    return measurement;
  }

  auto weight = sensor.computeRayDensity(voxel_size, depth) / (depth * depth);
  weight *= incidence;
  if (!std::isfinite(weight) || weight < config.min_measurement_weight) {
    return measurement;
  }

  measurement.label =
      interpolator_->interpolateID(data.label_image, measurement.interpolation_weights);
  measurement.weight = weight;
  measurement.valid = true;
  return measurement;
}

void ProjectiveSemanticTraversabilityIntegrator::updateCell(
    const CellMeasurement& measurement, TraversabilityVoxel& voxel) const {
  if (!measurement.valid || measurement.label < 0) {
    return;
  }

  // Unlisted labels are skipped so they do not dilute the belief.
  const auto iter =
      config.label_traversability.find(static_cast<uint32_t>(measurement.label));
  if (iter == config.label_traversability.end()) {
    return;
  }

  // Weighted running mean of the observed traversability probabilities.
  const auto weight = voxel.semantic_weight;
  const auto total = weight + measurement.weight;
  voxel.semantic_traversability =
      (voxel.semantic_traversability * weight + iter->second * measurement.weight) /
      total;
  voxel.semantic_weight = std::min(total, config.max_weight);
}

std::optional<std::pair<float, float>>
ProjectiveSemanticTraversabilityIntegrator::evaluate(
    const TraversabilityVoxel& voxel) const {
  if (voxel.semantic_weight <= 0.0f) {
    return std::nullopt;
  }

  // Saturating confidence: weight_half is the accumulated weight at half confidence.
  const auto confidence =
      voxel.semantic_weight / (voxel.semantic_weight + config.weight_half);
  return std::make_pair(voxel.semantic_traversability, confidence);
}

void ProjectiveSemanticTraversabilityIntegrator::applyToVoxel(
    TraversabilityVoxel& voxel) const {
  const auto result = evaluate(voxel);
  if (!result) {
    return;  // No semantic information: leave the geometric estimate untouched.
  }

  const auto [p_semantic, confidence_semantic] = *result;
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
      const auto total = voxel.confidence + confidence_semantic;
      if (total > 0.0f) {
        voxel.traversability = (voxel.traversability * voxel.confidence +
                                p_semantic * confidence_semantic) /
                               total;
      }
      break;
    }
  }

  voxel.confidence = std::max(voxel.confidence, confidence_semantic);
}

void ProjectiveSemanticTraversabilityIntegrator::updateBlock(
    const InputData& data,
    const Eigen::Isometry3f& sensor_T_world,
    const Point& sensor_p_world,
    float voxel_size,
    TraversabilityBlock& block) const {
  const auto voxels_per_side = static_cast<int>(block.voxels_per_side);
  for (int x = 0; x < voxels_per_side; ++x) {
    for (int y = 0; y < voxels_per_side; ++y) {
      auto& voxel = block.voxel(x, y);
      if (!voxel.height) {
        continue;  // No surface found for this column.
      }

      // Lift the cell to a world point. The +0.5 offsets move to the cell center.
      const auto global = block.globalFromLocalIndex(Index2D(x, y));
      const Point p_world((global.x() + 0.5f) * voxel_size,
                          (global.y() + 0.5f) * voxel_size,
                          *voxel.height);

      // Incidence relative to the vertical, in the world frame where the floor normal
      // is simply +z.
      const Point ray_world = p_world - sensor_p_world;
      const auto ray_norm = ray_world.norm();
      if (ray_norm <= 0.0f) {
        continue;
      }

      const auto incidence = std::abs(ray_world.z()) / ray_norm;
      const auto measurement =
          getCellMeasurement(data, sensor_T_world * p_world, incidence, voxel_size);
      updateCell(measurement, voxel);
    }
  }
}

void ProjectiveSemanticTraversabilityIntegrator::integrateFrame(
    const InputData& data, TraversabilityLayer& layer) const {
  const Eigen::Isometry3f world_T_sensor = data.getSensorPose().cast<float>();
  const Eigen::Isometry3f sensor_T_world = world_T_sensor.inverse();
  const Point sensor_p_world = world_T_sensor.translation();
  for (auto& block : layer) {
    updateBlock(data, sensor_T_world, sensor_p_world, layer.voxel_size, block);
  }
}

void ProjectiveSemanticTraversabilityIntegrator::applyToLayer(
    TraversabilityLayer& layer) const {
  for (auto& block : layer) {
    if (!block.updated) {
      continue;
    }

    for (auto& voxel : block.voxels) {
      applyToVoxel(voxel);
    }
  }
}

void ProjectiveSemanticTraversabilityIntegrator::apply(const ActiveWindowOutput& msg,
                                                       TraversabilityLayer& layer) {
  max_surface_distance_m_ = config.max_surface_distance < 0.0f
                                ? -config.max_surface_distance * layer.voxel_size
                                : config.max_surface_distance;

  // Messages can collate several frames (e.g. between keyframes), so integrate all of
  // them before applying the result once.
  bool integrated = false;
  for (const auto& data : msg.sensor_data) {
    if (!data || data->label_image.empty()) {
      continue;
    }

    integrateFrame(*data, layer);
    integrated = true;
  }

  if (!integrated) {
    LOG_FIRST_N(WARNING, 5) << "No label image in input data; semantic traversability "
                               "requires a semantic input source.";
  }

  // Blocks recomputed by the estimator this update lost their fused traversability, so
  // re-apply the accumulated semantics even if no new labels were observed.
  applyToLayer(layer);
}

}  // namespace hydra::places
