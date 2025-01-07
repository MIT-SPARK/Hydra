// The contents of this file are originally from Panoptic-Mapping,
// under the following license:
//
// BSD 3-Clause License
// Copyright (c) 2021, ETHZ ASL
// All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// See https://github.com/ethz-asl/panoptic_mapping for original code and paper
//
// Modifications (including work done by Lukas Schmid for Khronos) fall under the same
// license as Hydra and are subject to the following copyright and disclaimer:
//
// Copyright 2022 Massachusetts Institute of Technology.
// All Rights Reserved
//
// Research was sponsored by the United States Air Force Research Laboratory and
// the United States Air Force Artificial Intelligence Accelerator and was
// accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
// and conclusions contained in this document are those of the authors and should
// not be interpreted as representing the official policies, either expressed or
// implied, of the United States Air Force or the U.S. Government. The U.S.
// Government is authorized to reproduce and distribute reprints for Government
// purposes notwithstanding any copyright notation herein.
#include "hydra/reconstruction/projective_integrator.h"

#include <config_utilities/config.h>
#include <config_utilities/types/conversions.h>
#include <config_utilities/validation.h>

#include <algorithm>
#include <future>
#include <vector>

#include "hydra/input/sensor_utilities.h"
#include "hydra/reconstruction/index_getter.h"
#include "hydra/utils/printing.h"

namespace hydra {

using Measurement = ProjectiveIntegrator::VoxelMeasurement;
using MapConfig = VolumetricMap::Config;

void declare_config(ProjectiveIntegrator::Config& config) {
  using namespace config;
  name("ProjectiveIntegrator");
  field(config.verbosity, "verbosity");
  field(config.extra_integration_distance,
        "extra_integration_distance",
        config.extra_integration_distance >= 0 ? "m" : "vs");
  field(config.use_weight_dropoff, "use_weight_dropoff");
  field(config.weight_dropoff_epsilon,
        "weight_dropoff_epsilon",
        config.weight_dropoff_epsilon >= 0 ? "m" : "vs");
  field(config.use_constant_weight, "use_constant_weight");
  field(config.min_measurement_weight, "min_measurement_weight");
  field(config.max_weight, "max_weight");
  field<ThreadNumConversion>(config.num_threads, "num_threads");
  field(config.interpolation_method, "interpolation_method");
  config.semantic_integrator.setOptional();
  field(config.semantic_integrator, "semantic_integrator");

  check(config.num_threads, GT, 0, "num_threads");
  check(config.min_measurement_weight, GE, 0.0f, "min_measurement_weight");
  check(config.max_weight, GT, 0, "max_weight");
  if (config.use_weight_dropoff) {
    check(config.weight_dropoff_epsilon, NE, 0.0f, "weight_dropoff_epsilon");
  }
}

ProjectiveIntegrator::ProjectiveIntegrator(const ProjectiveIntegrator::Config& config)
    : config(config::checkValid(config)),
      interpolator_(config.interpolation_method.create()),
      semantic_integrator_(config.semantic_integrator.create()) {}

void ProjectiveIntegrator::updateMap(const InputData& data,
                                     VolumetricMap& map,
                                     bool allocate_blocks) const {
  auto& tsdf = map.getTsdfLayer();

  // Allocate all blocks that could be seen by the sensor.
  const auto body_T_sensor = data.getSensorPose().cast<float>();
  const auto block_indices = findBlocksInViewFrustum(
      data.getSensor(), body_T_sensor, map.blockSize(), data.min_range, data.max_range);
  BlockIndices new_blocks;
  if (allocate_blocks) {
    new_blocks = map.allocateBlocks(block_indices);
  }

  updateBlocks(block_indices, data, map);

  // De-allocate blocks that were not updated.
  for (const auto& idx : new_blocks) {
    if (!tsdf.getBlock(idx).updated) {
      map.removeBlock(idx);
    }
  }
}

void ProjectiveIntegrator::updateBlocks(const BlockIndices& block_indices,
                                        const InputData& data,
                                        VolumetricMap& map) const {
  LOG_IF(INFO, config.verbosity >= 3)
      << "Updating " << block_indices.size() << " blocks.";

  // Update all blocks in parallel.
  IndexGetter<BlockIndex> index_getter(block_indices);

  // TODO(nathan) reconsider future/async
  std::vector<std::future<void>> threads;
  for (int i = 0; i < config.num_threads; ++i) {
    threads.emplace_back(std::async(std::launch::async, [&]() {
      BlockIndex block_index;
      while (index_getter.getNextIndex(block_index)) {
        updateBlock(block_index, data, map);
      }
    }));
  }

  for (auto& thread : threads) {
    thread.get();
  }
}

void ProjectiveIntegrator::updateBlock(const BlockIndex& block_index,
                                       const InputData& data,
                                       VolumetricMap& map) const {
  // Get the requested blocks.
  BlockTuple blocks = map.getBlock(block_index);
  if (!blocks.tsdf) {
    // Skip unallocated blocks.
    return;
  }
  const auto sensor_T_body = data.getSensorPose().cast<float>().inverse();

  // Update all voxels.
  bool was_updated = false;
  for (size_t i = 0; i < blocks.tsdf->numVoxels(); ++i) {
    const auto p_sensor = sensor_T_body * blocks.tsdf->getVoxelPosition(i);
    const auto measurement = getVoxelMeasurement(map.config, data, p_sensor);
    if (!measurement.valid) {
      continue;
    }

    auto voxels = blocks.getVoxels(i);
    updateVoxel(map.config, data, measurement, voxels);
    was_updated = true;
  }

  if (was_updated) {
    LOG_IF(INFO, config.verbosity >= 10) << "integrator updated block "
                                         << showIndex(block_index);
    blocks.tsdf->setUpdated();
  }
}

Measurement ProjectiveIntegrator::getVoxelMeasurement(const MapConfig& map_config,
                                                      const InputData& data,
                                                      const Point& p_C) const {
  Measurement measurement;

  // Check the point is within range of the sensor and input data
  const auto voxel_range = p_C.norm();
  if (!data.inRange(voxel_range)) {
    return measurement;
  }

  // Check the point is valid for interpolation
  if (!interpolatePoint(data, p_C, measurement.interpolation_weights)) {
    return measurement;
  }

  // Compute the (partially truncated) signed distance to the surface
  measurement.sdf =
      computeSDF(map_config, data, measurement.interpolation_weights, voxel_range);
  if (!std::isfinite(measurement.sdf)) {
    return measurement;
  }

  // Avoid integrating points where we don't have information
  if (measurement.sdf < -map_config.truncation_distance) {
    if (!config.extra_integration_distance) {
      return measurement;  // voxel beyond truncation distance
    }

    // Voxel's negative distance past the truncation band
    const auto dist_m = measurement.sdf + map_config.truncation_distance;
    // Positive distance threshold past the truncation band
    const auto threshold_m =
        config.extra_integration_distance < 0.0f
            ? -config.extra_integration_distance * map_config.voxel_size
            : config.extra_integration_distance;

    // If dist_m is bigger than threshold_m in absolute value, the sum will be negative
    if (dist_m + threshold_m < 0.0f) {
      return measurement;
    }

    // Clip measurement to be at the negative truncation distance
    measurement.sdf = -map_config.truncation_distance;
  }

  // Get associated semantic label if applicable and check if it can be integrated
  if (!computeLabel(map_config, data, measurement)) {
    return measurement;
  }

  // Compute the weight of the measurement
  measurement.weight =
      computeWeight(map_config, data.getSensor(), p_C, measurement.sdf);
  measurement.valid = true;
  return measurement;
}

void ProjectiveIntegrator::updateVoxel(const MapConfig& map_config,
                                       const InputData& data,
                                       const Measurement& measurement,
                                       VoxelTuple& voxels) const {
  if (!std::isfinite(measurement.sdf)) {
    LOG(ERROR) << "found invalid measurement!";
    return;
  }

  // Cache old weight and add measurement to the current weight
  auto& tsdf_voxel = *voxels.tsdf;
  const auto prev_weight = tsdf_voxel.weight;
  tsdf_voxel.weight =
      std::min(tsdf_voxel.weight + measurement.weight, config.max_weight);

  // Update TSDF distance using weighted averaging fusion
  tsdf_voxel.distance =
      (tsdf_voxel.distance * prev_weight + measurement.sdf * measurement.weight) /
      (prev_weight + measurement.weight);

  if (voxels.tracking) {
    voxels.tracking->last_observed = data.timestamp_ns;
  }

  // Don't bother updating colors or labels outside the truncation distance
  if (std::abs(measurement.sdf) >= map_config.truncation_distance) {
    return;
  }

  // TODO(nathan) refactor into update functions
  if (!data.color_image.empty()) {
    const auto color = interpolator_->interpolateColor(
        data.color_image, measurement.interpolation_weights);
    const float ratio = measurement.weight / (tsdf_voxel.weight + measurement.weight);
    tsdf_voxel.color.merge(color, ratio);
  }

  if (!semantic_integrator_ || !voxels.semantic) {
    return;
  }

  if (semantic_integrator_->isValidLabel(measurement.label)) {
    semantic_integrator_->updateLikelihoods(measurement.label, *voxels.semantic);
  }
}

bool ProjectiveIntegrator::interpolatePoint(const InputData& data,
                                            const Point& p_C,
                                            InterpolationWeights& weights) const {
  // Project the current voxel into the range image, only count points that fall
  // fully into the image so the
  float u, v;
  if (!data.getSensor().projectPointToImagePlane(p_C, u, v)) {
    return false;
  }

  // Interpolate the voxel center in the images.
  weights = interpolator_->computeWeights(u, v, data.range_image);
  return weights.valid;
}

float ProjectiveIntegrator::computeSDF(const MapConfig& map_config,
                                       const InputData& data,
                                       const InterpolationWeights& weights,
                                       const float distance_to_voxel) const {
  const auto d_to_surface = interpolator_->InterpolateRange(data.range_image, weights);
  // NOTE(nathan) If the sdf value is NaN, this should return NaN (min returns a if a <
  // b, a < NaN is false) This is probably the desired behavior if the range image
  // contains NaNs (the only source of non-finite values) but may need to be checked
  return std::min(map_config.truncation_distance, d_to_surface - distance_to_voxel);
}

float ProjectiveIntegrator::computeWeight(const MapConfig& map_config,
                                          const Sensor& sensor,
                                          const Point& p_C,
                                          const float sdf) const {
  // This approximates the number of rays that would hit this voxel.
  // NOTE(lschmid): For (close) voxels hit by many pixels one could think of denoising
  // the depth image / looking at a local neighborhood for the update.
  const float depth = sensor.getPointDepth(p_C);
  auto weight = sensor.computeRayDensity(map_config.voxel_size, depth);

  // Weight reduction with distance squared (according to sensor noise models).
  if (!config.use_constant_weight) {
    weight /= std::pow(depth, 2.f);
  }

  // Apply weight drop-off if appropriate.
  if (config.use_weight_dropoff) {
    const float dropoff_epsilon =
        config.weight_dropoff_epsilon > 0.f
            ? config.weight_dropoff_epsilon
            : config.weight_dropoff_epsilon * -map_config.voxel_size;
    if (sdf < -dropoff_epsilon) {
      weight *= (map_config.truncation_distance + sdf) /
                (map_config.truncation_distance - dropoff_epsilon);
    }
  }

  // clip weight to be at least as much as the specified min weight (which is always
  // nonnegative)
  weight = std::max(weight, config.min_measurement_weight);
  return weight;
}

bool ProjectiveIntegrator::computeLabel(const MapConfig& map_config,
                                        const InputData& data,
                                        Measurement& measurement) const {
  if (std::abs(measurement.sdf) >= map_config.truncation_distance) {
    // If SDF value is beyond the truncation band, we don't need to
    // compute a label and the point is always valid for integration
    return true;
  }

  if (data.label_image.empty() || !semantic_integrator_) {
    return true;
  }

  measurement.label =
      interpolator_->interpolateID(data.label_image, measurement.interpolation_weights);
  return semantic_integrator_->canIntegrate(measurement.label);
}

}  // namespace hydra
