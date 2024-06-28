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

#include <algorithm>
#include <future>
#include <memory>
#include <utility>
#include <vector>

#include "hydra/input/sensor_utilities.h"
#include "hydra/reconstruction/index_getter.h"

namespace hydra {

using VoxelMeasurement = ProjectiveIntegrator::VoxelMeasurement;

ProjectiveIntegrator::ProjectiveIntegrator(const ProjectiveIntegratorConfig& config)
    : config(config::checkValid(config)),
      interpolator_(config::create<ProjectionInterpolator>(config.interp_method)),
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

  VLOG(config.verbosity) << "Updating " << block_indices.size() << " blocks.";
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
  const float truncation_distance = map.config.truncation_distance;
  const float voxel_size = map.config.voxel_size;
  for (size_t i = 0; i < blocks.tsdf->numVoxels(); ++i) {
    const auto p_sensor = sensor_T_body * blocks.tsdf->getVoxelPosition(i);
    const auto measurement =
        getVoxelMeasurement(p_sensor, data, truncation_distance, voxel_size);
    if (!measurement.valid) {
      continue;
    }

    auto voxels = blocks.getVoxels(i);
    updateVoxel(data, measurement, truncation_distance, voxels);
    was_updated = true;
  }

  if (was_updated) {
    VLOG(10) << "integrator updated block [" << showIndex(block_index) << "]";
    blocks.tsdf->setUpdated();
  }
}

VoxelMeasurement ProjectiveIntegrator::getVoxelMeasurement(
    const Point& p_C,
    const InputData& data,
    const float truncation_distance,
    const float voxel_size) const {
  VoxelMeasurement measurement;
  const auto voxel_range = p_C.norm();
  if (voxel_range < data.getSensor().min_range() ||
      voxel_range > data.getSensor().max_range() || voxel_range > data.max_range) {
    return measurement;
  }

  // Check the point is valid for interpolation.
  if (!interpolatePoint(p_C, data, measurement.interpolation_weights)) {
    return measurement;
  }

  // Compute the signed distance to the surface.
  measurement.sdf = computeSDF(
      data, measurement.interpolation_weights, truncation_distance, voxel_range);
  if (!std::isfinite(measurement.sdf)) {
    return measurement;
  }

  if (measurement.sdf < -truncation_distance) {
    return measurement;
  }

  // Don't integrate surface voxels of dynamic measurements.
  if (!computeLabel(data, truncation_distance, measurement)) {
    return measurement;
  }

  // Compute the weight of the measurement.
  measurement.weight = computeWeight(
      data.getSensor(), p_C, measurement.sdf, truncation_distance, voxel_size);
  measurement.valid = true;
  return measurement;
}

void ProjectiveIntegrator::updateVoxel(const InputData& data,
                                       const VoxelMeasurement& measurement,
                                       const float truncation_distance,
                                       VoxelTuple& voxels) const {
  if (!std::isfinite(measurement.sdf)) {
    LOG(ERROR) << "found invalid measurement!";
    return;
  }

  // Cache old weight and add measurement to the current weight.
  auto& tsdf_voxel = *voxels.tsdf;
  const auto prev_weight = tsdf_voxel.weight;
  tsdf_voxel.weight =
      std::min(tsdf_voxel.weight + measurement.weight, config.max_weight);

  // Update TSDF distance using weighted averaging fusion.
  tsdf_voxel.distance =
      (tsdf_voxel.distance * prev_weight + measurement.sdf * measurement.weight) /
      (prev_weight + measurement.weight);

  if (voxels.tracking) {
    voxels.tracking->last_observed = data.timestamp_ns;
  }

  // Only merge other quantities near the surface
  if (measurement.sdf >= truncation_distance) {
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

bool ProjectiveIntegrator::interpolatePoint(const Point& p_C,
                                            const InputData& data,
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

float ProjectiveIntegrator::computeSDF(const InputData& data,
                                       const InterpolationWeights& weights,
                                       const float truncation_distance,
                                       const float distance_to_voxel) const {
  const auto d_to_surface = interpolator_->InterpolateRange(data.range_image, weights);
  const auto sdf = d_to_surface - distance_to_voxel;
  return std::min(sdf, truncation_distance);
}

float ProjectiveIntegrator::computeWeight(const Sensor& sensor,
                                          const Point& p_C,
                                          const float sdf,
                                          const float truncation_distance,
                                          const float voxel_size) const {
  // This approximates the number of rays that would hit this voxel.
  // NOTE(lschmid): For (close) voxels hit by many pixels one could think of denoising
  // the depth image / looking at a local neighborhood for the update.
  auto weight = sensor.computeRayDensity(voxel_size, p_C.z());

  // Weight reduction with distance squared (according to sensor noise models).
  if (!config.use_constant_weight) {
    weight /= std::pow(p_C.z(), 2.f);
  }

  // Apply weight drop-off if appropriate.
  if (config.use_weight_dropoff) {
    const float dropoff_epsilon = config.weight_dropoff_epsilon > 0.f
                                      ? config.weight_dropoff_epsilon
                                      : config.weight_dropoff_epsilon * -voxel_size;
    if (sdf < -dropoff_epsilon) {
      weight *= (truncation_distance + sdf) / (truncation_distance - dropoff_epsilon);
      weight = std::max(weight, 0.f);
    }
  }
  return weight;
}

bool ProjectiveIntegrator::computeLabel(const InputData& data,
                                        const float truncation_distance,
                                        VoxelMeasurement& measurement) const {
  const bool is_surface = measurement.sdf < truncation_distance;
  if (data.label_image.empty() || !semantic_integrator_ || !is_surface) {
    return true;
  }
  measurement.label =
      interpolator_->interpolateID(data.label_image, measurement.interpolation_weights);
  return semantic_integrator_->canIntegrate(measurement.label);
}

}  // namespace hydra
