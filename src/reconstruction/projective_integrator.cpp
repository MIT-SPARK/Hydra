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

#include "hydra/common/common.h"
#include "hydra/reconstruction/index_getter.h"
#include "hydra/reconstruction/sensor_utilities.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using VoxelMeasurement = ProjectiveIntegrator::VoxelMeasurement;
using timing::ScopedTimer;
using voxblox::BlockIndex;
using voxblox::BlockIndexList;
using voxblox::Point;
using voxblox::TsdfVoxel;

ProjectiveIntegrator::ProjectiveIntegrator(const ProjectiveIntegratorConfig& config)
    : config_(config::checkValid(config)),
      interpolator_(config::create<ProjectionInterpolator>(config_.interp_method)),
      semantic_integrator_(config_.semantic_integrator.create()) {}

void ProjectiveIntegrator::updateMap(const Sensor& sensor,
                                     const FrameData& data,
                                     VolumetricMap& map,
                                     bool allocate_blocks) const {
  BlockIndexList block_indices;
  const auto body_T_sensor = data.getSensorPose<float>(sensor);
  if (config_.accurate_visible_blocks) {
    block_indices = findVisibleBlocks(sensor,
                                      body_T_sensor,
                                      data.vertex_map,
                                      map.block_size,
                                      map.truncation_distance());
  } else {
    block_indices = findBlocksInViewFrustum(
        sensor, body_T_sensor, map.block_size, data.min_range, data.max_range);
  }

  BlockIndexList new_blocks;
  if (allocate_blocks) {
    for (const auto& idx : block_indices) {
      if (map.getTsdfLayer().hasBlock(idx)) {
        continue;
      }

      new_blocks.push_back(idx);
      map.getTsdfLayer().allocateBlockPtrByIndex(idx);
      const auto semantic_layer = map.getSemanticLayer();
      if (semantic_integrator_ && semantic_layer) {
        semantic_layer->allocateBlockPtrByIndex(idx);
      }
    }
  }

  VLOG(config_.verbosity) << "Updating " << block_indices.size() << " blocks.";
  ScopedTimer timer("integration_tsdf", data.timestamp_ns);
  updateBlocks(std::bind(&ProjectiveIntegrator::updateMapBlock,
                         this,
                         std::placeholders::_1,
                         std::placeholders::_2,
                         std::placeholders::_3,
                         std::placeholders::_4),
               sensor,
               block_indices,
               data,
               map);

  for (const auto& idx : new_blocks) {
    const auto block = map.getTsdfLayer().getBlockPtrByIndex(idx);
    if (!block || block->updated().any()) {
      continue;
    }

    map.getTsdfLayer().removeBlock(idx);
    const auto semantic_layer = map.getSemanticLayer();
    if (semantic_layer) {
      semantic_layer->removeBlock(idx);
    }
  }
}

void ProjectiveIntegrator::updateBlocks(const BlockUpdateFunction& update_function,
                                        const Sensor& sensor,
                                        const BlockIndexList& block_indices,
                                        const FrameData& data,
                                        VolumetricMap& map) const {
  // Update all blocks in parallel.
  std::vector<BlockIndex> block_idx_vector(block_indices.begin(), block_indices.end());
  IndexGetter<BlockIndex> index_getter(block_idx_vector);

  // TODO(nathan) reconsider future/async
  std::vector<std::future<void>> threads;
  for (int i = 0; i < config_.num_threads; ++i) {
    threads.emplace_back(std::async(std::launch::async, [&]() {
      BlockIndex block_index;
      while (index_getter.getNextIndex(block_index)) {
        update_function(sensor, block_index, data, map);
      }
    }));
  }

  for (auto& thread : threads) {
    thread.get();
  }
}

void ProjectiveIntegrator::updateMapBlock(const Sensor& sensor,
                                          const BlockIndex& block_index,
                                          const FrameData& data,
                                          VolumetricMap& map) const {
  // Get and allocate block if necessary.
  const auto tsdf_block = map.getTsdfLayer().getBlockPtrByIndex(block_index);
  if (!tsdf_block) {
    LOG(WARNING) << "Tried to integrate non-existing TSDF block, skipping!";
    return;
  }

  voxblox::Block<SemanticVoxel>::Ptr semantic_block;
  const auto semantics = map.getSemanticLayer();
  if (semantics) {
    semantic_block = semantics->getBlockPtrByIndex(block_index);
  }

  const auto body_T_sensor = data.getSensorPose<float>(sensor);
  const auto sensor_T_body = body_T_sensor.inverse();

  // Update all voxels.
  bool was_updated = false;
  const float truncation_distance = map.truncation_distance();
  const float voxel_size = map.voxel_size();
  for (size_t i = 0; i < tsdf_block->num_voxels(); ++i) {
    const auto p_C = sensor_T_body * tsdf_block->computeCoordinatesFromLinearIndex(i);
    const auto measurement =
        getVoxelUpdate(sensor, p_C, data, truncation_distance, voxel_size);
    if (!measurement.valid) {
      continue;
    }

    auto& tsdf_voxel = tsdf_block->getVoxelByLinearIndex(i);
    SemanticVoxel* semantic_voxel =
        semantic_block ? &semantic_block->getVoxelByLinearIndex(i) : nullptr;
    updateVoxel(data, measurement, truncation_distance, tsdf_voxel, semantic_voxel);
    was_updated = true;
  }

  if (was_updated) {
    VLOG(VLEVEL_DEBUG) << "integrator updated block [" << block_index.x() << ", "
                       << block_index.y() << ", " << block_index.z() << "]";
    tsdf_block->updated().set();
  }
}

VoxelMeasurement ProjectiveIntegrator::getVoxelUpdate(const Sensor& sensor,
                                                      const Point& p_C,
                                                      const FrameData& data,
                                                      const float truncation_distance,
                                                      const float voxel_size) const {
  VoxelMeasurement measurement;
  const auto voxel_range = p_C.norm();
  if (voxel_range < sensor.min_range() || voxel_range > sensor.max_range()) {
    return measurement;
  }

  // Check the point is valid for interpolation.
  if (!interpolatePoint(sensor, p_C, data, measurement.interpolation_weights)) {
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
  const bool is_surface = measurement.sdf < truncation_distance;
  if (!data.label_image.empty() && semantic_integrator_ && is_surface) {
    measurement.label = interpolator_->interpolateID(data.label_image,
                                                     measurement.interpolation_weights);
    if (!semantic_integrator_->canIntegrate(measurement.label)) {
      return measurement;
    }
  }

  // Compute the weight of the measurement.
  measurement.weight =
      computeWeight(sensor, p_C, measurement.sdf, truncation_distance, voxel_size);
  measurement.valid = true;
  return measurement;
}

void ProjectiveIntegrator::updateVoxel(const FrameData& data,
                                       const VoxelMeasurement& measurement,
                                       const float truncation_distance,
                                       TsdfVoxel& voxel,
                                       SemanticVoxel* semantic_voxel) const {
  if (!measurement.valid) {
    return;
  }

  if (!std::isfinite(measurement.sdf)) {
    LOG(ERROR) << "found invalid measurement!";
    return;
  }

  // Cache old weight and add measurement to the current weight
  const auto prev_weight = voxel.weight;
  voxel.weight = std::min(voxel.weight + measurement.weight, config_.max_weight);

  // Update TSDF distance using weighted averaging fusion.
  voxel.distance =
      (voxel.distance * prev_weight + measurement.sdf * measurement.weight) /
      (prev_weight + measurement.weight);

  // Only merge other quantities near the surface
  if (measurement.sdf >= truncation_distance) {
    return;
  }

  // TODO(nathan) refactor into update functions

  if (!data.color_image.empty()) {
    const auto color = interpolator_->interpolateColor(
        data.color_image, measurement.interpolation_weights);
    voxel.color = voxblox::Color::blendTwoColors(
        voxel.color, prev_weight, color, measurement.weight);
  }

  if (semantic_integrator_->isValidLabel(measurement.label)) {
    semantic_integrator_->updateLikelihoods(measurement.label, *semantic_voxel);
  }
}

bool ProjectiveIntegrator::interpolatePoint(const Sensor& sensor,
                                            const Point& p_C,
                                            const FrameData& data,
                                            InterpolationWeights& weights) const {
  // Project the current voxel into the range image, only count points that fall
  // fully into the image so the
  float u, v;
  if (!sensor.projectPointToImagePlane(p_C, u, v)) {
    return false;
  }

  // Interpolate the voxel center in the images.
  weights = interpolator_->computeWeights(u, v, data.range_image);
  return weights.valid;
}

float ProjectiveIntegrator::computeSDF(const FrameData& data,
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
  if (!config_.use_constant_weight) {
    weight /= std::pow(p_C.z(), 2.f);
  }

  // Apply weight drop-off if appropriate.
  if (config_.use_weight_dropoff) {
    const float dropoff_epsilon = config_.weight_dropoff_epsilon > 0.f
                                      ? config_.weight_dropoff_epsilon
                                      : config_.weight_dropoff_epsilon * -voxel_size;
    if (sdf < -dropoff_epsilon) {
      weight *= (truncation_distance + sdf) / (truncation_distance - dropoff_epsilon);
      weight = std::max(weight, 0.f);
    }
  }
  return weight;
}

}  // namespace hydra
