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
#include "hydra_ros/places/external_traversability_estimator.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <ianvs/node_handle.h>
#include <spark_dsg/traversability_boundary.h>

namespace hydra::places {

using spark_dsg::TraversabilityState;

static const auto registration =
    config::RegistrationWithConfig<TraversabilityEstimator,
                                   ExternalTraversabilityEstimator,
                                   ExternalTraversabilityEstimator::Config>(
        "ExternalTraversabilityEstimator");

void declare_config(ExternalTraversabilityEstimator::Config& config) {
  using namespace config;
  name("ExternalTraversabilityEstimator::Config");
  field(config.input_topic, "input_topic");
  field(config.queue_size, "queue_size");
  checkCondition(!config.input_topic.empty(), "'input_topic' must be specified");
}

ExternalTraversabilityEstimator::ExternalTraversabilityEstimator(const Config& config)
    : TraversabilityEstimator(TraversabilityEstimator::Config{}),
      config(config::checkValid(config)) {
  auto nh = ianvs::NodeHandle::this_node();
  sub_ = nh.create_subscription<nav_msgs::msg::OccupancyGrid>(
      config.input_topic,
      config.queue_size,
      &ExternalTraversabilityEstimator::callback,
      this);
}

const TraversabilityLayer& ExternalTraversabilityEstimator::getTraversabilityLayer()
    const {
  std::lock_guard<std::mutex> lock(mutex_);
  return *traversability_layer_;
}

void ExternalTraversabilityEstimator::updateTraversability(
    const ActiveWindowOutput& msg) {
  if (!traversability_layer_) {
    const auto& map_config = msg.map().config;
    traversability_layer_ = std::make_unique<TraversabilityLayer>(
        map_config.voxel_size, map_config.voxels_per_side);
  }
}

void ExternalTraversabilityEstimator::callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  if (!traversability_layer_) {
    return;
  }
  // Reset update tracking.
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& block : *traversability_layer_) {
    block.updated = false;
    for (auto& voxel : block.voxels) {
      voxel.confidence = 0.0f;
    }
  }

  // Updated the stored layer with the map.
  const float voxel_size = msg->info.resolution;
  const Eigen::Vector3f origin(
      msg->info.origin.position.x, msg->info.origin.position.y, 0.0f);
  for (unsigned int y = 0; y < msg->info.height; ++y) {
    for (unsigned int x = 0; x < msg->info.width; ++x) {
      const int idx = x + y * msg->info.width;
      const State state = occupancyToTraversability(msg->data[idx]);
      // Compute the voxel coordinates.
      const Eigen::Vector3f position =
          Eigen::Vector3f(x, y, 0.0f) * voxel_size + origin;
      const auto index = traversability_layer_->globalIndexFromPoint(position);
      // TODO(lschmid): Fix the allocate block interfaces for the traversability layer.
      auto& block = traversability_layer_->allocateBlock(
          traversability_layer_->blockIndexFromGlobal(index),
          traversability_layer_->voxels_per_side);
      block.updated = true;
      auto& voxel = block.voxel(traversability_layer_->voxelIndexFromGlobal(index));
      if (voxel.confidence == 0.0f) {
        // Overwrite if this is the first observation this iteration.
        voxel.state = state;
        voxel.confidence = 1.0f;
      } else {
        spark_dsg::fuseStates(state, voxel.state);
        voxel.confidence += 1.0f;
      }
    }
  }

  // Remove blocks that were not updated.
  spatial_hash::BlockIndices to_remove;
  for (auto& block : *traversability_layer_) {
    if (!block.updated) {
      to_remove.push_back(block.index);
    }
  }
  traversability_layer_->removeBlocks(to_remove);
}

ExternalTraversabilityEstimator::State
ExternalTraversabilityEstimator::occupancyToTraversability(int8_t occupancy) const {
  if (occupancy < 0) {
    return State::UNKNOWN;
  }
  if (occupancy == 0) {
    return State::TRAVERSABLE;
  }
  return State::INTRAVERSABLE;
}

}  // namespace hydra::places
