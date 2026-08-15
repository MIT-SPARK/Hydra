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
#include "hydra_ros/active_window/tsdf_gradient_occupancy_publisher.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/types/eigen_matrix.h>
#include <config_utilities/validation.h>
#include <hydra/common/global_info.h>

#include <cmath>
#include <queue>
#include <set>

namespace hydra {

using hydra::places::GradientInfo;
using hydra::places::GradientMap;
using hydra::places::HeightMap;
using hydra::places::Index2D;

void declare_config(TsdfGradientOccupancyPublisher::Config& config) {
  using namespace config;
  name("TsdfGradientOccupancyPublisher::Config");
  field(config.ns, "ns");
  field(config.collate, "collate");
  field(config.use_relative_height, "use_relative_height");
  field(config.add_robot_footprint, "add_robot_footprint");
  field(config.footprint_min, "footprint_min");
  field(config.footprint_max, "footprint_max");
  field(config.gradient_threshold, "gradient_threshold");
  field(config.min_confidence, "min_confidence");
  field(config.probabilistic, "probabilistic");
  field(config.filter_disjoint, "filter_disjoint");

  checkCondition(config.gradient_threshold > 0.0f,
                 "gradient_threshold must be positive");
  checkInRange(config.min_confidence, 0.0f, 1.0f, "min_confidence");
}

TsdfGradientOccupancyPublisher::TsdfGradientOccupancyPublisher(const Config& config)
    : config(config::checkValid(config)),
      pub_(ianvs::NodeHandle::this_node(config.ns)
               .create_publisher<nav_msgs::msg::OccupancyGrid>(
                   "occupancy", rclcpp::QoS(1).transient_local())),
      height_map_pub_(ianvs::NodeHandle::this_node(config.ns)
                          .create_publisher<nav_msgs::msg::OccupancyGrid>(
                              "height_map_debug", rclcpp::QoS(1).transient_local())),
      gradient_map_pub_(
          ianvs::NodeHandle::this_node(config.ns)
              .create_publisher<nav_msgs::msg::OccupancyGrid>(
                  "gradient_map_debug", rclcpp::QoS(1).transient_local())) {}

std::string TsdfGradientOccupancyPublisher::printInfo() const {
  return config::toString(config);
}

void TsdfGradientOccupancyPublisher::call(const HeightMap& height_map,
                                          const GradientMap& gradient_map,
                                          const ActiveWindowOutput& output,
                                          const TsdfLayer& tsdf_layer) const {
  if (!pub_->get_subscription_count() && !height_map_pub_->get_subscription_count() &&
      !gradient_map_pub_->get_subscription_count()) {
    return;
  }

  const uint64_t timestamp_ns = output.timestamp_ns;
  const auto& world_T_body = output.world_T_body();
  const float robot_z = static_cast<float>(world_T_body.translation().z());

  publishHeightMapViz(height_map, tsdf_layer, timestamp_ns, robot_z);
  publishGradientMapViz(gradient_map, tsdf_layer, timestamp_ns, robot_z);

  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = GlobalInfo::instance().getFrames().map;
  msg.header.stamp = rclcpp::Time(timestamp_ns);
  msg.info.map_load_time = msg.header.stamp;
  fillOccupancyGrid(gradient_map, world_T_body, tsdf_layer, msg);
  if (config.filter_disjoint) {
    filterDisjointFreeSpace(msg, world_T_body);
  }
  pub_->publish(msg);
}

void TsdfGradientOccupancyPublisher::fillOccupancyGrid(
    const GradientMap& gradient_map,
    const Eigen::Isometry3d& world_T_sensor,
    const TsdfLayer& layer,
    nav_msgs::msg::OccupancyGrid& msg) const {
  if (gradient_map.empty()) {
    return;
  }

  // Compute bounds using block origins
  Eigen::Vector2f x_min = Eigen::Vector2f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector2f x_max =
      Eigen::Vector2f::Constant(std::numeric_limits<float>::lowest());

  const float voxel_size = layer.voxel_size;
  for (const auto& block : layer) {
    const auto lower = block.origin();
    const auto upper = lower + spatial_hash::Point::Constant(block.block_size);
    x_min = x_min.array().min(lower.template head<2>().array());
    x_max = x_max.array().max(upper.template head<2>().array());
  }

  const Eigen::Vector2f dims = (x_max - x_min) / voxel_size;

  // Initialize grid
  msg.info.resolution = voxel_size;
  msg.info.width = std::ceil(dims.x());
  msg.info.height = std::ceil(dims.y());
  msg.info.origin.position.x = x_min.x();
  msg.info.origin.position.y = x_min.y();
  msg.info.origin.position.z =
      config.use_relative_height ? static_cast<float>(world_T_sensor.translation().z())
                                 : 0.0f;
  msg.info.origin.orientation.w = 1.0;
  msg.data.resize(msg.info.width * msg.info.height, -1);

  // Setup robot footprint if needed
  spark_dsg::BoundingBox bbox;
  Eigen::Isometry3f sensor_T_world;
  if (config.add_robot_footprint) {
    bbox = spark_dsg::BoundingBox(config.footprint_min, config.footprint_max);
    sensor_T_world = world_T_sensor.inverse().cast<float>();
  }

  // Fill occupancy grid
  for (const auto& [global_idx, gradient_info] : gradient_map) {
    // Convert global 2D index to 3D, get voxel key, then actual position
    const spatial_hash::GlobalIndex global_3d(global_idx.x(), global_idx.y(), 0);
    const VoxelKey key =
        spatial_hash::keyFromGlobalIndex(global_3d, layer.voxels_per_side);
    const Eigen::Vector3f pos_3d = layer.getVoxelPosition(key);
    const Eigen::Vector2f pos = pos_3d.head<2>();
    const Eigen::Vector2f rel_pos = pos - x_min;

    const auto r = std::floor(rel_pos.y() / voxel_size);
    const auto c = std::floor(rel_pos.x() / voxel_size);
    const size_t index = r * msg.info.width + c;

    if (index >= msg.data.size()) {
      continue;
    }

    if (config.add_robot_footprint) {
      const Eigen::Vector3f pos_3d(pos.x(), pos.y(), 0.0f);
      if (bbox.contains((sensor_T_world * pos_3d).eval())) {
        msg.data[index] = 0;
        continue;
      }
    }

    msg.data[index] =
        gradientToOccupancy(gradient_info.gradient, gradient_info.confidence);
  }
}

void TsdfGradientOccupancyPublisher::filterDisjointFreeSpace(
    nav_msgs::msg::OccupancyGrid& msg, const Eigen::Isometry3d& world_T_body) const {
  if (msg.data.empty()) {
    return;
  }

  const int width = static_cast<int>(msg.info.width);
  const int height = static_cast<int>(msg.info.height);

  const auto robot_pos_world = world_T_body.translation();
  const float rel_x =
      static_cast<float>(robot_pos_world.x() - msg.info.origin.position.x);
  const float rel_y =
      static_cast<float>(robot_pos_world.y() - msg.info.origin.position.y);
  const int robot_r = static_cast<int>(std::floor(rel_y / msg.info.resolution));
  const int robot_c = static_cast<int>(std::floor(rel_x / msg.info.resolution));

  // Check if robot is within grid bounds
  if (robot_r < 0 || robot_r >= height || robot_c < 0 || robot_c >= width) {
    return;  // Robot outside grid - skip filtering
  }

  const size_t robot_index = robot_r * width + robot_c;

  // Only filter if robot is in free space
  if (msg.data[robot_index] != 0) {
    return;  // Robot not in free space - skip filtering
  }

  // BFS flood fill from robot position
  std::vector<bool> visited(msg.data.size(), false);
  std::queue<size_t> to_visit;
  to_visit.push(robot_index);
  visited[robot_index] = true;

  while (!to_visit.empty()) {
    const size_t current_index = to_visit.front();
    to_visit.pop();

    const int r = current_index / width;
    const int c = current_index % width;

    // Check 4-connected neighbors (up, down, left, right)
    // 4-way more conservative than 8-way
    const std::array<std::pair<int, int>, 4> neighbors = {
        {{r - 1, c}, {r + 1, c}, {r, c - 1}, {r, c + 1}}};

    for (const auto& [nr, nc] : neighbors) {
      if (nr < 0 || nr >= height || nc < 0 || nc >= width) {
        continue;
      }

      const size_t neighbor_index = nr * width + nc;
      if (visited[neighbor_index]) {
        continue;
      }

      // Only expand through free cells (value 0)
      if (msg.data[neighbor_index] == 0) {
        visited[neighbor_index] = true;
        to_visit.push(neighbor_index);
      }
    }
  }

  // Mark unvisited free cells as occupied
  for (size_t i = 0; i < msg.data.size(); ++i) {
    if (msg.data[i] == 0 && !visited[i]) {
      msg.data[i] = 100;  // Mark as occupied
    }
  }
}

int8_t TsdfGradientOccupancyPublisher::gradientToOccupancy(float gradient,
                                                           float confidence) const {
  if (confidence < config.min_confidence) {
    return -1;
  }

  if (config.probabilistic) {
    const float traversability = hydra::places::computeTraversabilityFromGradient(
        gradient, config.gradient_threshold);
    const float occupancy_float = (1.0f - traversability) * 100.0f;
    return static_cast<int8_t>(std::clamp(occupancy_float, 0.0f, 100.0f));
  } else {
    return gradient >= config.gradient_threshold ? 100 : 0;
  }
}

void TsdfGradientOccupancyPublisher::publishHeightMapViz(const HeightMap& height_map,
                                                         const TsdfLayer& layer,
                                                         uint64_t timestamp_ns,
                                                         float robot_z) const {
  if (!height_map_pub_->get_subscription_count()) {
    return;
  }

  if (height_map.empty()) {
    return;
  }

  float min_height = std::numeric_limits<float>::max();
  float max_height = std::numeric_limits<float>::lowest();
  for (const auto& [_, height] : height_map) {
    min_height = std::min(min_height, height);
    max_height = std::max(max_height, height);
  }

  Eigen::Vector2f x_min = Eigen::Vector2f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector2f x_max =
      Eigen::Vector2f::Constant(std::numeric_limits<float>::lowest());

  const float voxel_size = layer.voxel_size;
  for (const auto& block : layer) {
    const auto lower = block.origin();
    const auto upper = lower + spatial_hash::Point::Constant(block.block_size);
    x_min = x_min.array().min(lower.template head<2>().array());
    x_max = x_max.array().max(upper.template head<2>().array());
  }

  const Eigen::Vector2f dims = (x_max - x_min) / voxel_size;

  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = GlobalInfo::instance().getFrames().map;
  msg.header.stamp = rclcpp::Time(timestamp_ns);
  msg.info.map_load_time = msg.header.stamp;
  msg.info.resolution = voxel_size;
  msg.info.width = std::ceil(dims.x());
  msg.info.height = std::ceil(dims.y());
  msg.info.origin.position.x = x_min.x();
  msg.info.origin.position.y = x_min.y();
  msg.info.origin.position.z = config.use_relative_height ? robot_z : 0.0f;
  msg.info.origin.orientation.w = 1.0;
  msg.data.resize(msg.info.width * msg.info.height, -1);

  const float height_range = max_height - min_height;
  for (const auto& [global_idx, height] : height_map) {
    const spatial_hash::GlobalIndex global_3d(global_idx.x(), global_idx.y(), 0);
    const VoxelKey key =
        spatial_hash::keyFromGlobalIndex(global_3d, layer.voxels_per_side);
    const Eigen::Vector3f pos_3d = layer.getVoxelPosition(key);
    const Eigen::Vector2f pos = pos_3d.head<2>();
    const Eigen::Vector2f rel_pos = pos - x_min;

    const auto r = std::floor(rel_pos.y() / voxel_size);
    const auto c = std::floor(rel_pos.x() / voxel_size);
    const size_t index = r * msg.info.width + c;

    if (index >= msg.data.size()) {
      continue;
    }

    if (height_range > 1e-6f) {
      const float normalized = (height - min_height) / height_range * 100.0f;
      msg.data[index] = static_cast<int8_t>(std::clamp(normalized, 0.0f, 100.0f));
    } else {
      msg.data[index] = 50;
    }
  }

  height_map_pub_->publish(msg);
}

void TsdfGradientOccupancyPublisher::publishGradientMapViz(
    const GradientMap& gradient_map,
    const TsdfLayer& layer,
    uint64_t timestamp_ns,
    float robot_z) const {
  if (!gradient_map_pub_->get_subscription_count()) {
    return;
  }

  if (gradient_map.empty()) {
    return;
  }

  Eigen::Vector2f x_min = Eigen::Vector2f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector2f x_max =
      Eigen::Vector2f::Constant(std::numeric_limits<float>::lowest());

  const float voxel_size = layer.voxel_size;
  for (const auto& block : layer) {
    const auto lower = block.origin();
    const auto upper = lower + spatial_hash::Point::Constant(block.block_size);
    x_min = x_min.array().min(lower.template head<2>().array());
    x_max = x_max.array().max(upper.template head<2>().array());
  }

  const Eigen::Vector2f dims = (x_max - x_min) / voxel_size;

  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = GlobalInfo::instance().getFrames().map;
  msg.header.stamp = rclcpp::Time(timestamp_ns);
  msg.info.map_load_time = msg.header.stamp;
  msg.info.resolution = voxel_size;
  msg.info.width = std::ceil(dims.x());
  msg.info.height = std::ceil(dims.y());
  msg.info.origin.position.x = x_min.x();
  msg.info.origin.position.y = x_min.y();
  msg.info.origin.position.z = config.use_relative_height ? robot_z : 0.0f;
  msg.info.origin.orientation.w = 1.0;
  msg.data.resize(msg.info.width * msg.info.height, -1);

  for (const auto& [global_idx, gradient_info] : gradient_map) {
    const spatial_hash::GlobalIndex global_3d(global_idx.x(), global_idx.y(), 0);
    const VoxelKey key =
        spatial_hash::keyFromGlobalIndex(global_3d, layer.voxels_per_side);
    const Eigen::Vector3f pos_3d = layer.getVoxelPosition(key);
    const Eigen::Vector2f pos = pos_3d.head<2>();
    const Eigen::Vector2f rel_pos = pos - x_min;

    const auto r = std::floor(rel_pos.y() / voxel_size);
    const auto c = std::floor(rel_pos.x() / voxel_size);
    const size_t index = r * msg.info.width + c;

    if (index >= msg.data.size()) {
      continue;
    }

    const float normalized =
        std::min(gradient_info.gradient / config.gradient_threshold, 1.0f) * 100.0f;
    msg.data[index] = static_cast<int8_t>(normalized);
  }

  gradient_map_pub_->publish(msg);
}

namespace {

static const auto registration_ =
    config::RegistrationWithConfig<hydra::places::GradientTraversabilityEstimator::Sink,
                                   TsdfGradientOccupancyPublisher,
                                   TsdfGradientOccupancyPublisher::Config>(
        "TsdfGradientOccupancyPublisher");

}  // namespace

}  // namespace hydra
