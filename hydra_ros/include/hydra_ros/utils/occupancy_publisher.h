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
#pragma once
#include <hydra/common/global_info.h>
#include <ianvs/node_handle.h>
#include <spark_dsg/bounding_box.h>
#include <spatial_hash/voxel_layer.h>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/time.hpp>

#include <Eigen/Geometry>

namespace hydra {
namespace voxel_traits {

// TODO(nathan) move somewhere else?

template <typename T>
float getDistance(const T& /*voxel*/) {
  return 0.0f;
}

template <typename T>
bool isObserved(const T& /*voxel*/, float /*min_weight*/) {
  return false;
}

}  // namespace voxel_traits

struct OccupancyPublisherConfig {
  bool collate = false;
  bool use_relative_height = true;
  double slice_height = 0.0;
  size_t num_slices = 1;
  double min_observation_weight = 1.0e-6;
  double min_distance = 0.3;
  bool add_robot_footprint = false;
  Eigen::Vector3f footprint_min;
  Eigen::Vector3f footprint_max;
};

void declare_config(OccupancyPublisherConfig& config);

template <typename BlockT>
class OccupancyPublisher {
 public:
  using LayerT = spatial_hash::VoxelLayer<BlockT>;

  OccupancyPublisher(const OccupancyPublisherConfig& config, ianvs::NodeHandle nh)
      : config(config),
        pub_(nh.create_publisher<nav_msgs::msg::OccupancyGrid>(
            "occupancy", rclcpp::QoS(1).transient_local())) {}

  virtual ~OccupancyPublisher() = default;

  const OccupancyPublisherConfig config;

 protected:
  void publish(uint64_t timestamp_ns,
               const Eigen::Isometry3d& world_T_sensor,
               const LayerT& layer) const;

 private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  mutable typename LayerT::Ptr layer_;
};

struct Bounds {
  Eigen::Vector2f x_min = Eigen::Vector2f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector2f x_max =
      Eigen::Vector2f::Constant(std::numeric_limits<float>::lowest());
  Eigen::Vector2f dims = Eigen::Vector2f::Zero();
};

template <typename BlockT>
Bounds getLayerBounds(const spatial_hash::VoxelLayer<BlockT>& layer) {
  Bounds bounds;

  for (const auto& block : layer) {
    const auto lower = block.origin();
    const auto upper = lower + spatial_hash::Point::Constant(block.block_size);
    bounds.x_min = bounds.x_min.array().min(lower.template head<2>().array());
    bounds.x_max = bounds.x_max.array().max(upper.template head<2>().array());
  }

  bounds.dims = (bounds.x_max - bounds.x_min) / layer.voxel_size;
  return bounds;
}

template <typename BlockT>
void initGrid(const spatial_hash::VoxelLayer<BlockT>& layer,
              const Bounds& bounds,
              double height,
              nav_msgs::msg::OccupancyGrid& msg) {
  msg.info.resolution = layer.voxel_size;
  msg.info.width = std::ceil(bounds.dims.x());
  msg.info.height = std::ceil(bounds.dims.y());
  msg.info.origin.position.x = bounds.x_min.x();
  msg.info.origin.position.y = bounds.x_min.y();
  msg.info.origin.position.z = height;
  msg.info.origin.orientation.w = 1.0;
  msg.data.resize(msg.info.width * msg.info.height, -1);
}

template <typename BlockT>
void fillOccupancySlice(const OccupancyPublisherConfig& config,
                        const spatial_hash::VoxelLayer<BlockT>& layer,
                        const Eigen::Isometry3d& world_T_sensor,
                        const Bounds& bounds,
                        double height,
                        nav_msgs::msg::OccupancyGrid& msg) {
  const spatial_hash::Point slice_pos(0, 0, height);
  const auto slice_key = layer.getVoxelKey(slice_pos);
  auto bbox = config.add_robot_footprint
                  ? spark_dsg::BoundingBox(config.footprint_min, config.footprint_max)
                  : spark_dsg::BoundingBox();
  const Eigen::Isometry3f sensor_T_world = world_T_sensor.inverse().cast<float>();

  for (const auto block_ptr :
       layer.blocksWithCondition([&slice_key](const BlockT& block) {
         return block.index.z() == slice_key.first.z();
       })) {
    for (size_t x = 0; x < block_ptr->voxels_per_side; ++x) {
      for (size_t y = 0; y < block_ptr->voxels_per_side; ++y) {
        const spatial_hash::VoxelIndex voxel_index(x, y, slice_key.second.z());
        const auto& voxel = block_ptr->getVoxel(voxel_index);
        const Eigen::Vector3f pos = block_ptr->getVoxelPosition(voxel_index);
        const auto rel_pos = pos.head<2>() - bounds.x_min;
        // pos is center point, so we want floor
        const auto r = std::floor(rel_pos.y() / layer.voxel_size);
        const auto c = std::floor(rel_pos.x() / layer.voxel_size);
        const size_t index = r * msg.info.width + c;

        if (config.add_robot_footprint &&
            bbox.contains((sensor_T_world * pos).eval())) {
          msg.data[index] = 0;
          continue;
        }

        if (!voxel_traits::isObserved(voxel, config.min_observation_weight)) {
          msg.data[index] = -2;
          continue;
        }

        const auto occupied = voxel_traits::getDistance(voxel) < config.min_distance;
        if (occupied) {
          msg.data[index] = 100;
          continue;
        }

        if (msg.data[index] == -1) {
          // we only can mark cells as free if they haven't been touched
          msg.data[index] = 0;
        }
      }
    }
  }
}

template <typename BlockT>
void fillOccupancy(const OccupancyPublisherConfig& config,
                   const spatial_hash::VoxelLayer<BlockT>& layer,
                   const Eigen::Isometry3d& world_T_sensor,
                   nav_msgs::msg::OccupancyGrid& msg) {
  const auto bounds = getLayerBounds(layer);
  auto height = config.slice_height;
  if (config.use_relative_height) {
    height += world_T_sensor.translation().z();
  }

  initGrid(layer, bounds, height, msg);
  for (size_t i = 0; i < config.num_slices; ++i) {
    const auto curr_height = height + i * layer.voxel_size;
    fillOccupancySlice(config, layer, world_T_sensor, bounds, curr_height, msg);
  }

  // clean up all cells that were marked unobserved
  for (size_t i = 0; i < msg.data.size(); ++i) {
    if (msg.data[i] == -2) {
      msg.data[i] = -1;
    }
  }
}

template <typename BlockT>
void collate(const spatial_hash::VoxelLayer<BlockT>& layer_in,
             spatial_hash::VoxelLayer<BlockT>& layer_out,
             double min_observation_weight) {
  for (const auto& block : layer_in) {
    bool unobserved = true;
    for (size_t i = 0; i < block.numVoxels(); ++i) {
      if (voxel_traits::isObserved(block.getVoxel(i), min_observation_weight)) {
        unobserved = false;
        break;
      }
    }

    if (unobserved) {
      continue;
    }

    auto new_block = layer_out.allocateBlockPtr(block.index);
    new_block->updated = block.updated;
    for (size_t i = 0; i < block.numVoxels(); ++i) {
      const auto& voxel = block.getVoxel(i);
      if (voxel_traits::isObserved(voxel, min_observation_weight)) {
        new_block->getVoxel(i) = voxel;
      }
    }
  }
}

template <typename T>
void OccupancyPublisher<T>::publish(uint64_t timestamp_ns,
                                    const Eigen::Isometry3d& world_T_sensor,
                                    const OccupancyPublisher<T>::LayerT& layer) const {
  if (config.collate) {
    if (!layer_) {
      layer_.reset(new LayerT(layer.voxel_size, layer.voxels_per_side));
    }

    collate(layer, *layer_, config.min_observation_weight);
  }

  if (!pub_->get_subscription_count()) {
    return;
  }

  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = GlobalInfo::instance().getFrames().map;
  msg.header.stamp = rclcpp::Time(timestamp_ns);
  msg.info.map_load_time = msg.header.stamp;

  const auto& layer_to_use = config.collate ? *layer_ : layer;
  fillOccupancy(config, layer_to_use, world_T_sensor, msg);
  pub_->publish(msg);
}

}  // namespace hydra
