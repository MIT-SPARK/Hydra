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
#include <hydra_visualizer/drawing.h>
#include <spatial_hash/voxel_layer.h>

#include <tf2_eigen/tf2_eigen.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace hydra {

using MarkerMsg = visualization_msgs::msg::Marker;

struct VoxelSliceConfig {
  enum Axis {
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
  } axis = Z_AXIS;
  double slice_offset = 0.0;
  bool use_relative_offset = true;
};

void declare_config(VoxelSliceConfig& config);

struct VoxelZSliceConfig {
  double slice_height = 0.0;
  bool use_relative_height = true;
};

template <typename Voxel>
using Colormap = std::function<std_msgs::msg::ColorRGBA(const Voxel&)>;

template <typename Voxel>
using Filter = std::function<bool(const Voxel&)>;

// adapted from khronos
template <typename Voxel, typename Block>
MarkerMsg drawVoxelSlice(const VoxelSliceConfig& config,
                         const std_msgs::msg::Header& header,
                         const spatial_hash::VoxelLayer<Block>& layer,
                         const Eigen::Isometry3d& world_T_sensor,
                         const Filter<Voxel>& observed,
                         const Colormap<Voxel>& colormap,
                         const std::string& ns) {
  int axis;
  switch (config.axis) {
    case VoxelSliceConfig::X_AXIS:
      axis = 0;
      break;
    case VoxelSliceConfig::Y_AXIS:
      axis = 1;
      break;
    case VoxelSliceConfig::Z_AXIS:
      axis = 2;
      break;
    default:
      throw std::runtime_error("Invalid enum value!");
  }

  MarkerMsg msg;
  msg.header = header;
  msg.action = MarkerMsg::ADD;
  msg.id = 0;
  msg.ns = ns;
  msg.type = MarkerMsg::CUBE_LIST;
  msg.scale.x = layer.voxel_size;
  msg.scale.y = layer.voxel_size;
  msg.scale.z = layer.voxel_size;

  auto offset = config.slice_offset;
  if (config.use_relative_offset) {
    offset += world_T_sensor.translation()(axis);
  }

  spatial_hash::Point slice_pos = spatial_hash::Point::Zero();
  slice_pos(axis) = offset;

  const auto slice_index = layer.getBlockIndex(slice_pos);
  const auto origin =
      spatial_hash::originPointFromIndex(slice_index, layer.blockSize());
  const auto grid_index = spatial_hash::indexFromPoint<spatial_hash::VoxelIndex>(
      slice_pos - origin, layer.voxel_size_inv);

  for (const auto& block : layer) {
    if (block.index(axis) != slice_index(axis)) {
      continue;
    }

    for (size_t i = 0; i < block.voxels_per_side; ++i) {
      for (size_t j = 0; j < block.voxels_per_side; ++j) {
        spatial_hash::VoxelIndex voxel_idx;
        switch (config.axis) {
          case VoxelSliceConfig::X_AXIS:
            voxel_idx << grid_index(axis), i, j;
            break;
          case VoxelSliceConfig::Y_AXIS:
            voxel_idx << i, grid_index(axis), j;
            break;
          case VoxelSliceConfig::Z_AXIS:
            voxel_idx << i, j, grid_index(axis);
            break;
        }

        const auto& voxel = block.getVoxel(voxel_idx);
        if (!observed(voxel)) {
          continue;
        }

        const auto pos = block.getVoxelPosition(voxel_idx);
        tf2::convert(pos.template cast<double>().eval(), msg.points.emplace_back());
        msg.colors.push_back(colormap(voxel));
      }
    }
  }

  return msg;
}

template <typename Voxel, typename Block>
MarkerMsg drawVoxelSlice(const VoxelZSliceConfig& config,
                         const std_msgs::msg::Header& header,
                         const spatial_hash::VoxelLayer<Block>& layer,
                         const Eigen::Isometry3d& world_T_sensor,
                         const Filter<Voxel>& observed,
                         const Colormap<Voxel>& colormap,
                         const std::string& ns) {
  return drawVoxelSlice(
      {VoxelSliceConfig::Axis::Z_AXIS, config.slice_height, config.use_relative_height},
      header,
      layer,
      world_T_sensor,
      observed,
      colormap,
      ns);
}

template <typename Block>
using BlockColoring = std::function<std_msgs::msg::ColorRGBA(const Block&)>;

struct ActiveBlockColoring {
  explicit ActiveBlockColoring(const spark_dsg::Color& active_color);

  std_msgs::msg::ColorRGBA call(const spatial_hash::Block& block) const;

  template <typename Block>
  BlockColoring<Block> getCallback() const {
    return [this](const auto& block) { return this->call(block); };
  }

  const spark_dsg::Color active_color;
};

template <typename Block>
MarkerMsg drawSpatialGrid(const spatial_hash::BlockLayer<Block>& layer,
                          double scale,
                          const std::string& ns,
                          double alpha = 1.0,
                          const BlockColoring<Block>& cmap = {}) {
  visualization_msgs::msg::Marker marker;
  marker.type = MarkerMsg::LINE_LIST;
  marker.action = MarkerMsg::ADD;
  marker.id = 0;
  marker.ns = ns;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;

  Eigen::Vector3d identity_pos = Eigen::Vector3d::Zero();
  tf2::convert(identity_pos, marker.pose.position);
  tf2::convert(Eigen::Quaterniond::Identity(), marker.pose.orientation);

  for (const auto& block : layer) {
    const auto position = block.position();
    spark_dsg::BoundingBox box(Eigen::Vector3f::Constant(block.block_size), position);
    std_msgs::msg::ColorRGBA color;
    if (cmap) {
      color = cmap(block);
    }

    color.a = alpha;
    visualizer::drawBoundingBox(box, color, marker);
  }

  return marker;
}

}  // namespace hydra
