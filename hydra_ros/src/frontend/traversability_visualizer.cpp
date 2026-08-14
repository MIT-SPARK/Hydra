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
#include "hydra_ros/frontend/traversability_visualizer.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <hydra/common/global_info.h>
#include <hydra_visualizer/color/color_parsing.h>
#include <hydra_visualizer/color/colormap_utilities.h>
#include <spark_dsg/colormaps.h>

namespace hydra {

using spark_dsg::Color;
using visualization_msgs::msg::Marker;
using visualizer::RangeColormap;

void declare_config(TraversabilityVisualizer::Config& config) {
  using namespace config;
  name("TraversabilityVisualizer::Config");
  field(config.ns, "ns");
  field(config.traversability_colormap, "traversability_colormap");
  field(config.confidence_colormap, "confidence_colormap");
  field(config.state_colors, "state_colors");
  field(config.drawing_offset_z, "drawing_offset_z", "m");
  field(config.use_relative_offset, "use_relative_offset");
  check(config.state_colors.size(), EQ, 4, "state_colors");
}

TraversabilityVisualizer::TraversabilityVisualizer(const Config& config)
    : config_("traversability_visualizer",
              config::checkValid(config),
              [this]() { onConfigUpdate(); }),
      nh_(ianvs::NodeHandle::this_node(config.ns)),
      layer_pub_(nh_.create_publisher<visualization_msgs::msg::Marker>("layer", 100)) {
  onConfigUpdate();
}

visualizer::RangeColormap::Config TraversabilityVisualizer::createSpectrumColormap(
    std::vector<spark_dsg::Color> colors) {
  visualizer::RangeColormap::Config config;
  config.palette = visualizer::SpectrumPalette::Config{std::move(colors)};
  return config;
}

std::string TraversabilityVisualizer::printInfo() const {
  return config::toString(active_config_);
}

void TraversabilityVisualizer::onConfigUpdate() {
  std::lock_guard<std::mutex> lock(config_mutex_);
  active_config_ = config_.get();
  traversability_colormap_ = std::make_unique<visualizer::RangeColormap>(
      active_config_.traversability_colormap);
  confidence_colormap_ =
      std::make_unique<visualizer::RangeColormap>(active_config_.confidence_colormap);
}

void TraversabilityVisualizer::call(uint64_t timestamp_ns,
                                    const Eigen::Vector3d& world_t_body,
                                    const places::TraversabilityLayer& layer) const {
  std_msgs::msg::Header header;
  header.frame_id = GlobalInfo::instance().getFrames().map;
  header.stamp = rclcpp::Time(timestamp_ns);
  std::lock_guard<std::mutex> lock(config_mutex_);
  visualizeLayer(header, world_t_body, layer);
}

void TraversabilityVisualizer::visualizeLayer(
    const std_msgs::msg::Header& header,
    const Eigen::Vector3d& world_t_body,
    const places::TraversabilityLayer& layer) const {
  visualization_msgs::msg::Marker msg;
  msg.header = header;
  msg.action = visualization_msgs::msg::Marker::ADD;
  msg.id = 0;
  msg.ns = "traversability";
  msg.type = visualization_msgs::msg::Marker::CUBE_LIST;
  msg.scale.x = layer.voxel_size;
  msg.scale.y = layer.voxel_size;
  msg.scale.z = layer.voxel_size;
  msg.pose.orientation.w = 1.0;

  visualization_msgs::msg::Marker msg2 = msg;
  msg2.ns = "confidence";

  visualization_msgs::msg::Marker msg3 = msg;
  msg3.ns = "state";

  visualization_msgs::msg::Marker msg4 = msg;
  msg4.ns = "debug";

  auto height = active_config_.drawing_offset_z;
  if (active_config_.use_relative_offset) {
    height += world_t_body.z();
  }
  geometry_msgs::msg::Point pos;
  pos.z = height;

  for (const auto& block : layer) {
    for (size_t x = 0; x < block.voxels_per_side; ++x) {
      pos.x = block.origin().x() + (x + 0.5f) * layer.voxel_size;
      for (size_t y = 0; y < block.voxels_per_side; ++y) {
        const auto& voxel = block.voxel(x, y);
        if (voxel.confidence <= 0.0f) {
          continue;  // Unobserved voxels.
        }
        pos.y = block.origin().y() + (y + 0.5f) * layer.voxel_size;
        if (voxel.height) {
          pos.z = *voxel.height;
        }
        msg.points.push_back(pos);
        msg.colors.push_back(visualizer::makeColorMsg(
            traversability_colormap_->getColor(voxel.traversability),
            active_config_.alpha));
        msg2.colors.push_back(visualizer::makeColorMsg(
            confidence_colormap_->getColor(voxel.confidence), active_config_.alpha));
        msg3.colors.push_back(visualizer::makeColorMsg(
            active_config_.state_colors[static_cast<uint8_t>(voxel.state)],
            active_config_.alpha));
        msg4.colors.push_back(visualizer::makeColorMsg(debugColor(voxel.debug_value),
                                                       active_config_.alpha));
      }
    }
  }
  msg2.points = msg.points;
  msg3.points = msg.points;
  msg4.points = msg.points;

  layer_pub_->publish(msg);
  layer_pub_->publish(msg2);
  layer_pub_->publish(msg3);
  layer_pub_->publish(msg4);
}

Color TraversabilityVisualizer::debugColor(float value) const {
  // Connected components.
  if (value < 0.0f) {
    return Color::black();
  }

  return spark_dsg::colormaps::rainbowId(static_cast<int>(value), 10);
}

}  // namespace hydra
