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
#include "hydra_visualizer/plugins/region_growing_boundary_plugin.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/validation.h>
#include <spark_dsg/node_attributes.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include "hydra_visualizer/color/color_parsing.h"
#include "hydra_visualizer/utils/polygon_utilities.h"

namespace hydra {
namespace {

static const auto registration_ =
    config::RegistrationWithConfig<LayerPlugin,
                                   RegionGrowingBoundaryPlugin,
                                   RegionGrowingBoundaryPlugin::Config,
                                   std::string>("RegionGrowingBoundaryPlugin");

std_msgs::msg::ColorRGBA makeBoundaryColor(const std::vector<spark_dsg::Color>& colors,
                                           spark_dsg::TraversabilityState state,
                                           float alpha) {
  return visualizer::makeColorMsg(colors.at(static_cast<size_t>(state)), alpha);
}

}  // namespace

using visualization_msgs::msg::Marker;

void declare_config(RegionGrowingBoundaryPlugin::Config& config) {
  using namespace config;
  name("RegionGrowingBoundaryPlugin::Config");
  field(config.use_node_color, "use_node_color");
  field(config.colors, "colors");
  field(config.line_width, "line_width");
  field(config.fill_alpha, "fill_alpha");
  field(config.fill_boundaries, "fill_boundaries");
  checkCondition(config.colors.size() == 4, "colors.size() must be 4");
  check(config.line_width, GT, 0.0f, "line_width");
  checkInRange(config.fill_alpha, 0.0f, 1.0f, "fill_alpha", false);
}

RegionGrowingBoundaryPlugin::RegionGrowingBoundaryPlugin(const Config& config,
                                                         const std::string& ns)
    : ns_(ns),
      config_(ns + "_region_growing_plugin", config, [this]() { has_change_ = true; }) {
}

void RegionGrowingBoundaryPlugin::draw(const std_msgs::msg::Header& header,
                                       const visualizer::LayerInfo& info,
                                       const spark_dsg::SceneGraphLayer& layer,
                                       const spark_dsg::Mesh*,
                                       visualization_msgs::msg::MarkerArray& msg,
                                       MarkerTracker& tracker) {
  const auto config = config_.get();

  Marker marker;
  marker.id = 0;
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.action = Marker::ADD;
  marker.ns = ns_ + "_region_growing_boundaries";
  marker.pose.orientation.w = 1.0;
  marker.scale.x = config.line_width;
  marker.scale.y = config.line_width;
  marker.scale.z = config.line_width;

  Marker fill_marker;
  fill_marker.header = header;
  fill_marker.ns = ns_ + "_region_growing_polygons";
  fill_marker.id = 0;
  fill_marker.type = Marker::TRIANGLE_LIST;
  fill_marker.action = Marker::ADD;
  fill_marker.pose.orientation.w = 1.0;
  fill_marker.color.a = config.fill_alpha;
  fill_marker.scale.x = 1.0;
  fill_marker.scale.y = 1.0;
  fill_marker.scale.z = 1.0;

  for (const auto& [node_id, node] : layer.nodes()) {
    if (info.filter && !info.filter(*node)) {
      continue;
    }

    auto attrs = node->tryAttributes<spark_dsg::TravNodeAttributes>();
    if (!attrs || attrs->radii.empty()) {
      continue;
    }

    Eigen::MatrixXd points(3, attrs->radii.size());
    const auto color =
        visualizer::makeColorMsg(info.node_color(*node), info.config.nodes.alpha);
    for (size_t i = 1; i <= attrs->radii.size(); ++i) {
      const auto start_idx = i - 1;
      const auto end_idx = i % attrs->radii.size();
      auto start = attrs->getBoundaryPoint(start_idx);
      start.z() += info.z_offset;
      points.col(start_idx) = start;

      tf2::convert(start, marker.points.emplace_back());
      auto end = attrs->getBoundaryPoint(end_idx);
      end.z() += info.z_offset;
      tf2::convert(end, marker.points.emplace_back());
      if (config.use_node_color) {
        marker.colors.emplace_back(color);
        marker.colors.emplace_back(color);
      } else {
        const auto start_color = makeBoundaryColor(
            config.colors, attrs->states[start_idx], info.config.nodes.alpha);
        const auto end_color = makeBoundaryColor(
            config.colors, attrs->states[end_idx], info.config.nodes.alpha);
        marker.colors.emplace_back(start_color);
        marker.colors.emplace_back(end_color);
      }
    }

    if (config.fill_boundaries) {
      auto mesh_color = color;
      mesh_color.a = config.fill_alpha;
      makeFilledPolygon(points, mesh_color, fill_marker);
    }
  }

  tracker.add(marker, msg);
  if (config.fill_boundaries) {
    tracker.add(fill_marker, msg);
  }
}

YAML::Node RegionGrowingBoundaryPlugin::dumpConfig() const {
  auto root = config::toYaml(config_.get());
  root["type"] = "RegionGrowingBoundaryPlugin";
  return root;
}

}  // namespace hydra
