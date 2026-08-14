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
#include "hydra_visualizer/plugins/traversability_plugin.h"

#include <config_utilities/config_utilities.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include "hydra_visualizer/color/color_parsing.h"

namespace hydra {
namespace {

static const auto registration_ =
    config::RegistrationWithConfig<VisualizerPlugin,
                                   TraversabilityPlugin,
                                   TraversabilityPlugin::Config,
                                   ianvs::NodeHandle,
                                   std::string>("TraversabilityPlugin");

}

using spark_dsg::DynamicSceneGraph;
using spark_dsg::SceneGraphLayer;
using spark_dsg::TraversabilityNodeAttributes;
using spark_dsg::TraversabilityState;
using spark_dsg::TravNodeAttributes;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

void declare_config(TraversabilityPlugin::Config& config) {
  using namespace config;
  name("TraversabilityPlugin::Config");
  field(config.layer, "layer");
  field(config.colors, "colors");
  field(config.slice_height, "slice_height", "m");
  field(config.line_width, "line_width", "m");
  checkCondition(config.colors.size() == 4,
                 "colors.size() must be 4 colors for each traversability type");
  check(config.line_width, GT, 0.0f, "line_width");
}

TraversabilityPlugin::TraversabilityPlugin(const Config& config,
                                           ianvs::NodeHandle nh,
                                           const std::string& name)
    : VisualizerPlugin(name),
      config_(name, config::checkValid(config)),
      pub_(nh.create_publisher<MarkerArray>(name, rclcpp::QoS(1).transient_local())) {}

void TraversabilityPlugin::draw(const std_msgs::msg::Header& header,
                                const DynamicSceneGraph& graph) {
  if (pub_->get_subscription_count() == 0) {
    return;
  }

  MarkerArray msg;
  fillMarkers(header, graph, msg);
  tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    pub_->publish(msg);
  }
}

void TraversabilityPlugin::reset(const std_msgs::msg::Header& header) {
  MarkerArray msg;
  tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    pub_->publish(msg);
  }
}

void TraversabilityPlugin::fillMarkers(const std_msgs::msg::Header& header,
                                       const DynamicSceneGraph& graph,
                                       MarkerArray& msg) const {
  const auto config = config_.get();
  auto layer = graph.findLayer(config.layer);
  if (!layer) {
    return;
  }

  drawBoundaries(config, header, *layer, msg);
}

void TraversabilityPlugin::drawBoundaries(const Config& config,
                                          const std_msgs::msg::Header& header,
                                          const SceneGraphLayer& layer,
                                          MarkerArray& msg) const {
  size_t id = 0;
  Marker marker;
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.action = Marker::ADD;
  marker.ns = "boundaries";
  marker.pose.orientation.w = 1.0;
  marker.scale.x = config.line_width;
  marker.scale.y = config.line_width;
  marker.scale.z = config.line_width;
  for (const auto& [node_id, node] : layer.nodes()) {
    // Reset the marker.
    marker.id = id++;
    marker.points.clear();
    marker.colors.clear();

    auto block_attrs = node->tryAttributes<TraversabilityNodeAttributes>();
    if (block_attrs) {
      drawBlockBoundary(config, *block_attrs, marker);
    } else {
      auto region_attrs = node->tryAttributes<TravNodeAttributes>();
      if (region_attrs) {
        drawRegionBoundary(config, *region_attrs, marker);
      }
    }

    tracker_.add(marker, msg);
  }
}

void TraversabilityPlugin::drawBlockBoundary(const Config& config,
                                             const TraversabilityNodeAttributes& attrs,
                                             Marker& marker) const {
  // Get the world frame positions of the boundary points, adjusted for the line width
  // for non-overlapping rendering. bot-right, bot-left, top-left, top-right
  std::vector<Eigen::Vector3d> pts;
  pts.reserve(4);
  pts.emplace_back(attrs.boundary.max.x() - config.line_width,
                   attrs.boundary.min.y() + config.line_width,
                   0);
  pts.emplace_back(attrs.boundary.min.x() + config.line_width,
                   attrs.boundary.min.y() + config.line_width,
                   0);
  pts.emplace_back(attrs.boundary.min.x() + config.line_width,
                   attrs.boundary.max.y() - config.line_width,
                   0);
  pts.emplace_back(attrs.boundary.max.x() - config.line_width,
                   attrs.boundary.max.y() - config.line_width,
                   0);
  for (auto& point : pts) {
    point += attrs.position;
    point.z() = config.slice_height;
  }

  // Draw the boundary points as a line segment, where individual states break up the
  // line in equal parts if present.
  for (size_t i = 0; i < 4; ++i) {
    const auto& states = attrs.boundary.states[i];
    const size_t start = state_pairs_[i].first;
    const size_t end = state_pairs_[i].second;
    tf2::convert(pts[start], marker.points.emplace_back());  // First point.

    if (states.empty()) {
      // Single unknown boundary.
      addBoundaryPoint(config, marker, pts[end], TraversabilityState::UNKNOWN, true);
      continue;
    }

    // Add line segments for continuous states.
    auto current_state = states[0];
    for (size_t j = 1; j < states.size(); ++j) {
      if (states[j] != current_state) {
        const double fraction = static_cast<double>(j) / (states.size() - 1);
        Eigen::Vector3d segment_point =
            pts[start] * (1.0 - fraction) + pts[end] * fraction;
        addBoundaryPoint(config, marker, segment_point, current_state);
        current_state = states[j];
      }
    }
    addBoundaryPoint(config, marker, pts[end], current_state, true);
  }
}

void TraversabilityPlugin::drawRegionBoundary(const Config& config,
                                              const TravNodeAttributes& attrs,
                                              Marker& marker) const {
  marker.type = Marker::LINE_STRIP;
  for (size_t i = 0; i < attrs.radii.size(); ++i) {
    tf2::convert(attrs.getBoundaryPoint(i), marker.points.emplace_back());
    marker.colors.emplace_back(
        visualizer::makeColorMsg(config.colors[static_cast<size_t>(attrs.states[i])]));
  }
  // Close the circle.
  if (!attrs.radii.empty()) {
    marker.points.emplace_back(marker.points.front());
    marker.colors.emplace_back(marker.colors.front());
  }
}

void TraversabilityPlugin::addBoundaryPoint(const Config& config,
                                            Marker& marker,
                                            const Eigen::Vector3d& point,
                                            TraversabilityState state,
                                            bool last_point) const {
  tf2::convert(point, marker.points.emplace_back());
  marker.colors.emplace_back(
      visualizer::makeColorMsg(config.colors[static_cast<size_t>(state)]));
  // Duplicate the point and color to create a line segment.
  marker.colors.emplace_back(marker.colors.back());
  if (!last_point) {
    marker.points.emplace_back(marker.points.back());
  }
}

YAML::Node TraversabilityPlugin::dumpConfig() const {
  auto root = config::toYaml(config_.get());
  root["type"] = "TraversabilityPlugin";
  return root;
}

}  // namespace hydra
