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
#include "hydra_visualizer/plugins/footprint_plugin.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/node_attributes.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include "hydra_visualizer/color/colormap_utilities.h"
#include "hydra_visualizer/utils/polygon_utilities.h"

namespace hydra {
namespace {

static const auto registration_ =
    config::RegistrationWithConfig<VisualizerPlugin,
                                   FootprintPlugin,
                                   FootprintPlugin::Config,
                                   ianvs::NodeHandle,
                                   std::string>("FootprintPlugin");
}

using spark_dsg::DynamicSceneGraph;
using spark_dsg::LayerId;
using spark_dsg::PlaceNodeAttributes;
using spark_dsg::SemanticNodeAttributes;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

void declare_config(FootprintPlugin::Config& config) {
  using namespace config;
  name("FootprintPlugin::Config");
  field(config.use_place_radius, "use_place_radius");
  field(config.draw_boundaries, "draw_boundaries");
  field(config.draw_boundary_vertices, "draw_boundary_vertices");
  field(config.line_width, "line_width");
  field(config.line_alpha, "line_alpha");
  field(config.mesh_alpha, "mesh_alpha");
  field(config.footprint_radius, "footprint_radius");
  field(config.num_samples, "num_samples");
  field(config.layer, "layer");

  check(config.line_width, GT, 0.0, "line_width");
  check(config.line_alpha, GT, 0.0, "line_alpha");
}

FootprintPlugin::FootprintPlugin(const Config& config,
                                 ianvs::NodeHandle nh,
                                 const std::string& name)
    : VisualizerPlugin(name),
      config(config::checkValid(config)),
      pub_(nh.create_publisher<MarkerArray>(name, rclcpp::QoS(1).transient_local())) {}

void FootprintPlugin::draw(const std_msgs::msg::Header& header,
                           const DynamicSceneGraph& graph) {
  MarkerArray markers;
  markers.markers.resize(
      config.draw_boundaries ? (config.draw_boundary_vertices ? 3 : 2) : 1);

  std::string ns = config.layer + "_footprints";
  markers.markers[0].header = header;
  markers.markers[0].ns = ns + "_polygons";
  markers.markers[0].id = 0;
  markers.markers[0].type = Marker::TRIANGLE_LIST;
  markers.markers[0].action = Marker::ADD;
  markers.markers[0].pose.orientation.w = 1.0;
  markers.markers[0].color.a = config.mesh_alpha;
  markers.markers[0].scale.x = 1.0;
  markers.markers[0].scale.y = 1.0;
  markers.markers[0].scale.z = 1.0;

  if (config.draw_boundaries) {
    markers.markers[1].header = header;
    markers.markers[1].ns = ns + "_boundaries";
    markers.markers[1].id = 0;
    markers.markers[1].type = Marker::LINE_LIST;
    markers.markers[1].action = Marker::ADD;
    markers.markers[1].pose.orientation.w = 1.0;
    markers.markers[1].scale.x = config.line_width;
    markers.markers[1].scale.y = 0.0;
    markers.markers[1].scale.z = 0.0;

    if (config.draw_boundary_vertices) {
      markers.markers[2].header = header;
      markers.markers[2].ns = ns + "_vertices";
      markers.markers[2].id = 0;
      markers.markers[2].type = Marker::SPHERE_LIST;
      markers.markers[2].action = Marker::ADD;
      markers.markers[2].pose.orientation.w = 1.0;
      markers.markers[2].scale.x = config.line_width;
      markers.markers[2].scale.y = config.line_width;
      markers.markers[2].scale.z = config.line_width;
    }
  }

  const auto& layer = graph.getLayer(config.layer);
  for (auto&& [id, node] : layer.nodes()) {
    const auto mean_z = getMeanNeighborHeight(layer, *node);
    const auto& attrs = node->attributes<SemanticNodeAttributes>();

    auto color = visualizer::makeColorMsg(attrs.color);
    color.a = config.line_alpha;

    double radius = config.footprint_radius;
    if (config.use_place_radius) {
      radius = node->attributes<PlaceNodeAttributes>().distance;
    }

    const auto footprint = getCirclePolygon(*node, radius, config.num_samples);

    auto mesh_color = color;
    mesh_color.a = config.mesh_alpha;
    makeFilledPolygon(footprint, mesh_color, markers.markers[0], mean_z);
    if (config.draw_boundaries) {
      makePolygonBoundary(
          footprint,
          color,
          markers.markers[1],
          mean_z,
          config.draw_boundary_vertices ? &markers.markers[2] : nullptr);
    }
  }

  MarkerArray msg;
  tracker_.add(markers, msg);
  tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    pub_->publish(msg);
  }
}

void FootprintPlugin::reset(const std_msgs::msg::Header& header) {
  MarkerArray msg;
  tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    pub_->publish(msg);
  }
}

YAML::Node FootprintPlugin::dumpConfig() const {
  auto root = config::toYaml(config);
  root["type"] = "FootprintPlugin";
  return root;
}

}  // namespace hydra
