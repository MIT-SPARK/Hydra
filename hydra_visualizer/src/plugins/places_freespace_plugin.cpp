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
#include "hydra_visualizer/plugins/places_freespace_plugin.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/node_attributes.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include "hydra_visualizer/color/color_parsing.h"
#include "hydra_visualizer/drawing.h"

namespace hydra {
namespace {

inline static const auto registration_ =
    config::RegistrationWithConfig<VisualizerPlugin,
                                   PlacesFreespacePlugin,
                                   PlacesFreespacePlugin::Config,
                                   ianvs::NodeHandle,
                                   std::string>("PlacesFreespacePlugin");

}

using spark_dsg::Color;
using spark_dsg::DsgLayers;
using spark_dsg::DynamicSceneGraph;
using spark_dsg::PlaceNodeAttributes;
using spark_dsg::SceneGraphLayer;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

void declare_config(PlacesFreespacePlugin::Config& config) {
  using namespace config;
  name("PlacesFreespacePlugin::Config");
  field(config.draw_edges, "draw_edges");
  field(config.sphere_color, "sphere_color");
  field(config.sphere_alpha, "sphere_alpha");
  field(config.graph, "graph");
}

PlacesFreespacePlugin::PlacesFreespacePlugin(const Config& config,
                                             ianvs::NodeHandle nh,
                                             const std::string& name)
    : VisualizerPlugin(name),
      config_(name, config::checkValid(config)),
      pub_(nh.create_publisher<MarkerArray>(name, rclcpp::QoS(1).transient_local())) {}

void PlacesFreespacePlugin::draw(const std_msgs::msg::Header& header,
                                 const DynamicSceneGraph& graph) {
  if (!pub_->get_subscription_count()) {
    return;
  }

  MarkerArray msg;
  fillMarkers(header, graph, msg);
  tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    pub_->publish(msg);
  }
}

void PlacesFreespacePlugin::reset(const std_msgs::msg::Header& header) {
  MarkerArray msg;
  tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    pub_->publish(msg);
  }
}

void PlacesFreespacePlugin::fillMarkers(const std_msgs::msg::Header& header,
                                        const DynamicSceneGraph& graph,
                                        MarkerArray& msg) const {
  if (!graph.hasLayer(DsgLayers::PLACES)) {
    return;
  }

  const auto config = config_.get();
  const auto& places = graph.getLayer(DsgLayers::PLACES);
  if (!config.graph.visualize) {
    return;
  }

  tracker_.add(makeLayerNodeMarkers(header, config.graph, places, "nodes"), msg);
  if (config.draw_edges) {
    tracker_.add(makeLayerEdgeMarkers(header, config.graph, places, "edges"), msg);
  }

  drawSpheres(config, header, places, msg);
}

void PlacesFreespacePlugin::drawSpheres(const Config& config,
                                        const std_msgs::msg::Header& header,
                                        const SceneGraphLayer& layer,
                                        MarkerArray& msg) const {
  size_t id = 0;
  for (const auto& id_node_pair : layer.nodes()) {
    const auto& attrs = id_node_pair.second->attributes<PlaceNodeAttributes>();

    Marker marker;
    marker.header = header;
    marker.type = Marker::SPHERE;
    marker.action = Marker::ADD;
    marker.id = id;
    marker.ns = "places_spheres";

    marker.scale.x = 2 * attrs.distance;
    marker.scale.y = 2 * attrs.distance;
    marker.scale.z = 2 * attrs.distance;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    tf2::convert(id_node_pair.second->attributes().position, marker.pose.position);

    marker.color = visualizer::makeColorMsg(config.sphere_color);
    marker.color.a = config.sphere_alpha;
    tracker_.add(marker, msg);
    ++id;
  }
}

YAML::Node PlacesFreespacePlugin::dumpConfig() const {
  auto root = config::toYaml(config_.get());
  root["type"] = "PlacesFreespacePlugin";
  return root;
}

}  // namespace hydra
