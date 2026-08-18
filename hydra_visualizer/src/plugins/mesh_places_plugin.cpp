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
#include "hydra_visualizer/plugins/mesh_places_plugin.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/validation.h>
#include <spark_dsg/node_attributes.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include "hydra_visualizer/color/color_parsing.h"

using hydra::visualizer::LayerInfo;
using hydra::visualizer::makeColorMsg;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using namespace spark_dsg;

namespace hydra {
namespace {

static const auto registration_ =
    config::RegistrationWithConfig<LayerPlugin,
                                   MeshPlacesPlugin,
                                   MeshPlacesPlugin::Config,
                                   std::string>("MeshPlacesPlugin");

Marker makeLayerEllipseBoundaries(const MeshPlacesPlugin::Config& config,
                                  const std_msgs::msg::Header& header,
                                  const LayerInfo& info,
                                  const SceneGraphLayer& layer,
                                  const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = ns;
  marker.scale.x = config.wireframe_scale;
  marker.pose.position.z += config.collapse ? 0.0 : info.z_offset;

  std_msgs::msg::ColorRGBA color;
  geometry_msgs::msg::Point last_point;
  for (const auto& [node_id, node] : layer.nodes()) {
    const auto attrs = node->tryAttributes<Place2dNodeAttributes>();
    if (!attrs || attrs->boundary.size() <= 1) {
      // TODO(nathan) log warning
      continue;
    }

    color = makeColorMsg(info.node_color(*node), config.ellipse_alpha);
    const auto pos = attrs->position;
    last_point.x = attrs->ellipse_matrix_expand(0, 0) + attrs->ellipse_centroid(0);
    last_point.y = attrs->ellipse_matrix_expand(1, 0) + attrs->ellipse_centroid(1);
    last_point.z = pos.z();

    int npts = 20;
    for (int ix = 1; ix < npts + 1; ++ix) {
      marker.points.push_back(last_point);
      marker.colors.push_back(color);

      float t = ix * 2 * M_PI / npts;
      Eigen::Vector2d p2 =
          attrs->ellipse_matrix_expand * Eigen::Vector2d(cos(t), sin(t));
      last_point.x = p2(0) + attrs->ellipse_centroid(0);
      last_point.y = p2(1) + attrs->ellipse_centroid(1);
      last_point.z = pos.z();

      marker.points.push_back(last_point);
      marker.colors.push_back(color);
    }
  }

  return marker;
}

Marker makeLayerPolygonEdges(const MeshPlacesPlugin::Config& config,
                             const std_msgs::msg::Header& header,
                             const LayerInfo& info,
                             const SceneGraphLayer& layer,
                             const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = ns;
  marker.scale.x = config.wireframe_scale;

  for (const auto& [node_id, node] : layer.nodes()) {
    const auto attrs = node->tryAttributes<Place2dNodeAttributes>();
    if (!attrs || attrs->boundary.size() <= 1) {
      // TODO(nathan) log warning
      continue;
    }

    const auto pos = attrs->position;
    geometry_msgs::msg::Point node_point;
    tf2::convert(pos, node_point);
    node_point.z += info.z_offset;
    const auto color = makeColorMsg(info.node_color(*node), config.alpha);

    for (size_t i = 0; i < attrs->boundary.size(); ++i) {
      geometry_msgs::msg::Point boundary_point;
      tf2::convert(attrs->boundary[i], boundary_point);
      boundary_point.z = pos.z();

      marker.points.push_back(boundary_point);
      marker.colors.push_back(color);
      marker.points.push_back(node_point);
      marker.colors.push_back(color);
    }
  }

  return marker;
}

Marker makeLayerPolygonBoundaries(const MeshPlacesPlugin::Config& config,
                                  const std_msgs::msg::Header& header,
                                  const LayerInfo& info,
                                  const SceneGraphLayer& layer,
                                  const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = ns;
  marker.scale.x = config.wireframe_scale;
  marker.pose.position.z += config.collapse ? 0.0 : info.z_offset;

  for (const auto& [node_id, node] : layer.nodes()) {
    const auto attrs = node->tryAttributes<Place2dNodeAttributes>();
    if (!attrs || attrs->boundary.size() <= 1) {
      continue;
    }

    const auto pos = attrs->position;
    const auto color = config.use_node_color ? info.node_color(*node) : Color();
    const auto color_msg = makeColorMsg(color, config.alpha);

    geometry_msgs::msg::Point last_point;
    tf2::convert(attrs->boundary.back(), last_point);
    last_point.z = pos.z();

    for (size_t i = 0; i < attrs->boundary.size(); ++i) {
      marker.points.push_back(last_point);
      marker.colors.push_back(color_msg);

      tf2::convert(attrs->boundary[i], last_point);
      last_point.z = pos.z();
      marker.points.push_back(last_point);
      marker.colors.push_back(color_msg);
    }
  }

  return marker;
}

}  // namespace

void declare_config(MeshPlacesPlugin::Config& config) {
  using namespace config;
  name("MeshPlacesPlugin::Config");
  field(config.draw, "draw");
  field(config.collapse, "collapse");
  field(config.wireframe_scale, "wireframe_scale");
  field(config.use_node_color, "use_node_color");
  field(config.alpha, "alpha");
  field(config.draw_ellipse, "draw_ellipse");
  field(config.ellipse_alpha, "ellipse_alpha");

  check(config.wireframe_scale, GT, 0.0, "wireframe_scale");
  checkInRange(config.alpha, 0.0, 1.0, "alpha");
  checkInRange(config.ellipse_alpha, 0.0, 1.0, "ellipse_alpha");
}

MeshPlacesPlugin::MeshPlacesPlugin(const Config& config, const std::string& ns)
    : ns_(ns),
      config_(ns + "_mesh_places_plugin", config, [this]() { has_change_ = true; }) {}

void MeshPlacesPlugin::draw(const std_msgs::msg::Header& header,
                            const visualizer::LayerInfo& info,
                            const spark_dsg::SceneGraphLayer& layer,
                            const spark_dsg::Mesh*,
                            visualization_msgs::msg::MarkerArray& msg,
                            MarkerTracker& tracker) {
  const auto config = config_.get();

  if (config.draw) {
    const auto ns = ns_ + "_polygon_boundaries";
    tracker.add(makeLayerPolygonBoundaries(config, header, info, layer, ns), msg);
    if (config.collapse) {
      const auto edge_ns = ns_ + "_polygon_boundaries_edges";
      tracker.add(makeLayerPolygonEdges(config, header, info, layer, edge_ns), msg);
    }
  }

  if (config.draw_ellipse) {
    const auto ns = ns_ + "_ellipsoid_boundaries";
    tracker.add(makeLayerEllipseBoundaries(config, header, info, layer, ns), msg);
  }
}

YAML::Node MeshPlacesPlugin::dumpConfig() const {
  auto root = config::toYaml(config_.get());
  root["type"] = "MeshPlacesPlugin";
  return root;
}

}  // namespace hydra
