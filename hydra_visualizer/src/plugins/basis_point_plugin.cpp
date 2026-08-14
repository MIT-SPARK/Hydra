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
#include "hydra_visualizer/plugins/basis_point_plugin.h"

#include <config_utilities/config_utilities.h>
#include <config_utilities/factory.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/node_attributes.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include "hydra_visualizer/drawing.h"

namespace hydra {
namespace {
static const auto registration =
    config::RegistrationWithConfig<VisualizerPlugin,
                                   BasisPointPlugin,
                                   BasisPointPlugin::Config,
                                   ianvs::NodeHandle,
                                   std::string>("BasisPointPlugin");
}

using spark_dsg::Color;
using spark_dsg::DsgLayers;
using spark_dsg::DynamicSceneGraph;
using spark_dsg::PlaceNodeAttributes;
using spark_dsg::SceneGraphNode;
using spark_dsg::SemanticNodeAttributes;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

void declare_config(BasisPointPlugin::Config& config) {
  using namespace config;
  name("BasisPlugin::Config");
  field(config.show_voxblox_connections, "show_voxblox_connections");
  field(config.draw_basis_points, "draw_basis_points");
  field(config.draw_layer_edges, "draw_layer_edges");
  field(config.places_edge_scale, "places_edge_scale");
  field(config.places_edge_alpha, "places_edge_alpha");
  field(config.basis_point_scale, "basis_point_scale");
  field(config.basis_point_alpha, "basis_point_alpha");
  field(config.colormap, "colormap");
  field(config.graph, "graph");
}

struct BasisPoint {
  Eigen::Vector3d pos;
  Color color;
};

std::vector<BasisPoint> getBasisPoints(const visualizer::CategoricalColormap& cmap,
                                       const PlaceNodeAttributes& attrs,
                                       bool use_voxblox,
                                       const spark_dsg::Mesh* vertices) {
  std::vector<BasisPoint> to_return;
  if (!use_voxblox && !vertices) {
    return to_return;
  }

  if (use_voxblox) {
    for (const auto& info : attrs.voxblox_mesh_connections) {
      auto& point = to_return.emplace_back();
      point.pos = Eigen::Map<const Eigen::Vector3d>(info.voxel_pos);
      if (info.label) {
        point.color = cmap.getColor(*info.label);
      }
    }
  } else {
    for (const auto idx : attrs.pcl_mesh_connections) {
      auto& point = to_return.emplace_back();
      point.pos = vertices->pos(idx).cast<double>();
      point.color = vertices->colors.at(idx);
    }
  }

  return to_return;
}

BasisPointPlugin::BasisPointPlugin(const Config& config,
                                   ianvs::NodeHandle nh,
                                   const std::string& name)
    : VisualizerPlugin(name),
      config_(name, config::checkValid(config)),
      pub_(nh.create_publisher<MarkerArray>(name, rclcpp::QoS(1).transient_local())) {}

void BasisPointPlugin::draw(const std_msgs::msg::Header& header,
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

void BasisPointPlugin::reset(const std_msgs::msg::Header& header) {
  MarkerArray msg;
  tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    pub_->publish(msg);
  }
}

void BasisPointPlugin::fillMarkers(const std_msgs::msg::Header& header,
                                   const DynamicSceneGraph& graph,
                                   MarkerArray& msg) const {
  if (!graph.hasLayer(DsgLayers::PLACES)) {
    return;
  }

  const auto config = config_.get();
  const visualizer::CategoricalColormap colormap(config.colormap);
  const auto& places = graph.getLayer(DsgLayers::PLACES);

  visualizer::LayerInfo info(config.graph);
  info.node_color = [&](const SceneGraphNode& node) {
    const auto parent = node.getParent();
    return parent ? graph.getNode(*parent).attributes<SemanticNodeAttributes>().color
                  : spark_dsg::Color();
  };

  if (!info.config.visualize) {
    return;
  }

  tracker_.add(makeLayerNodeMarkers(header, info, places, "nodes"), msg);
  if (config.draw_layer_edges) {
    tracker_.add(makeLayerEdgeMarkers(header, info, places, "edges"), msg);
  }

  drawEdges(config, colormap, header, graph, msg);
  if (config.draw_basis_points) {
    drawBasisPoints(config, colormap, header, graph, msg);
  }
}

void BasisPointPlugin::drawEdges(const Config& config,
                                 const visualizer::CategoricalColormap& colormap,
                                 const std_msgs::msg::Header& header,
                                 const DynamicSceneGraph& graph,
                                 MarkerArray& msg) const {
  Marker marker;
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = "places_parent_edges";
  marker.pose.orientation.w = 1.0;
  marker.scale.x = config.places_edge_scale;
  marker.color.a = config.places_edge_alpha;

  const auto& layer = graph.getLayer(DsgLayers::PLACES);
  for (const auto& id_node_pair : layer.nodes()) {
    auto& attrs = id_node_pair.second->attributes<PlaceNodeAttributes>();

    geometry_msgs::msg::Point start;
    tf2::convert(attrs.position, start);

    const auto basis_points = getBasisPoints(
        colormap, attrs, config.show_voxblox_connections, graph.mesh().get());
    for (const auto& basis_point : basis_points) {
      marker.points.push_back(start);
      auto& point = marker.points.emplace_back();
      tf2::convert(basis_point.pos, point);
    }
  }

  tracker_.add(marker, msg);
}

void BasisPointPlugin::drawBasisPoints(const Config& config,
                                       const visualizer::CategoricalColormap& colormap,
                                       const std_msgs::msg::Header& header,
                                       const DynamicSceneGraph& graph,
                                       MarkerArray& msg) const {
  const auto mesh = graph.mesh();
  if (!config.show_voxblox_connections && !mesh) {
    return;
  }

  Marker marker;
  marker.header = header;
  marker.type = Marker::CUBE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = "places_parents";
  marker.pose.orientation.w = 1.0;
  marker.scale.x = config.basis_point_scale;
  marker.scale.y = config.basis_point_scale;
  marker.scale.z = config.basis_point_scale;

  const auto& layer = graph.getLayer(DsgLayers::PLACES);
  for (const auto& id_node_pair : layer.nodes()) {
    auto& attrs = id_node_pair.second->attributes<PlaceNodeAttributes>();
    const auto points =
        getBasisPoints(colormap, attrs, config.show_voxblox_connections, mesh.get());
    for (const auto& basis_point : points) {
      auto& point = marker.points.emplace_back();
      tf2::convert(basis_point.pos, point);
      auto& color = marker.colors.emplace_back();
      color = visualizer::makeColorMsg(basis_point.color, config.basis_point_alpha);
    }
  }

  tracker_.add(marker, msg);
}

YAML::Node BasisPointPlugin::dumpConfig() const {
  auto root = config::toYaml(config_.get());
  root["type"] = "BasisPointPlugin";
  return root;
}

}  // namespace hydra
