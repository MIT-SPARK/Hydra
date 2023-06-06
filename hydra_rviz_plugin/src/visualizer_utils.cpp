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
#include "hydra_utils/visualizer_utils.h"

#include <tf2_eigen/tf2_eigen.h>

#include <algorithm>
#include <opencv2/imgproc.hpp>

namespace hydra {

inline double lerp(double min, double max, double ratio) {
  return (max - min) * ratio + min;
}

NodeColor getRgbFromHls(double hue, double luminance, double saturation) {
  // make sure we clip the inputs to the expected range
  hue = std::clamp(hue, 0.0, 1.0);
  luminance = std::clamp(luminance, 0.0, 1.0);
  saturation = std::clamp(saturation, 0.0, 1.0);

  cv::Mat hls_value(1, 1, CV_32FC3);
  // hue is in degrees, not [0, 1]
  hls_value.at<float>(0) = hue * 360.0;
  hls_value.at<float>(1) = luminance;
  hls_value.at<float>(2) = saturation;

  cv::Mat bgr;
  cv::cvtColor(hls_value, bgr, cv::COLOR_HLS2BGR);

  NodeColor color;
  color(0, 0) = static_cast<uint8_t>(255 * bgr.at<float>(2));
  color(1, 0) = static_cast<uint8_t>(255 * bgr.at<float>(1));
  color(2, 0) = static_cast<uint8_t>(255 * bgr.at<float>(0));
  return color;
}

NodeColor interpolateColorMap(const ColormapConfig& config, double ratio) {
  // override ratio input to be in [0, 1]
  ratio = std::clamp(ratio, 0.0, 1.0);

  cv::Mat hls_value(1, 1, CV_32FC3);
  // hue is in degrees, not [0, 1]
  hls_value.at<float>(0) = lerp(config.min_hue, config.max_hue, ratio) * 360.0;
  hls_value.at<float>(1) = lerp(config.min_luminance, config.max_luminance, ratio);
  hls_value.at<float>(2) = lerp(config.min_saturation, config.max_saturation, ratio);

  cv::Mat bgr;
  cv::cvtColor(hls_value, bgr, cv::COLOR_HLS2BGR);

  NodeColor color;
  color(0, 0) = static_cast<uint8_t>(255 * bgr.at<float>(2));
  color(1, 0) = static_cast<uint8_t>(255 * bgr.at<float>(1));
  color(2, 0) = static_cast<uint8_t>(255 * bgr.at<float>(0));
  return color;
}

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
using Node = SceneGraphLayer::Node;
using dsg_utils::makeColorMsg;

namespace {

inline double getRatio(double min, double max, double value) {
  double ratio = (value - min) / (max - min);
  ratio = !std::isfinite(ratio) ? 0.0 : ratio;
  ratio = ratio > 1.0 ? 1.0 : ratio;
  ratio = ratio < 0.0 ? 0.0 : ratio;
  return ratio;
}

inline NodeColor getDistanceColor(const VisualizerConfig& config,
                                  const ColormapConfig& colors,
                                  double distance) {
  if (config.places_colormap_max_distance <= config.places_colormap_min_distance) {
    // TODO(nathan) consider warning
    return NodeColor::Zero();
  }

  double ratio = getRatio(config.places_colormap_min_distance,
                          config.places_colormap_max_distance,
                          distance);

  return dsg_utils::interpolateColorMap(colors, ratio);
}

}  // namespace

Marker makeTextMarker(const std_msgs::Header& header,
                      const LayerConfig& config,
                      const Node& node,
                      const VisualizerConfig& visualizer_config,
                      const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.ns = ns;
  marker.id = node.id;
  marker.type = Marker::TEXT_VIEW_FACING;
  marker.action = Marker::ADD;
  marker.lifetime = ros::Duration(0);
  marker.text = node.attributes<SemanticNodeAttributes>().name;
  marker.scale.z = config.label_scale;
  marker.color = makeColorMsg(NodeColor::Zero());

  fillPoseWithIdentity(marker.pose);
  tf2::convert(node.attributes().position, marker.pose.position);
  marker.pose.position.z += getZOffset(config, visualizer_config) + config.label_height;

  return marker;
}

Marker makeTextMarkerNoHeight(const std_msgs::Header& header,
                              const LayerConfig& config,
                              const Node& node,
                              const VisualizerConfig& visualizer_config,
                              const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.ns = ns;
  marker.id = node.id;
  marker.type = Marker::TEXT_VIEW_FACING;
  marker.action = Marker::ADD;
  marker.lifetime = ros::Duration(0);
  marker.text = node.attributes<SemanticNodeAttributes>().name;
  marker.scale.z = config.label_scale;
  marker.color = makeColorMsg(NodeColor::Zero());

  fillPoseWithIdentity(marker.pose);
  tf2::convert(node.attributes().position, marker.pose.position);
  marker.pose.position.z += config.label_height;

  return marker;
}

bool shouldVisualize(const DynamicSceneGraph& graph,
                     const SceneGraphNode& node,
                     const std::map<LayerId, LayerConfig>& configs,
                     const std::map<LayerId, DynamicLayerConfig>& dynamic_configs) {
  if (graph.isDynamic(node.id)) {
    return dynamic_configs.count(node.layer) &&
           dynamic_configs.at(node.layer).visualize &&
           dynamic_configs.at(node.layer).visualize_interlayer_edges;
  }

  return configs.count(node.layer) && configs.at(node.layer).visualize;
}

LayerId getConfigLayer(const DynamicSceneGraph& graph,
                       const SceneGraphNode& source,
                       const SceneGraphNode& target) {
  if (graph.isDynamic(source.id)) {
    return source.layer;
  } else {
    return target.layer;
  }
}

MarkerArray makeDynamicGraphEdgeMarkers(
    const std_msgs::Header& header,
    const DynamicSceneGraph& graph,
    const std::map<LayerId, LayerConfig>& configs,
    const std::map<LayerId, DynamicLayerConfig>& dynamic_configs,
    const VisualizerConfig& visualizer_config,
    const std::string& ns_prefix) {
  MarkerArray layer_edges;
  std::map<LayerId, Marker> layer_markers;
  std::map<LayerId, size_t> num_since_last_insertion;

  for (const auto& edge : graph.dynamic_interlayer_edges()) {
    const Node& source = graph.getNode(edge.second.source).value();
    const Node& target = graph.getNode(edge.second.target).value();

    if (!shouldVisualize(graph, source, configs, dynamic_configs)) {
      continue;
    }

    if (!shouldVisualize(graph, target, configs, dynamic_configs)) {
      continue;
    }

    DynamicLayerConfig config =
        dynamic_configs.at(getConfigLayer(graph, source, target));

    size_t num_between_insertions = config.interlayer_edge_insertion_skip;

    if (layer_markers.count(source.layer) == 0) {
      layer_markers[source.layer] = makeNewEdgeList(
          header, configs.at(source.layer), ns_prefix, source.layer, target.layer);
      layer_markers[source.layer].color =
          makeColorMsg(NodeColor::Zero(), config.edge_alpha);
      // make sure we always draw at least one edge
      num_since_last_insertion[source.layer] = num_between_insertions;
    }

    if (num_since_last_insertion[source.layer] >= num_between_insertions) {
      num_since_last_insertion[source.layer] = 0;
    } else {
      num_since_last_insertion[source.layer]++;
      continue;
    }

    Marker& marker = layer_markers.at(source.layer);
    geometry_msgs::Point source_point;
    tf2::convert(source.attributes().position, source_point);
    source_point.z += getZOffset(configs.at(source.layer), visualizer_config);
    marker.points.push_back(source_point);

    geometry_msgs::Point target_point;
    tf2::convert(target.attributes().position, target_point);
    target_point.z += getZOffset(configs.at(target.layer), visualizer_config);
    marker.points.push_back(target_point);
  }

  for (const auto& id_marker_pair : layer_markers) {
    layer_edges.markers.push_back(id_marker_pair.second);
  }
  return layer_edges;
}

// TODO(nathan) consider making this shorter
MarkerArray makeGraphEdgeMarkers(const std_msgs::Header& header,
                                 const DynamicSceneGraph& graph,
                                 const std::map<LayerId, LayerConfig>& configs,
                                 const VisualizerConfig& visualizer_config,
                                 const std::string& ns_prefix) {
  MarkerArray layer_edges;
  std::map<LayerId, Marker> layer_markers;
  std::map<LayerId, size_t> num_since_last_insertion;

  for (const auto& edge : graph.interlayer_edges()) {
    const Node& source = *(graph.getNode(edge.second.source));
    const Node& target = *(graph.getNode(edge.second.target));

    if (!configs.count(source.layer) || !configs.count(target.layer)) {
      continue;
    }

    if (!configs.at(source.layer).visualize) {
      continue;
    }

    if (!configs.at(target.layer).visualize) {
      continue;
    }

    size_t num_between_insertions =
        configs.at(source.layer).interlayer_edge_insertion_skip;

    // parent is always source
    // TODO(nathan) make the above statement an invariant
    if (layer_markers.count(source.layer) == 0) {
      layer_markers[source.layer] = makeNewEdgeList(
          header, configs.at(source.layer), ns_prefix, source.layer, target.layer);
      // make sure we always draw at least one edge
      num_since_last_insertion[source.layer] = num_between_insertions;
    }

    if (num_since_last_insertion[source.layer] >= num_between_insertions) {
      num_since_last_insertion[source.layer] = 0;
    } else {
      num_since_last_insertion[source.layer]++;
      continue;
    }

    Marker& marker = layer_markers.at(source.layer);
    geometry_msgs::Point source_point;
    tf2::convert(source.attributes().position, source_point);
    source_point.z += getZOffset(configs.at(source.layer), visualizer_config);
    marker.points.push_back(source_point);

    geometry_msgs::Point target_point;
    tf2::convert(target.attributes().position, target_point);
    target_point.z += getZOffset(configs.at(target.layer), visualizer_config);
    marker.points.push_back(target_point);

    NodeColor edge_color;
    if (configs.at(source.layer).interlayer_edge_use_color) {
      if (configs.at(source.layer).use_edge_source) {
        // TODO(nathan) this might not be a safe cast in general
        edge_color = source.attributes<SemanticNodeAttributes>().color;
      } else {
        // TODO(nathan) this might not be a safe cast in general
        edge_color = target.attributes<SemanticNodeAttributes>().color;
      }
    } else {
      edge_color = NodeColor::Zero();
    }

    marker.colors.push_back(
        makeColorMsg(edge_color, configs.at(source.layer).intralayer_edge_alpha));
    marker.colors.push_back(
        makeColorMsg(edge_color, configs.at(source.layer).intralayer_edge_alpha));
  }

  for (const auto& id_marker_pair : layer_markers) {
    layer_edges.markers.push_back(id_marker_pair.second);
  }
  return layer_edges;
}

Marker makeDynamicCentroidMarkers(const std_msgs::Header& header,
                                  const DynamicLayerConfig& config,
                                  const DynamicSceneGraphLayer& layer,
                                  const VisualizerConfig& visualizer_config,
                                  const NodeColor& color,
                                  const std::string& ns,
                                  size_t marker_id) {
  return makeDynamicCentroidMarkers(
      header,
      config,
      layer,
      config.z_offset_scale,
      visualizer_config,
      ns,
      [&](const auto&) -> NodeColor { return color; },
      marker_id);
}

Marker makeDynamicCentroidMarkers(const std_msgs::Header& header,
                                  const DynamicLayerConfig& config,
                                  const DynamicSceneGraphLayer& layer,
                                  double layer_offset_scale,
                                  const VisualizerConfig& visualizer_config,
                                  const std::string& ns,
                                  const ColorFunction& color_func,
                                  size_t marker_id) {
  Marker marker;
  marker.header = header;
  marker.type = config.node_use_sphere ? Marker::SPHERE_LIST : Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.ns = ns;
  marker.id = marker_id;

  marker.scale.x = config.node_scale;
  marker.scale.y = config.node_scale;
  marker.scale.z = config.node_scale;

  fillPoseWithIdentity(marker.pose);

  marker.points.reserve(layer.numNodes());
  for (const auto& node : layer.nodes()) {
    geometry_msgs::Point node_centroid;
    tf2::convert(node->attributes().position, node_centroid);
    node_centroid.z += getZOffset(layer_offset_scale, visualizer_config);
    marker.points.push_back(node_centroid);
    marker.colors.push_back(makeColorMsg(color_func(*node), config.node_alpha));
  }

  return marker;
}

Marker makeDynamicEdgeMarkers(const std_msgs::Header& header,
                              const DynamicLayerConfig& config,
                              const DynamicSceneGraphLayer& layer,
                              const VisualizerConfig& visualizer_config,
                              const NodeColor& color,
                              const std::string& ns,
                              size_t marker_id) {
  Marker marker;
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.ns = ns;
  marker.id = marker_id;

  marker.action = Marker::ADD;
  marker.scale.x = config.edge_scale;
  marker.color = makeColorMsg(color, config.edge_alpha);
  fillPoseWithIdentity(marker.pose);

  for (const auto& id_edge_pair : layer.edges()) {
    geometry_msgs::Point source;
    tf2::convert(layer.getPosition(id_edge_pair.second.source), source);
    source.z += getZOffset(config.z_offset_scale, visualizer_config);
    marker.points.push_back(source);

    geometry_msgs::Point target;
    tf2::convert(layer.getPosition(id_edge_pair.second.target), target);
    target.z += getZOffset(config.z_offset_scale, visualizer_config);
    marker.points.push_back(target);
  }

  return marker;
}

Marker makeDynamicLabelMarker(const std_msgs::Header& header,
                              const DynamicLayerConfig& config,
                              const DynamicSceneGraphLayer& layer,
                              const VisualizerConfig& visualizer_config,
                              const std::string& ns,
                              size_t marker_id) {
  Marker marker;
  marker.header = header;
  marker.type = Marker::TEXT_VIEW_FACING;
  marker.ns = ns;
  marker.id = marker_id;
  marker.action = Marker::ADD;
  marker.lifetime = ros::Duration(0);
  marker.text = "Agent";  // std::to_string(layer.id) + ":" + layer.prefix.str();
  marker.scale.z = config.label_scale;
  marker.color = makeColorMsg(NodeColor::Zero());

  Eigen::Vector3d latest_position = layer.getPositionByIndex(layer.numNodes() - 1);
  fillPoseWithIdentity(marker.pose);
  tf2::convert(latest_position, marker.pose.position);
  marker.pose.position.z +=
      getZOffset(config.z_offset_scale, visualizer_config) + config.label_height;

  return marker;
}

}  // namespace hydra
