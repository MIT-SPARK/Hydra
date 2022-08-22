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
#include "hydra_utils/colormap_utils.h"

#include <tf2_eigen/tf2_eigen.h>

namespace hydra {

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

inline void fillPoseWithIdentity(geometry_msgs::Pose& pose) {
  Eigen::Vector3d identity_pos = Eigen::Vector3d::Zero();
  tf2::convert(identity_pos, pose.position);
  tf2::convert(Eigen::Quaterniond::Identity(), pose.orientation);
}

}  // namespace

Marker makeDeleteMarker(const std_msgs::Header& header,
                        size_t id,
                        const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.action = Marker::DELETE;
  marker.id = id;
  marker.ns = ns;
  return marker;
}

Marker makeBoundingBoxMarker(const std_msgs::Header& header,
                             const LayerConfig& config,
                             const Node& node,
                             const VisualizerConfig& visualizer_config,
                             const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.type = Marker::CUBE;
  marker.action = Marker::ADD;
  marker.id = node.id;
  marker.ns = ns;
  marker.color = makeColorMsg(node.attributes<SemanticNodeAttributes>().color,
                              config.bounding_box_alpha);

  BoundingBox bounding_box = node.attributes<SemanticNodeAttributes>().bounding_box;

  Eigen::Quaternionf world_q_center =
      bounding_box.type == BoundingBox::Type::AABB
          ? Eigen::Quaternionf::Identity()
          : Eigen::Quaternionf(bounding_box.world_R_center);

  switch (bounding_box.type) {
    case BoundingBox::Type::OBB:
      marker.pose.position =
          tf2::toMsg(bounding_box.world_P_center.cast<double>().eval());
      tf2::convert(world_q_center.cast<double>(), marker.pose.orientation);
      marker.pose.position.z +=
          config.collapse_bounding_box ? 0.0 : getZOffset(config, visualizer_config);
      break;
    case BoundingBox::Type::AABB:
    case BoundingBox::Type::RAABB:
      marker.pose.position =
          tf2::toMsg(bounding_box.world_P_center.cast<double>().eval());
      tf2::convert(world_q_center.cast<double>(), marker.pose.orientation);
      marker.pose.position.z +=
          config.collapse_bounding_box ? 0.0 : getZOffset(config, visualizer_config);
      break;
    default:
      ROS_ERROR("Invalid bounding box encountered!");
      break;
  }

  tf2::toMsg((bounding_box.max - bounding_box.min).cast<double>().eval(), marker.scale);

  return marker;
}

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
  marker.text = NodeSymbol(node.id).getLabel();
  marker.scale.z = config.label_scale;
  marker.color = makeColorMsg(NodeColor::Zero());

  fillPoseWithIdentity(marker.pose);
  tf2::convert(node.attributes().position, marker.pose.position);
  marker.pose.position.z += getZOffset(config, visualizer_config) + config.label_height;

  return marker;
}

Marker makeCentroidMarkers(const std_msgs::Header& header,
                           const LayerConfig& config,
                           const SceneGraphLayer& layer,
                           const VisualizerConfig& visualizer_config,
                           const std::string& ns) {
  return makeCentroidMarkers(header,
                             config,
                             layer,
                             visualizer_config,
                             ns,
                             [&](const SceneGraphNode& node) -> NodeColor {
                               try {
                                 return node.attributes<SemanticNodeAttributes>().color;
                               } catch (const std::bad_cast&) {
                                 return NodeColor::Zero();
                               }
                             });
}

Marker makeCentroidMarkers(const std_msgs::Header& header,
                           const LayerConfig& config,
                           const SceneGraphLayer& layer,
                           const VisualizerConfig& visualizer_config,
                           const std::string& ns,
                           const ColormapConfig& colors) {
  return makeCentroidMarkers(
      header, config, layer, visualizer_config, ns, [&](const SceneGraphNode& node) {
        return getDistanceColor(
            visualizer_config, colors, node.attributes<PlaceNodeAttributes>().distance);
      });
}

Marker makeCentroidMarkers(const std_msgs::Header& header,
                           const LayerConfig& config,
                           const SceneGraphLayer& layer,
                           const VisualizerConfig& visualizer_config,
                           const std::string& ns,
                           const ColorFunction& color_func) {
  Marker marker;
  marker.header = header;
  marker.type = config.use_sphere_marker ? Marker::SPHERE_LIST : Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0;
  marker.ns = ns;

  marker.scale.x = config.marker_scale;
  marker.scale.y = config.marker_scale;
  marker.scale.z = config.marker_scale;

  fillPoseWithIdentity(marker.pose);

  marker.points.reserve(layer.numNodes());
  marker.colors.reserve(layer.numNodes());
  for (const auto& id_node_pair : layer.nodes()) {
    geometry_msgs::Point node_centroid;
    tf2::convert(id_node_pair.second->attributes().position, node_centroid);
    node_centroid.z += getZOffset(config, visualizer_config);
    marker.points.push_back(node_centroid);

    NodeColor desired_color = color_func(*id_node_pair.second);
    marker.colors.push_back(makeColorMsg(desired_color, config.marker_alpha));
  }

  return marker;
}

namespace {

inline Marker makeNewEdgeList(const std_msgs::Header& header,
                              const LayerConfig& config,
                              const std::string& ns_prefix,
                              LayerId source,
                              LayerId target) {
  Marker marker;
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = ns_prefix + std::to_string(source) + "_" + std::to_string(target);
  marker.scale.x = config.interlayer_edge_scale;
  fillPoseWithIdentity(marker.pose);
  return marker;
}

}  // namespace

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

Marker makeMeshEdgesMarker(const std_msgs::Header& header,
                           const LayerConfig& config,
                           const VisualizerConfig& visualizer_config,
                           const DynamicSceneGraph& graph,
                           const SceneGraphLayer& layer,
                           const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = ns;

  marker.scale.x = config.interlayer_edge_scale;
  fillPoseWithIdentity(marker.pose);

  for (const auto& id_node_pair : layer.nodes()) {
    const Node& node = *id_node_pair.second;
    auto mesh_edge_indices = graph.getMeshConnectionIndices(node.id);
    if (mesh_edge_indices.empty()) {
      continue;
    }

    SemanticNodeAttributes attrs = node.attributes<SemanticNodeAttributes>();

    geometry_msgs::Point center_point;
    tf2::convert(attrs.position, center_point);
    center_point.z +=
        visualizer_config.mesh_edge_break_ratio * getZOffset(config, visualizer_config);

    geometry_msgs::Point centroid_location;
    tf2::convert(attrs.position, centroid_location);
    centroid_location.z += getZOffset(config, visualizer_config);

    // make first edge
    marker.points.push_back(centroid_location);
    marker.points.push_back(center_point);
    if (config.interlayer_edge_use_color) {
      marker.colors.push_back(makeColorMsg(attrs.color, config.interlayer_edge_alpha));
      marker.colors.push_back(makeColorMsg(attrs.color, config.interlayer_edge_alpha));
    } else {
      marker.colors.push_back(
          makeColorMsg(NodeColor::Zero(), config.interlayer_edge_alpha));
      marker.colors.push_back(
          makeColorMsg(NodeColor::Zero(), config.interlayer_edge_alpha));
    }

    for (size_t i = 0; i < mesh_edge_indices.size();
         i += config.interlayer_edge_insertion_skip + 1) {
      std::optional<Eigen::Vector3d> vertex_pos =
          graph.getMeshPosition(mesh_edge_indices[i]);
      if (!vertex_pos) {
        continue;
      }

      geometry_msgs::Point vertex;
      tf2::convert(*vertex_pos, vertex);
      if (!visualizer_config.collapse_layers) {
        vertex.z += visualizer_config.mesh_layer_offset;
      }

      marker.points.push_back(center_point);
      marker.points.push_back(vertex);

      if (config.interlayer_edge_use_color) {
        marker.colors.push_back(
            makeColorMsg(attrs.color, config.interlayer_edge_alpha));
        marker.colors.push_back(
            makeColorMsg(attrs.color, config.interlayer_edge_alpha));
      } else {
        marker.colors.push_back(
            makeColorMsg(NodeColor::Zero(), config.interlayer_edge_alpha));
        marker.colors.push_back(
            makeColorMsg(NodeColor::Zero(), config.interlayer_edge_alpha));
      }
    }
  }

  return marker;
}

Marker makeLayerEdgeMarkers(const std_msgs::Header& header,
                            const LayerConfig& config,
                            const SceneGraphLayer& layer,
                            const VisualizerConfig& visualizer_config,
                            const NodeColor& color,
                            const std::string& ns) {
  return makeLayerEdgeMarkers(
      header,
      config,
      layer,
      visualizer_config,
      ns,
      [&](const SceneGraphNode&, const SceneGraphNode&, bool) { return color; });
}

Marker makeLayerEdgeMarkers(const std_msgs::Header& header,
                            const LayerConfig& config,
                            const SceneGraphLayer& layer,
                            const VisualizerConfig& visualizer_config,
                            const std::string& ns,
                            const EdgeColorFunction& color_func) {
  Marker marker;
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.id = 0;
  marker.ns = ns;

  marker.action = Marker::ADD;
  marker.scale.x = config.intralayer_edge_scale;
  fillPoseWithIdentity(marker.pose);

  auto edge_iter = layer.edges().begin();
  while (edge_iter != layer.edges().end()) {
    const SceneGraphNode& source_node = layer.getNode(edge_iter->second.source).value();
    const SceneGraphNode& target_node = layer.getNode(edge_iter->second.target).value();

    geometry_msgs::Point source;
    tf2::convert(source_node.attributes().position, source);
    source.z += getZOffset(config, visualizer_config);
    marker.points.push_back(source);

    geometry_msgs::Point target;
    tf2::convert(target_node.attributes().position, target);
    target.z += getZOffset(config, visualizer_config);
    marker.points.push_back(target);

    marker.colors.push_back(makeColorMsg(color_func(source_node, target_node, true),
                                         config.intralayer_edge_alpha));
    marker.colors.push_back(makeColorMsg(color_func(source_node, target_node, false),
                                         config.intralayer_edge_alpha));

    std::advance(edge_iter, config.intralayer_edge_insertion_skip + 1);
  }

  return marker;
}

Marker makeDynamicCentroidMarkers(const std_msgs::Header& header,
                                  const DynamicLayerConfig& config,
                                  const DynamicSceneGraphLayer& layer,
                                  const VisualizerConfig& visualizer_config,
                                  const NodeColor& color,
                                  const std::string& ns) {
  return makeDynamicCentroidMarkers(header,
                                    config,
                                    layer,
                                    config.z_offset_scale,
                                    visualizer_config,
                                    ns,
                                    [&](const auto&) -> NodeColor { return color; });
}

Marker makeDynamicCentroidMarkers(const std_msgs::Header& header,
                                  const DynamicLayerConfig& config,
                                  const DynamicSceneGraphLayer& layer,
                                  double layer_offset_scale,
                                  const VisualizerConfig& visualizer_config,
                                  const std::string& ns,
                                  const ColorFunction& color_func) {
  Marker marker;
  marker.header = header;
  marker.type = config.node_use_sphere ? Marker::SPHERE_LIST : Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.ns = ns;
  marker.id = 0;

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
                              const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.ns = ns;
  marker.id = 0;

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
                              const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.type = Marker::TEXT_VIEW_FACING;
  marker.ns = ns;
  marker.id = 0;
  marker.action = Marker::ADD;
  marker.lifetime = ros::Duration(0);
  marker.text = std::to_string(layer.id) + ":" + layer.prefix.str();
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
