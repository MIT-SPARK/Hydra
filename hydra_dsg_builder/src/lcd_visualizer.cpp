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
#include "hydra_dsg_builder/lcd_visualizer.h"

#include <hydra_utils/colormap_utils.h>
#include <hydra_utils/visualizer_utils.h>

#include <tf2_eigen/tf2_eigen.h>

namespace hydra {
namespace lcd {

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
using Node = SceneGraphLayer::Node;

void makeCircle(Marker& marker,
                const Eigen::Vector3d& position,
                double z_offset,
                double radius,
                size_t num_points = 100) {
  geometry_msgs::Point prev;
  prev.x = radius + position.x();
  prev.y = position.y();
  prev.z = position.z() + z_offset;

  for (size_t i = 1; i <= num_points; ++i) {
    double t = i / static_cast<double>(num_points);

    geometry_msgs::Point curr;
    curr.x = radius * std::cos(2 * M_PI * t) + position.x();
    curr.y = radius * std::sin(2 * M_PI * t) + position.y();
    curr.z = position.z() + z_offset;
    marker.points.push_back(prev);
    marker.points.push_back(curr);

    prev = curr;
  }
}

void readColor(const ros::NodeHandle& nh,
               const std::string& name,
               std::vector<double> default_color,
               NodeColor& color) {
  nh.getParam(name, default_color);
  CHECK_EQ(default_color.size(), 3u) << name << " size != 3";
  color << static_cast<uint8_t>(default_color[0] * 255.0),
      static_cast<uint8_t>(default_color[1] * 255.0),
      static_cast<uint8_t>(default_color[2] * 255.0);
}

LcdVisualizer::LcdVisualizer(const ros::NodeHandle& nh, double radius)
    : DynamicSceneGraphVisualizer(nh, LcdVisualizer::getDefaultLayerIds()),
      radius_(radius) {
  // red-ish
  std::vector<double> invalid_color{0.78, 0.192, 0.173};
  readColor(nh, "invalid_color", invalid_color, invalid_match_color_);

  // green
  std::vector<double> valid_color{0.231, 0.820, 0.278};
  readColor(nh, "valid_color", valid_color, valid_match_color_);

  std::vector<double> default_color{0.0, 0.0, 0.0};
  readColor(nh, "default_color", default_color, default_graph_color_);

  std::vector<double> query_color{0.259, 0.529, 0.961};
  readColor(nh, "query_color", query_color, query_color_);

  const std::string ns = visualizer_ns_ + "/agent_layer";
  agent_config_ = std::make_shared<DynamicLayerConfigManager>(ros::NodeHandle(""), ns);
}

void LcdVisualizer::setLcdDetector(DsgLcdDetector* detector) {
  lcd_detector_ = detector;
}

std::vector<NodeId> LcdVisualizer::getMatchRoots(LayerId layer) const {
  const auto& layer_match_map = lcd_detector_->getLayerRemapping();
  if (!layer_match_map.count(layer)) {
    return {};
  }

  const auto& matches = lcd_detector_->getLatestMatches();
  const size_t match_idx = layer_match_map.at(layer);
  if (!matches.count(match_idx)) {
    return {};
  }

  return matches.at(match_idx).match_root;
}

std::set<NodeId> LcdVisualizer::getMatchedNodes(LayerId layer) const {
  const auto& layer_match_map = lcd_detector_->getLayerRemapping();
  if (!layer_match_map.count(layer)) {
    return {};
  }

  const auto& matches = lcd_detector_->getLatestMatches();
  const size_t match_idx = layer_match_map.at(layer);
  if (!matches.count(match_idx)) {
    return {};
  }

  const auto& result = matches.at(match_idx);

  std::set<NodeId> to_return;
  for (const auto& match : result.match_nodes) {
    to_return.insert(match.begin(), match.end());
  }

  return to_return;
}

std::optional<NodeId> LcdVisualizer::getQueryNode() const {
  const auto& matches = lcd_detector_->getLatestMatches();
  if (!matches.count(0)) {
    return std::nullopt;
  }

  return *matches.at(0).query_nodes.begin();
}

std::set<NodeId> LcdVisualizer::getValidNodes(LayerId layer) const {
  const auto& layer_match_map = lcd_detector_->getLayerRemapping();
  if (!layer_match_map.count(layer)) {
    return {};
  }

  const auto& matches = lcd_detector_->getLatestMatches();
  const size_t match_idx = layer_match_map.at(layer);
  if (!matches.count(match_idx)) {
    return {};
  }

  const auto& descriptor_cache = lcd_detector_->getDescriptorCache(layer);
  const auto& result = matches.at(match_idx);

  std::set<NodeId> to_return;
  for (const auto& valid_root : result.valid_matches) {
    const auto& nodes = descriptor_cache.at(valid_root)->nodes;
    to_return.insert(nodes.begin(), nodes.end());
  }

  return to_return;
}

Marker makeCircleMarker(const std_msgs::Header& header,
                        const NodeColor& color,
                        double alpha,
                        const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = ns;
  marker.color = dsg_utils::makeColorMsg(color, alpha);

  Eigen::Vector3d identity_pos = Eigen::Vector3d::Zero();
  tf2::convert(identity_pos, marker.pose.position);
  tf2::convert(Eigen::Quaterniond::Identity(), marker.pose.orientation);

  return marker;
}

void LcdVisualizer::drawAgent(const std_msgs::Header& header, MarkerArray& msg) {
  const DynamicLayerConfig& config = agent_config_->get();
  const std::string node_ns = "lcd_agent_nodes";
  const std::string circle_ns = "query_radius";

  if (!config.visualize) {
    deleteMultiMarker(header, node_ns, msg);
    return;
  }

  const auto& agent_layer = scene_graph_->getLayer(DsgLayers::AGENTS, 'a');

  auto query_node = getQueryNode();
  if (!query_node) {
    deleteMultiMarker(header, circle_ns, msg);
  } else {
    Marker query_radius = makeCircleMarker(header, query_color_, 1.0, circle_ns);
    query_radius.scale.x = config.edge_scale;

    const double z_offset =
        getZOffset(config.z_offset_scale, visualizer_config_->get());
    makeCircle(query_radius, agent_layer.getPosition(*query_node), z_offset, radius_);
    addMultiMarkerIfValid(query_radius, msg);
  }

  Marker nodes =
      makeDynamicCentroidMarkers(header,
                                 config,
                                 agent_layer,
                                 config.z_offset_scale,
                                 visualizer_config_->get(),
                                 node_ns,
                                 [&](const auto& node) -> NodeColor {
                                   if (query_node && *query_node == node.id) {
                                     return query_color_;
                                   } else {
                                     return default_graph_color_;
                                   }
                                 });
  addMultiMarkerIfValid(nodes, msg);
}

void LcdVisualizer::redrawImpl(const std_msgs::Header& header, MarkerArray& msg) {
  if (!lcd_detector_) {
    return;
  }

  DynamicSceneGraphVisualizer::redrawImpl(header, msg);
  drawAgent(header, msg);
}

void LcdVisualizer::drawLayer(const std_msgs::Header& header,
                              const SceneGraphLayer& layer,
                              const LayerConfig& config,
                              MarkerArray& msg) {
  const auto& viz_config = visualizer_config_->get();
  const std::string node_ns = getLayerNodeNamespace(layer.id);

  std::set<NodeId> matched_ids = getMatchedNodes(layer.id);
  std::set<NodeId> valid_ids = getValidNodes(layer.id);

  Marker nodes = makeCentroidMarkers(header,
                                     config,
                                     layer,
                                     viz_config,
                                     node_ns,
                                     [&](const SceneGraphNode& node) -> NodeColor {
                                       if (matched_ids.count(node.id)) {
                                         return valid_match_color_;
                                       } else if (valid_ids.count(node.id)) {
                                         return invalid_match_color_;
                                       } else {
                                         return default_graph_color_;
                                       }
                                     });
  addMultiMarkerIfValid(nodes, msg);

  auto match_roots = getMatchRoots(layer.id);
  const std::string circle_ns = "match_radius" + std::to_string(layer.id);
  if (match_roots.empty()) {
    deleteMultiMarker(header, circle_ns, msg);
  } else {
    Marker match_radius = makeCircleMarker(header, valid_match_color_, 1.0, circle_ns);
    match_radius.scale.x = agent_config_->get().edge_scale;

    const double z_offset = getZOffset(config, visualizer_config_->get());
    for (const auto& node_id : match_roots) {
      makeCircle(match_radius, scene_graph_->getPosition(node_id), z_offset, radius_);
    }
    addMultiMarkerIfValid(match_radius, msg);
  }

  const std::string edge_ns = getLayerEdgeNamespace(layer.id);
  Marker edges = makeLayerEdgeMarkers(
      header,
      config,
      layer,
      viz_config,
      edge_ns,
      [&](const SceneGraphNode& src, const SceneGraphNode& tgt, bool) -> NodeColor {
        if (matched_ids.count(src.id) && matched_ids.count(tgt.id)) {
          return valid_match_color_;
        } else if (valid_ids.count(src.id) && valid_ids.count(tgt.id)) {
          return invalid_match_color_;
        } else {
          return default_graph_color_;
        }
      });
  addMultiMarkerIfValid(edges, msg);
}

void LcdVisualizer::drawLayerMeshEdges(const std_msgs::Header&,
                                       LayerId,
                                       const std::string&,
                                       MarkerArray&) {}

}  // namespace lcd
}  // namespace hydra
