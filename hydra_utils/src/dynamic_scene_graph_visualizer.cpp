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
#include "hydra_utils/dynamic_scene_graph_visualizer.h"
#include "hydra_utils/colormap_utils.h"
#include "hydra_utils/visualizer_utils.h"

#include <tf2_eigen/tf2_eigen.h>

namespace hydra {

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
using Node = SceneGraphNode;

void clearPrevMarkers(const std_msgs::Header& header,
                      const std::set<NodeId>& curr_nodes,
                      const std::string& ns,
                      std::set<NodeId>& prev_nodes,
                      MarkerArray& msg) {
  for (const auto& node : prev_nodes) {
    if (curr_nodes.count(node)) {
      continue;
    }

    Marker marker = makeDeleteMarker(header, node, ns);
    msg.markers.push_back(marker);
  }

  prev_nodes = curr_nodes;
}

DynamicSceneGraphVisualizer::DynamicSceneGraphVisualizer(
    const ros::NodeHandle& nh,
    const DynamicSceneGraph::LayerIds& layer_ids)
    : nh_(nh),
      need_redraw_(false),
      periodic_redraw_(false),
      world_frame_("world"),
      visualizer_ns_(nh.resolveName("config")),
      visualizer_layer_ns_(nh.resolveName("config/layer")) {
  nh_.param("world_frame", world_frame_, world_frame_);
  nh_.param("visualizer_ns", visualizer_ns_, visualizer_ns_);
  nh_.param("visualizer_layer_ns", visualizer_layer_ns_, visualizer_layer_ns_);

  dsg_pub_ = nh_.advertise<MarkerArray>("dsg_markers", 1, true);

  setupConfigs(layer_ids);

  for (const auto& id : layer_ids) {
    prev_labels_[id] = {};
    curr_labels_[id] = {};
    prev_bboxes_[id] = {};
    curr_bboxes_[id] = {};
  }

  dynamic_layers_viz_pub_ = nh_.advertise<MarkerArray>("dynamic_layers_viz", 1, true);
}

void DynamicSceneGraphVisualizer::start(bool periodic_redraw) {
  periodic_redraw_ = periodic_redraw;
  double visualizer_loop_period = 1.0e-1;
  nh_.param("visualizer_loop_period", visualizer_loop_period, visualizer_loop_period);
  visualizer_loop_timer_ =
      nh_.createWallTimer(ros::WallDuration(visualizer_loop_period),
                          &DynamicSceneGraphVisualizer::displayLoop,
                          this);
}

void DynamicSceneGraphVisualizer::reset() {
  if (scene_graph_) {
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = world_frame_;

    MarkerArray msg;
    resetImpl(header, msg);

    if (!msg.markers.empty()) {
      dsg_pub_.publish(msg);
    }
  }

  scene_graph_.reset();
}

bool DynamicSceneGraphVisualizer::redraw() {
  if (!scene_graph_) {
    return false;
  }

  need_redraw_ |= hasConfigChanged();

  if (!need_redraw_) {
    return false;
  }
  need_redraw_ = false;

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = world_frame_;

  MarkerArray msg;
  redrawImpl(header, msg);

  if (!msg.markers.empty()) {
    dsg_pub_.publish(msg);
  }

  clearConfigChangeFlags();
  return true;
}

void DynamicSceneGraphVisualizer::setGraph(const DynamicSceneGraph::Ptr& scene_graph,
                                           bool need_reset) {
  if (scene_graph == nullptr) {
    ROS_ERROR("Request to visualize invalid scene graph! Ignoring");
    return;
  }

  if (need_reset) {
    reset();
  }

  scene_graph_ = scene_graph;
  need_redraw_ = true;
}

const DynamicLayerConfig& DynamicSceneGraphVisualizer::getConfig(LayerId layer) {
  if (!dynamic_configs_.count(layer)) {
    const std::string ns = visualizer_ns_ + "/dynamic_layer/" + std::to_string(layer);

    ros::NodeHandle nh("");
    dynamic_configs_[layer] = std::make_shared<DynamicLayerConfigManager>(nh, ns);
  }

  return dynamic_configs_.at(layer)->get();
}

inline double getDynamicHue(const DynamicLayerConfig& config, char prefix) {
  // distance is measured from first relatively readable character prefix
  int color_num = (std::abs((prefix - '0')) + config.color_offset) % config.num_colors;
  return static_cast<double>(color_num) / static_cast<double>(config.num_colors);
}

NodeColor getNodeColor(const DynamicLayerConfig& config, char prefix) {
  const double hue = getDynamicHue(config, prefix);
  return dsg_utils::getRgbFromHls(hue, config.luminance, config.saturation);
}

NodeColor getEdgeColor(const DynamicLayerConfig& config, char prefix) {
  const double hue = getDynamicHue(config, prefix);
  const double saturation = config.saturation * config.edge_sl_ratio;
  const double luminance = config.luminance * config.edge_sl_ratio;
  return dsg_utils::getRgbFromHls(hue, saturation, luminance);
}

void DynamicSceneGraphVisualizer::drawDynamicLayer(const std_msgs::Header& header,
                                                   const DynamicSceneGraphLayer& layer,
                                                   const DynamicLayerConfig& config,
                                                   const VisualizerConfig& viz_config,
                                                   MarkerArray& msg) {
  const std::string node_ns = getDynamicNodeNamespace(layer.prefix);
  Marker nodes = makeDynamicCentroidMarkers(
      header, config, layer, viz_config, getNodeColor(config, layer.prefix), node_ns);
  addMultiMarkerIfValid(nodes, msg);

  const std::string edge_ns = getDynamicEdgeNamespace(layer.prefix);
  Marker edges = makeDynamicEdgeMarkers(
      header, config, layer, viz_config, getEdgeColor(config, layer.prefix), edge_ns);
  addMultiMarkerIfValid(edges, msg);

  if (layer.numNodes() == 0) {
    deleteLabel(header, layer.prefix, msg);
    return;
  }

  const std::string label_ns = getDynamicLabelNamespace(layer.prefix);
  Marker label = makeDynamicLabelMarker(header, config, layer, viz_config, label_ns);
  msg.markers.push_back(label);
  published_dynamic_labels_.insert(label_ns);
}

void DynamicSceneGraphVisualizer::deleteLabel(const std_msgs::Header& header,
                                              char prefix,
                                              MarkerArray& msg) {
  const std::string label_ns = getDynamicLabelNamespace(prefix);

  if (published_dynamic_labels_.count(label_ns)) {
    Marker marker = makeDeleteMarker(header, 0, label_ns);
    msg.markers.push_back(marker);
  }
  published_dynamic_labels_.erase(label_ns);
}

void DynamicSceneGraphVisualizer::deleteDynamicLayer(const std_msgs::Header& header,
                                                     char prefix,
                                                     MarkerArray& msg) {
  const std::string node_ns = getDynamicNodeNamespace(prefix);
  deleteMultiMarker(header, node_ns, msg);

  const std::string edge_ns = getDynamicEdgeNamespace(prefix);
  deleteMultiMarker(header, edge_ns, msg);

  deleteLabel(header, prefix, msg);
}

void DynamicSceneGraphVisualizer::drawDynamicLayers(const std_msgs::Header& header,
                                                    MarkerArray& msg) {
  const VisualizerConfig& viz_config = visualizer_config_->get();

  for (const auto& id_layer_map_pair : scene_graph_->dynamicLayers()) {
    const LayerId layer_id = id_layer_map_pair.first;
    if (!layer_configs_.count(layer_id)) {
      continue;
    }

    const DynamicLayerConfig& config = getConfig(layer_id);

    for (const auto& prefix_layer_pair : id_layer_map_pair.second) {
      if (!config.visualize) {
        deleteDynamicLayer(header, prefix_layer_pair.first, msg);
        continue;
      }

      const DynamicSceneGraphLayer& layer = *prefix_layer_pair.second;
      drawDynamicLayer(header, layer, config, viz_config, msg);
    }
  }
}

void DynamicSceneGraphVisualizer::resetImpl(const std_msgs::Header& header,
                                            MarkerArray& msg) {
  auto to_delete = published_multimarkers_;
  for (const auto& ns : to_delete) {
    deleteMultiMarker(header, ns, msg);
  }

  for (const auto& id_layer_pair : scene_graph_->layers()) {
    const LayerId layer_id = id_layer_pair.first;
    const std::string label_ns = getLayerLabelNamespace(layer_id);
    const std::string bbox_ns = getLayerBboxNamespace(layer_id);
    clearPrevMarkers(header, {}, label_ns, prev_labels_.at(layer_id), msg);
    clearPrevMarkers(header, {}, bbox_ns, prev_bboxes_.at(layer_id), msg);
  }

  for (auto& label_set : prev_labels_) {
    label_set.second.clear();
  }
  for (auto& label_set : curr_labels_) {
    label_set.second.clear();
  }
  for (auto& bbox_set : prev_bboxes_) {
    bbox_set.second.clear();
  }
  for (auto& bbox_set : curr_bboxes_) {
    bbox_set.second.clear();
  }

  // vanilla scene graph also makes delete markers for dynamic layers, so we duplicate
  // them here (rviz checks for topic / namespace coherence)
  MarkerArray dynamic_msg = msg;
  auto to_delete_dynamic = published_dynamic_labels_;
  for (const auto& ns : to_delete_dynamic) {
    Marker marker = makeDeleteMarker(header, 0, ns);
    dynamic_msg.markers.push_back(marker);
  }

  if (!dynamic_msg.markers.empty()) {
    dynamic_layers_viz_pub_.publish(dynamic_msg);
  }

  published_dynamic_labels_.clear();

  for (const auto& plugin : plugins_) {
    plugin->reset(header, *scene_graph_);
  }
}

void DynamicSceneGraphVisualizer::redrawImpl(const std_msgs::Header& header,
                                             MarkerArray& msg) {
  for (const auto& id_layer_pair : scene_graph_->layers()) {
    if (!layer_configs_.count(id_layer_pair.first)) {
      continue;
    }

    LayerConfig config = layer_configs_.at(id_layer_pair.first)->get();
    const SceneGraphLayer& layer = *(id_layer_pair.second);

    if (!config.visualize) {
      deleteLayer(header, layer, msg);
    } else {
      drawLayer(header, layer, config, msg);
    }
  }

  drawLayerMeshEdges(header, mesh_edge_source_layer_, mesh_edge_ns_, msg);

  std::map<LayerId, LayerConfig> all_configs;
  for (const auto& id_manager_pair : layer_configs_) {
    all_configs[id_manager_pair.first] = id_manager_pair.second->get();
  }

  MarkerArray interlayer_edge_markers =
      makeGraphEdgeMarkers(header,
                           *scene_graph_,
                           all_configs,
                           visualizer_config_->get(),
                           interlayer_edge_ns_prefix_);

  std::set<std::string> seen_edge_labels;
  for (const auto& marker : interlayer_edge_markers.markers) {
    addMultiMarkerIfValid(marker, msg);
    seen_edge_labels.insert(marker.ns);
  }

  for (const auto& source_pair : all_configs) {
    for (const auto& target_pair : all_configs) {
      if (source_pair.first == target_pair.first) {
        continue;
      }

      const std::string curr_ns = interlayer_edge_ns_prefix_ +
                                  std::to_string(source_pair.first) + "_" +
                                  std::to_string(target_pair.first);
      if (seen_edge_labels.count(curr_ns)) {
        continue;
      }

      deleteMultiMarker(header, curr_ns, msg);
    }
  }

  MarkerArray dynamic_markers;
  drawDynamicLayers(header, dynamic_markers);

  std::map<LayerId, DynamicLayerConfig> all_dynamic_configs;
  for (const auto& id_manager_pair : dynamic_configs_) {
    all_dynamic_configs[id_manager_pair.first] = id_manager_pair.second->get();
  }

  const std::string dynamic_interlayer_edge_prefix = "dynamic_interlayer_edges_";
  MarkerArray dynamic_interlayer_edge_markers =
      makeDynamicGraphEdgeMarkers(header,
                                  *scene_graph_,
                                  all_configs,
                                  all_dynamic_configs,
                                  visualizer_config_->get(),
                                  dynamic_interlayer_edge_prefix);

  std::set<std::string> seen_dyn_edge_labels;
  for (const auto& marker : dynamic_interlayer_edge_markers.markers) {
    addMultiMarkerIfValid(marker, msg);
    seen_dyn_edge_labels.insert(marker.ns);
  }

  for (const auto& source_pair : all_configs) {
    for (const auto& target_pair : all_dynamic_configs) {
      std::string source_to_target_ns = dynamic_interlayer_edge_prefix +
                                        std::to_string(source_pair.first) + "_" +
                                        std::to_string(target_pair.first);
      if (!seen_dyn_edge_labels.count(source_to_target_ns)) {
        deleteMultiMarker(header, source_to_target_ns, msg);
      }

      std::string target_to_source_ns = dynamic_interlayer_edge_prefix +
                                        std::to_string(target_pair.first) + "_" +
                                        std::to_string(source_pair.first);
      if (!seen_dyn_edge_labels.count(target_to_source_ns)) {
        deleteMultiMarker(header, target_to_source_ns, msg);
      }
    }
  }

  if (!dynamic_markers.markers.empty()) {
    dynamic_layers_viz_pub_.publish(dynamic_markers);
  }

  // TODO(nathan) move to scene graph probably
  for (const auto& plugin : plugins_) {
    plugin->draw(header, *scene_graph_);
  }
}

bool DynamicSceneGraphVisualizer::hasConfigChanged() const {
  bool has_changed = false;

  has_changed |= visualizer_config_->hasChange();
  has_changed |= places_colormap_->hasChange();
  for (const auto& id_manager_pair : layer_configs_) {
    has_changed |= id_manager_pair.second->hasChange();
  }

  for (const auto& id_manager_pair : dynamic_configs_) {
    has_changed |= id_manager_pair.second->hasChange();
  }

  return has_changed;
}

void DynamicSceneGraphVisualizer::clearConfigChangeFlags() {
  visualizer_config_->clearChangeFlag();
  places_colormap_->clearChangeFlag();
  for (auto& id_manager_pair : layer_configs_) {
    id_manager_pair.second->clearChangeFlag();
  }

  for (auto& id_manager_pair : dynamic_configs_) {
    id_manager_pair.second->clearChangeFlag();
  }
}

void DynamicSceneGraphVisualizer::deleteMultiMarker(const std_msgs::Header& header,
                                                    const std::string& ns,
                                                    MarkerArray& msg) {
  if (!published_multimarkers_.count(ns)) {
    return;
  }

  Marker delete_marker = makeDeleteMarker(header, 0, ns);
  msg.markers.push_back(delete_marker);

  published_multimarkers_.erase(ns);
}

void DynamicSceneGraphVisualizer::addMultiMarkerIfValid(const Marker& marker,
                                                        MarkerArray& msg) {
  if (!marker.points.empty()) {
    msg.markers.push_back(marker);
    published_multimarkers_.insert(marker.ns);
    return;
  }

  deleteMultiMarker(marker.header, marker.ns, msg);
}

void DynamicSceneGraphVisualizer::setupConfigs(
    const DynamicSceneGraph::LayerIds& layer_ids) {
  ros::NodeHandle nh("");
  visualizer_config_.reset(new VisualizerConfigManager(nh, visualizer_ns_));

  const std::string colormap_ns = visualizer_ns_ + "/places_colormap";
  places_colormap_.reset(new ColormapConfigManager(nh, colormap_ns));

  for (const auto& layer : layer_ids) {
    const std::string layer_ns = visualizer_layer_ns_ + std::to_string(layer);
    layer_configs_[layer] = std::make_shared<LayerConfigManager>(nh, layer_ns);
  }
}

void DynamicSceneGraphVisualizer::displayLoop(const ros::WallTimerEvent&) {
  if (periodic_redraw_) {
    need_redraw_ = true;
  }
  redraw();
}

void DynamicSceneGraphVisualizer::deleteLayer(const std_msgs::Header& header,
                                              const SceneGraphLayer& layer,
                                              MarkerArray& msg) {
  deleteMultiMarker(header, getLayerNodeNamespace(layer.id), msg);
  deleteMultiMarker(header, getLayerEdgeNamespace(layer.id), msg);

  const std::string label_ns = getLayerLabelNamespace(layer.id);
  for (const auto& node : prev_labels_.at(layer.id)) {
    Marker marker = makeDeleteMarker(header, node, label_ns);
    msg.markers.push_back(marker);
  }
  prev_labels_.at(layer.id).clear();

  const std::string bbox_ns = getLayerBboxNamespace(layer.id);
  for (const auto& node : prev_bboxes_.at(layer.id)) {
    Marker marker = makeDeleteMarker(header, node, bbox_ns);
    msg.markers.push_back(marker);
  }
  prev_bboxes_.at(layer.id).clear();
}

void DynamicSceneGraphVisualizer::drawLayer(const std_msgs::Header& header,
                                            const SceneGraphLayer& layer,
                                            const LayerConfig& config,
                                            MarkerArray& msg) {
  const auto& viz_config = visualizer_config_->get();
  const std::string node_ns = getLayerNodeNamespace(layer.id);

  const bool color_by_distance = layer.id == DsgLayers::PLACES &&
                                 visualizer_config_->get().color_places_by_distance;

  Marker nodes;
  if (color_by_distance) {
    nodes = makeCentroidMarkers(
        header, config, layer, viz_config, node_ns, places_colormap_->get());
  } else if (layer.id == DsgLayers::PLACES) {
    nodes = makeCentroidMarkers(header,
                                config,
                                layer,
                                viz_config,
                                node_ns,
                                [&](const SceneGraphNode& node) -> NodeColor {
                                  auto parent = node.getParent();
                                  if (!parent) {
                                    return NodeColor::Zero();
                                  }

                                  return scene_graph_->getNode(*parent)
                                      .value()
                                      .get()
                                      .attributes<SemanticNodeAttributes>()
                                      .color;
                                });
  } else {
    nodes = makeCentroidMarkers(header, config, layer, viz_config, node_ns);
  }
  addMultiMarkerIfValid(nodes, msg);

  const std::string edge_ns = getLayerEdgeNamespace(layer.id);
  Marker edges = makeLayerEdgeMarkers(
      header, config, layer, viz_config, NodeColor::Zero(), edge_ns);
  addMultiMarkerIfValid(edges, msg);

  const std::string label_ns = getLayerLabelNamespace(layer.id);
  const std::string bbox_ns = getLayerBboxNamespace(layer.id);

  curr_labels_.at(layer.id).clear();
  curr_bboxes_.at(layer.id).clear();
  for (const auto& id_node_pair : layer.nodes()) {
    const Node& node = *id_node_pair.second;

    if (config.use_label) {
      Marker label = makeTextMarker(header, config, node, viz_config, label_ns);
      msg.markers.push_back(label);
      curr_labels_.at(layer.id).insert(node.id);
    }

    if (config.use_bounding_box) {
      try {
        node.attributes<SemanticNodeAttributes>();
      } catch (const std::bad_cast&) {
        continue;
      }

      Marker bbox = makeBoundingBoxMarker(header, config, node, viz_config, bbox_ns);
      msg.markers.push_back(bbox);
      curr_bboxes_.at(layer.id).insert(node.id);
    }
  }

  clearPrevMarkers(
      header, curr_labels_.at(layer.id), label_ns, prev_labels_.at(layer.id), msg);
  clearPrevMarkers(
      header, curr_bboxes_.at(layer.id), bbox_ns, prev_bboxes_.at(layer.id), msg);
}

void DynamicSceneGraphVisualizer::drawLayerMeshEdges(const std_msgs::Header& header,
                                                     LayerId layer_id,
                                                     const std::string& ns,
                                                     MarkerArray& msg) {
  if (!scene_graph_->hasLayer(layer_id)) {
    return;
  }

  if (!layer_configs_.count(layer_id)) {
    return;
  }

  LayerConfig config = layer_configs_.at(layer_id)->get();
  if (!config.visualize) {
    deleteMultiMarker(header, ns, msg);
    return;
  }

  Marker mesh_edges = makeMeshEdgesMarker(header,
                                          config,
                                          visualizer_config_->get(),
                                          *scene_graph_,
                                          scene_graph_->getLayer(layer_id),
                                          ns);
  addMultiMarkerIfValid(mesh_edges, msg);
}

}  // namespace hydra
