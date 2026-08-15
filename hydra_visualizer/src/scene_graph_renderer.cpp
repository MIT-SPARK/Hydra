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
#include "hydra_visualizer/scene_graph_renderer.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/types/collections.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/printing.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include "hydra_visualizer/color/colormap_utilities.h"
#include "hydra_visualizer/drawing.h"

namespace hydra {

using namespace spark_dsg;
using namespace visualizer;

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace {

inline std::string keyToLayerName(LayerKey key) {
  std::stringstream ss;
  ss << "layer" << key.layer;
  if (key.partition) {
    ss << "p" << key.partition;
  }

  return ss.str();
}

struct MarkerNamespaces {
  static std::string layerNodeNamespace(LayerKey key) {
    return keyToLayerName(key) + "_nodes";
  }

  static std::string layerEdgeNamespace(LayerKey key) {
    return keyToLayerName(key) + "_edges";
  }

  static std::string layerTextNamespace(LayerKey key) {
    return keyToLayerName(key) + "_text";
  }

  static std::string layerBboxNamespace(LayerKey key) {
    return keyToLayerName(key) + "_bounding_boxes";
  }

  static std::string layerBoundaryNamespace(LayerKey key) {
    return keyToLayerName(key) + "_polygon_boundaries";
  }

  static std::string layerBoundaryEllipseNamespace(LayerKey key) {
    return keyToLayerName(key) + "_ellipsoid_boundaries";
  }

  static std::string layerBoundaryEdgeNamespace(LayerKey key) {
    return keyToLayerName(key) + "_polygon_boundaries_edges";
  }

  static std::string meshEdgeNamespace(LayerKey key) {
    return keyToLayerName(key) + "_mesh_edges";
  }
};

struct InterlayerInfo {
  using EdgeColor = EdgeColorAdapter::EdgeColor;
  using ColorFunc = std::function<EdgeColor(const SceneGraphEdge& edge)>;
  InterlayerEdgeConfig config;
  size_t marker_idx;
  std::unique_ptr<EdgeColorAdapter> adapter;
  size_t num_since_last = 0;
};

}  // namespace

InterlayerEdgeConfig::InterlayerEdgeConfig() { color = {}; }

void declare_config(InterlayerEdgeConfig& config) {
  using namespace config;
  name("InterlayerEdgeConfig");
  base<visualizer::LayerConfig::Edges>(config);
  field(config.use_child_color, "use_child_color");
}

void declare_config(SceneGraphRenderer::Config::InterlayerEdges& config) {
  using namespace config;
  name("SceneGraphRenderer::Config::InterlayerEdges");
  field(config.config, "");
  field<SelectorConversion>(config.from, "from");
  field<SelectorConversion>(config.to, "to");
}

void declare_config(GraphRenderConfig& config) {
  using namespace config;
  name("GraphRenderConfig");
  field(config.layer_z_step, "layer_z_step");
  field(config.collapse_layers, "collapse_layers");
}

void declare_config(SceneGraphRenderer::LayerPluginsConfig& config) {
  using namespace config;
  name("SceneGraphRenderer::LayerPluginsConfig");
  field<LayerKeyConversion>(config.layer, "layer");
  field(config.plugins, "plugins");
}

void declare_config(SceneGraphRenderer::Config& config) {
  using namespace config;
  name("SceneGraphRenderer::Config");
  field(config.graph, "graph");
  field<MapKeyConverter<SelectorConversion>>(config.layers, "layers");
  field(config.interlayer_edges, "interlayer_edges");
  field(config.layer_plugins, "layer_plugins");
}

SceneGraphRenderer::SceneGraphRenderer(const Config& config, ianvs::NodeHandle nh)
    : init_config_(config),
      nh_(nh),
      graph_config_("scene_graph", config.graph, [this]() { has_change_ = true; }),
      pub_(nh.create_publisher<MarkerArray>("graph", rclcpp::QoS(1).transient_local())),
      has_change_(false) {
  for (const auto& plugin_config : config.layer_plugins) {
    const auto key = plugin_config.layer;
    auto& plugin_list = layer_plugins_[key];
    for (const auto& plugin : plugin_config.plugins) {
      plugin_list.emplace_back(plugin.create(keyToLayerName(key)));
    }
  }
}

void SceneGraphRenderer::reset(const std_msgs::msg::Header& header) {
  MarkerArray msg;
  tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    pub_->publish(msg);
  }
}

bool SceneGraphRenderer::hasChange() const {
  if (has_change_) {
    return true;
  }

  for (const auto& [key, plugins] : layer_plugins_) {
    for (const auto& plugin : plugins) {
      if (plugin && plugin->hasChange()) {
        return true;
      }
    }
  }

  return false;
}

void SceneGraphRenderer::clearChangeFlag() {
  has_change_ = false;
  for (const auto& [key, plugins] : layer_plugins_) {
    for (const auto& plugin : plugins) {
      if (plugin) {
        plugin->clearChangeFlag();
      }
    }
  }
}

void SceneGraphRenderer::draw(const std_msgs::msg::Header& header,
                              const DynamicSceneGraph& graph) const {
  setConfigs(graph);

  MarkerArray msg;
  for (const auto& [_, layer] : graph.layers()) {
    drawLayer(header, layer_infos_.at(layer->id), *layer, graph.mesh().get(), msg);
  }

  for (const auto& [_, partitions] : graph.layer_partitions()) {
    for (const auto& [_, partition] : partitions) {
      const auto& partition_info = layer_infos_.at(partition->id);
      drawLayer(header, partition_info, *partition, graph.mesh().get(), msg);
    }
  }

  MarkerArray edges;
  drawInterlayerEdges(header, graph, edges);
  tracker_.add(edges, msg);
  tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    pub_->publish(msg);
  }
}

YAML::Node SceneGraphRenderer::dumpConfig() const {
  YAML::Node layers(YAML::NodeType::Map);
  for (const auto& [key, wrapper] : layers_) {
    layers[LayerKeySelector{key}.str()] = config::toYaml(wrapper->get());
  }

  YAML::Node layer_plugins(YAML::NodeType::Sequence);
  for (const auto& [key, plugins] : layer_plugins_) {
    YAML::Node plugin_configs(YAML::NodeType::Sequence);
    for (const auto& plugin : plugins) {
      if (!plugin) {
        continue;
      }

      plugin_configs.push_back(plugin->dumpConfig());
    }

    YAML::Node layer_info;
    layer_info["layer"] = LayerKeySelector{key}.str();
    layer_info["plugins"] = plugin_configs;
    layer_plugins.push_back(layer_info);
  }

  YAML::Node interlayer_edge_configs(YAML::NodeType::Sequence);
  for (const auto& [_, info] : interlayer_edges_) {
    YAML::Node this_config = config::toYaml(info.config->get());
    this_config["from"] = LayerKeySelector{info.parent}.str();
    this_config["to"] = LayerKeySelector{info.child}.str();
    interlayer_edge_configs.push_back(this_config);
  }

  YAML::Node root = config::toYaml(graph_config_.get());
  root["layers"] = layers;
  root["layer_plugins"] = layer_plugins;
  root["interlayer_edges"] = interlayer_edge_configs;
  return root;
}

InterlayerEdgeConfig SceneGraphRenderer::getInterlayerEdgeConfig(LayerKey parent,
                                                                 LayerKey child) const {
  const auto name = keyToLayerName(parent) + "_to_" + keyToLayerName(child);
  auto iter = interlayer_edges_.find(name);
  if (iter == interlayer_edges_.end()) {
    InterlayerEdgeConfig config;
    for (const auto& info : init_config_.interlayer_edges) {
      bool from_parent = info.from.matches(parent) && info.to.matches(child);
      bool to_parent = info.to.matches(parent) && info.from.matches(child);
      if (!from_parent && !to_parent) {
        continue;
      }

      config = info.config;
      if (info.from.wildcard || info.to.wildcard) {
        continue;  // allow more specific keys to take precedence
      } else {
        break;
      }
    }

    const auto ns = "scene_graph_interlayer_" + name;
    auto wrapper = std::make_unique<EdgeConfigWrapper>(ns, config);
    wrapper->setCallback([this]() { has_change_ = true; });
    iter = interlayer_edges_
               .emplace(name, EdgeConfigInfo{parent, child, std::move(wrapper)})
               .first;
  }

  return iter->second.config->get();
}

void SceneGraphRenderer::drawInterlayerEdges(const std_msgs::msg::Header& header,
                                             const DynamicSceneGraph& graph,
                                             MarkerArray& msg) const {
  const std::string ns_prefix = "interlayer_edges_";
  std::map<std::pair<LayerKey, LayerKey>, InterlayerInfo> edge_info;
  for (const auto& [key, edge] : graph.interlayer_edges()) {
    const auto& source = graph.getNode(edge.source);
    const auto& target = graph.getNode(edge.target);
    const auto& source_info = getLayerInfo(source.layer);
    const auto& target_info = getLayerInfo(target.layer);
    if (!source_info.shouldVisualize(source) || !target_info.shouldVisualize(target)) {
      continue;
    }

    const auto target_is_parent = source.layer < target.layer;
    auto keys = target_is_parent ? std::make_pair(target.layer, source.layer)
                                 : std::make_pair(source.layer, target.layer);
    auto iter = edge_info.find(keys);
    if (iter == edge_info.end()) {
      std::stringstream ss;
      ss << ns_prefix << source.layer << "_" << target.layer;
      const auto new_conf = getInterlayerEdgeConfig(keys.first, keys.second);

      InterlayerInfo new_info{new_conf, msg.markers.size(), new_conf.color.create()};
      new_info.num_since_last = new_conf.insertion_skip;
      if (new_info.adapter) {
        // TODO(nathan) this is awkard because it only supports intralayer edges
        new_info.adapter->setGraph(graph, keys.first);
      }

      iter = edge_info.emplace(keys, std::move(new_info)).first;
      auto& marker = msg.markers.emplace_back();
      marker.header = header;
      marker.type = Marker::LINE_LIST;
      marker.action = Marker::ADD;
      marker.id = 0;
      marker.ns = ss.str();
      marker.scale.x = new_conf.scale;
    }

    auto& info = iter->second;
    if (!info.config.draw) {
      continue;
    }

    if (info.num_since_last >= info.config.insertion_skip) {
      info.num_since_last = 0;
    } else {
      ++info.num_since_last;
      continue;
    }

    auto& marker = msg.markers.at(info.marker_idx);
    auto& source_point = marker.points.emplace_back();
    tf2::convert(source.attributes().position, source_point);
    source_point.z += source_info.z_offset;

    auto& target_point = marker.points.emplace_back();
    tf2::convert(target.attributes().position, target_point);
    target_point.z += target_info.z_offset;

    if (info.adapter) {
      const auto [c_s, c_t] = info.adapter->getColor(graph, edge);
      marker.colors.push_back(makeColorMsg(c_s, info.config.alpha));
      marker.colors.push_back(makeColorMsg(c_t, info.config.alpha));
    } else {
      bool use_source = !target_is_parent ^ info.config.use_child_color;
      const auto color =
          use_source ? source_info.node_color(source) : target_info.node_color(target);
      marker.colors.push_back(makeColorMsg(color, info.config.alpha));
      marker.colors.push_back(makeColorMsg(color, info.config.alpha));
    }
  }
}

void SceneGraphRenderer::drawLayer(const std_msgs::msg::Header& header,
                                   const LayerInfo& info,
                                   const SceneGraphLayer& layer,
                                   const Mesh* mesh,
                                   MarkerArray& msg) const {
  if (!info.config.visualize) {
    return;
  }

  if (info.config.draw_frontier_ellipse) {
    info.filter = [](const SceneGraphNode& node) {
      auto attrs = node.tryAttributes<PlaceNodeAttributes>();
      return attrs ? attrs->real_place : true;
    };
  }

  const auto node_ns = MarkerNamespaces::layerNodeNamespace(layer.id);
  if (info.config.nodes.draw) {
    tracker_.add(makeLayerNodeMarkers(header, info, layer, node_ns), msg);
  }

  if (info.config.text.draw) {
    if (info.config.text.draw_layer) {
      LOG_FIRST_N(WARNING, 5) << "use_text and use_layer_text are mutually exclusive!";
    }

    const auto ns = MarkerNamespaces::layerTextNamespace(layer.id);
    tracker_.add(makeLayerNodeTextMarkers(header, info, layer, ns), msg);
  } else if (info.config.text.draw_layer && !layer.nodes().empty()) {
    const auto ns = MarkerNamespaces::layerTextNamespace(layer.id);
    tracker_.add(makeLayerTextMarker(header, info, layer, ns), msg);
  }

  if (info.config.bounding_boxes.draw) {
    const auto ns = MarkerNamespaces::layerBboxNamespace(layer.id);
    tracker_.add(makeLayerBoundingBoxes(header, info, layer, ns), msg);
  }

  if (info.config.boundaries.draw) {
    const auto ns = MarkerNamespaces::layerBoundaryNamespace(layer.id);
    const auto edge_ns = MarkerNamespaces::layerBoundaryEdgeNamespace(layer.id);
    tracker_.add(makeLayerPolygonBoundaries(header, info, layer, ns), msg);
    if (info.config.boundaries.collapse) {
      tracker_.add(makeLayerPolygonEdges(header, info, layer, edge_ns), msg);
    }
  }

  if (info.config.boundaries.draw_ellipse) {
    const auto ns = MarkerNamespaces::layerBoundaryEllipseNamespace(layer.id);
    tracker_.add(makeLayerEllipseBoundaries(header, info, layer, ns), msg);
  }

  if (info.config.draw_frontier_ellipse) {
    tracker_.add(makeEllipsoidMarkers(header, info, layer, "frontier_ns"), msg);
    info.filter = {};  // we reset the manual filter to draw edges to frontiers
  }

  const auto edge_ns = MarkerNamespaces::layerEdgeNamespace(layer.id);
  tracker_.add(makeLayerEdgeMarkers(header, info, layer, edge_ns), msg);

  // dispatch drawing to layer plugins
  auto plugins = layer_plugins_.find(layer.id);
  if (plugins != layer_plugins_.end()) {
    for (const auto& plugin : plugins->second) {
      if (!plugin) {
        continue;
      }

      plugin->draw(header, info, layer, mesh, msg, tracker_);
    }
  }
}

LayerConfig SceneGraphRenderer::getLayerConfig(spark_dsg::LayerKey key) const {
  auto iter = layers_.find(key);
  if (iter == layers_.end()) {
    // TODO(nathan) actually allow layer names when looking up configs
    LayerConfig init_config;
    for (const auto& [config_key, config] : init_config_.layers) {
      if (!config_key.matches(key)) {
        continue;
      }

      init_config = config;
      if (config_key.wildcard) {
        continue;  // allow more specific keys to take precedence
      } else {
        break;
      }
    }

    const auto ns = "scene_graph_" + keyToLayerName(key);
    iter = layers_.emplace(key, std::make_unique<LayerConfigWrapper>(ns, init_config))
               .first;
    iter->second->setCallback([this]() { has_change_ = true; });
  }

  return iter->second->get();
}

void SceneGraphRenderer::setConfigs(const DynamicSceneGraph& graph) const {
  layer_infos_.clear();
  const auto graph_config = graph_config_.get();
  const auto z_step = graph_config.layer_z_step;
  const auto collapse = graph_config.collapse_layers;
  for (const auto& [_, layer] : graph.layers()) {
    const auto layer_config = getLayerConfig(layer->id);
    auto& [_key, info] = *layer_infos_.emplace(layer->id, layer_config).first;
    info.offset(z_step, collapse).graph(graph, layer->id);
  }

  for (const auto& [_, partitions] : graph.layer_partitions()) {
    for (const auto& [_, layer] : partitions) {
      const auto layer_config = getLayerConfig(layer->id);
      auto& [_key, info] = *layer_infos_.emplace(layer->id, layer_config).first;
      info.offset(z_step, collapse).graph(graph, layer->id);
    }
  }
}

const LayerInfo& SceneGraphRenderer::getLayerInfo(LayerKey layer) const {
  return layer_infos_.at(layer);
}

}  // namespace hydra
