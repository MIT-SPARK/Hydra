#include "hydra_rviz_plugin/scene_graph_display.h"

#include <ros/package.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <spark_dsg/graph_binary_serialization.h>
#include <yaml-cpp/yaml.h>

#include "hydra_rviz_plugin/bounding_box_visual.h"
#include "hydra_rviz_plugin/color_functions.h"
#include "hydra_rviz_plugin/colormap.h"
#include "hydra_rviz_plugin/edge_config.h"
#include "hydra_rviz_plugin/interlayer_edge_visual.h"
#include "hydra_rviz_plugin/layer_visual.h"

namespace hydra {

using spark_dsg::NodeSymbol;
using spark_dsg::SceneGraphNode;

SceneGraphDisplay::SceneGraphDisplay() {
  collapse_layers_ =
      std::make_unique<rviz::BoolProperty>("Collapse Layers",
                                           false,
                                           "Draw layers without any z offset",
                                           this,
                                           SLOT(updateProperties()));
  layer_height_ = std::make_unique<rviz::FloatProperty>("Layer Height",
                                                        5.0,
                                                        "Nominal height between layers",
                                                        this,
                                                        SLOT(updateProperties()));
}

SceneGraphDisplay::~SceneGraphDisplay() {}

void SceneGraphDisplay::onInitialize() {
  rviz::MessageFilterDisplay<hydra_msgs::DsgUpdate>::onInitialize();
  readDefaults();
}

#define PARSE_FIELD(yaml_node, config, name)                    \
  if (yaml_node[#name]) {                                       \
    config.name = yaml_node[#name].as<decltype(config.name)>(); \
  }                                                             \
  static_assert(true, "")

void parseLayerConfig(const YAML::Node& node, LayerConfig& config) {
  PARSE_FIELD(node, config, offset_scale);
  PARSE_FIELD(node, config, visualize);
  PARSE_FIELD(node, config, node_scale);
  PARSE_FIELD(node, config, node_alpha);
  PARSE_FIELD(node, config, use_spheres);
  PARSE_FIELD(node, config, use_label);
  PARSE_FIELD(node, config, label_height_ratio);
  PARSE_FIELD(node, config, label_scale);
  PARSE_FIELD(node, config, use_bounding_box);
  PARSE_FIELD(node, config, collapse_bounding_box);
  PARSE_FIELD(node, config, bounding_box_scale);
  PARSE_FIELD(node, config, bounding_box_alpha);
  PARSE_FIELD(node, config, edge_scale);
  PARSE_FIELD(node, config, edge_alpha);
  PARSE_FIELD(node, config, label_colormap);

  std::string color_mode = "none";
  if (node["color_mode"]) {
    color_mode = node["color_mode"].as<std::string>();
  }
  config.color_mode = stringToColorMode(color_mode);
}

void parseEdgeConfig(const YAML::Node& node, EdgeConfig& config) {
  PARSE_FIELD(node, config, insertion_skip);
  PARSE_FIELD(node, config, visualize);
  PARSE_FIELD(node, config, edge_scale);
  PARSE_FIELD(node, config, edge_alpha);

  std::string color_mode = "none";
  if (node["color_mode"]) {
    color_mode = node["color_mode"].as<std::string>();
  }
  config.color_mode = stringToEdgeColorMode(color_mode);
}

void SceneGraphDisplay::readDefaults() {
  std::string default_file =
      ros::package::getPath("hydra_rviz_plugin") + "/config/defaults.yaml";
  auto node = YAML::LoadFile(default_file);
  if (node["layers"]) {
    for (const auto& lc : node["layers"]) {
      const auto id = lc["id"].as<spark_dsg::LayerId>();
      LayerConfig config;
      parseLayerConfig(lc, config);
      default_configs_[id] = config;
      ROS_DEBUG_STREAM("parsed layer " << id << ": " << config);
    }
  }

  if (node["edges"]) {
    for (const auto& ec : node["edges"]) {
      const auto from = ec["from"].as<spark_dsg::LayerId>();
      const auto to = ec["to"].as<spark_dsg::LayerId>();
      const LayerPair layer_pair(from, to);

      EdgeConfig config;
      parseEdgeConfig(ec, config);
      default_edge_configs_[layer_pair] = config;
      ROS_DEBUG_STREAM("parsed edge config " << layer_pair << ": " << config);
    }
  }
}

#undef PARSE_FIELD

void SceneGraphDisplay::reset() {
  rviz::MessageFilterDisplay<hydra_msgs::DsgUpdate>::reset();
  layers_.clear();
}

void SceneGraphDisplay::updateProperties() {
  if (!last_pose_) {
    return;
  }

  setLayerPoses();
}

void SceneGraphDisplay::initLayers() {
  for (const auto layer : graph_->layer_ids) {
    auto iter = layers_.find(layer);
    if (iter != layers_.end()) {
      continue;
    }

    iter = layers_.emplace(layer, LayerContainer()).first;
    iter->second.visual =
        std::make_unique<LayerVisual>(context_->getSceneManager(), scene_node_);

    auto citer = default_configs_.find(layer);
    if (citer == default_configs_.end()) {
      continue;
    }

    iter->second.config = citer->second;
  }
}

void SceneGraphDisplay::setLayerPoses() {
  if (!last_pose_) {
    return;
  }

  const bool collapse_layers = collapse_layers_->getBool();
  const float layer_height = layer_height_->getFloat();

  for (auto& id_layer_pair : layers_) {
    auto& layer = id_layer_pair.second;

    if (collapse_layers) {
      layer.offset = Pose();
      layer.pose = *last_pose_;
    } else {
      layer.offset = Pose::ZOffset(layer.config.offset_scale * layer_height);
      layer.pose = (*last_pose_) * layer.offset;
    }

    layer.visual->setPose(layer.pose);

    if (!layer.config.use_bounding_box) {
      layer.bbox_visual.reset();
      continue;
    }

    if (!layer.bbox_visual) {
      layer.bbox_visual =
          std::make_unique<BoundingBoxVisual>(context_->getSceneManager(), scene_node_);
    }

    layer.bbox_visual->setPose(layer.config.collapse_bounding_box ? *last_pose_
                                                                  : layer.pose);
  }
}

void SceneGraphDisplay::assignColorFunctions() {
  static const auto layer_colormap = Colormap::SingleColor();

  for (auto& id_layer_pair : layers_) {
    auto& layer = id_layer_pair.second;
    switch (layer.config.color_mode) {
      case LayerConfig::ColorMode::COLORMAP:
        layer.node_color_callback = std::make_unique<ColormapFunctor>(
            Colormap::Default(), layer.config.label_colormap);
        break;
      case LayerConfig::ColorMode::PARENT_COLOR:
        layer.node_color_callback = std::make_unique<ParentColorFunctor>(
            Colormap::Default(), graph_, layer.config.label_colormap);
        break;
      case LayerConfig::ColorMode::SINGLE_COLOR:
        layer.node_color_callback =
            std::make_unique<SingleColorFunctor>(layer_colormap(id_layer_pair.first));
        break;
      default:
        layer.node_color_callback.reset(nullptr);
        break;
    }
  }
}

bool SceneGraphDisplay::readGraph(const hydra_msgs::DsgUpdate::ConstPtr& msg) {
  try {
    if (!graph_) {
      graph_ = spark_dsg::readGraph(msg->layer_contents);
    } else {
      spark_dsg::updateGraph(*graph_, msg->layer_contents, true);
    }
  } catch (const std::exception& e) {
    // TODO(nathan) log to plugin
    ROS_ERROR_STREAM("Received invalid message: " << e.what());
    return false;
  }

  return true;
}

bool SceneGraphDisplay::readPose(const hydra_msgs::DsgUpdate::ConstPtr& msg) {
  Ogre::Quaternion rot;
  Ogre::Vector3 pos;
  auto fmanager = context_->getFrameManager();
  if (!fmanager->getTransform(msg->header.frame_id, msg->header.stamp, pos, rot)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(),
              qPrintable(fixed_frame_));
    return false;
  }

  last_pose_ = std::make_unique<Pose>(rot, pos);
  return true;
}

void SceneGraphDisplay::updateLayerVisuals() {
  for (auto& id_layer_pair : layers_) {
    auto& layer = id_layer_pair.second;
    layer.visual->setMessage(layer.config,
                             graph_->getLayer(id_layer_pair.first),
                             layer.node_color_callback.get());

    if (layer.config.use_bounding_box && layer.bbox_visual) {
      layer.bbox_visual->setMessage(layer.config,
                                    graph_->getLayer(id_layer_pair.first),
                                    layer.node_color_callback.get(),
                                    layer.offset);
    }
  }
}

void SceneGraphDisplay::updateDynamicLayerVisuals() {
  for (auto& id_layer_pair : layers_) {
    auto& layer = id_layer_pair.second;
    layer.visual->setMessage(layer.config,
                             graph_->getLayer(id_layer_pair.first),
                             layer.node_color_callback.get());
  }
}

void SceneGraphDisplay::updateInterlayerEdgeVisual() {
  if (!interlayer_edges_) {
    interlayer_edges_ = std::make_unique<InterlayerEdgeVisual>(
        context_->getSceneManager(), scene_node_);
  }

  for (const auto& id_edge_pair : graph_->interlayer_edges()) {
    const auto& source = graph_->getNode(id_edge_pair.second.source)->get();
    const auto& target = graph_->getNode(id_edge_pair.second.target)->get();
    const LayerPair layer_pair(source.layer, target.layer);
    auto citer = edge_configs_.find(layer_pair);
    if (citer != edge_configs_.end()) {
      continue;
    }

    auto diter = default_edge_configs_.find(layer_pair);
    if (diter != default_edge_configs_.end()) {
      edge_configs_[layer_pair] = diter->second;
    } else {
      edge_configs_[layer_pair] = EdgeConfig();
    }
  }

  interlayer_edges_->setPose(*last_pose_);
  interlayer_edges_->setMessage(*graph_, layers_, edge_configs_);
}

void SceneGraphDisplay::processMessage(const hydra_msgs::DsgUpdate::ConstPtr& msg) {
  if (!readGraph(msg)) {
    return;
  }

  if (!readPose(msg)) {
    return;
  }

  initLayers();
  setLayerPoses();
  assignColorFunctions();

  updateLayerVisuals();
  updateDynamicLayerVisuals();
  updateInterlayerEdgeVisual();
}

}  // namespace hydra

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hydra::SceneGraphDisplay, rviz::Display)
