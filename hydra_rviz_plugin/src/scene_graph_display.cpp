#include "hydra_rviz_plugin/scene_graph_display.h"

#include <ros/package.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <spark_dsg/graph_binary_serialization.h>
#include <yaml-cpp/yaml.h>

#include "hydra_rviz_plugin/bounding_box_visual.h"
#include "hydra_rviz_plugin/color_functions.h"
#include "hydra_rviz_plugin/colormap.h"
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

void SceneGraphDisplay::readDefaults() {
  std::string default_file =
      ros::package::getPath("hydra_rviz_plugin") + "/config/defaults.yaml";
  auto node = YAML::LoadFile(default_file);
  if (node["layers"]) {
    for (const auto& lc : node["layers"]) {
      const auto id = lc["id"].as<spark_dsg::LayerId>();
      LayerConfig config;
      PARSE_FIELD(lc, config, offset_scale);
      PARSE_FIELD(lc, config, visualize);
      PARSE_FIELD(lc, config, node_scale);
      PARSE_FIELD(lc, config, node_alpha);
      PARSE_FIELD(lc, config, use_spheres);
      PARSE_FIELD(lc, config, use_label);
      PARSE_FIELD(lc, config, label_height_ratio);
      PARSE_FIELD(lc, config, label_scale);
      PARSE_FIELD(lc, config, use_bounding_box);
      PARSE_FIELD(lc, config, collapse_bounding_box);
      PARSE_FIELD(lc, config, bounding_box_scale);
      PARSE_FIELD(lc, config, bounding_box_alpha);
      PARSE_FIELD(lc, config, edge_scale);
      PARSE_FIELD(lc, config, edge_alpha);
      PARSE_FIELD(lc, config, label_colormap);

      std::string color_mode = "none";
      if (lc["color_mode"]) {
        color_mode = lc["color_mode"].as<std::string>();
      }
      config.color_mode = stringToColorMode(color_mode);
      default_configs_[id] = config;
      ROS_DEBUG_STREAM("parsed layer " << id << ": " << config);
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
      layer.pose = *last_pose_;
    } else {
      const auto offset = Pose::ZOffset(layer.config.offset_scale * layer_height);
      layer.pose = (*last_pose_) * offset;
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

void SceneGraphDisplay::processMessage(const hydra_msgs::DsgUpdate::ConstPtr& msg) {
  Ogre::Quaternion rot;
  Ogre::Vector3 pos;
  auto fmanager = context_->getFrameManager();
  if (!fmanager->getTransform(msg->header.frame_id, msg->header.stamp, pos, rot)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(),
              qPrintable(fixed_frame_));
    return;
  }

  last_pose_ = std::make_unique<Pose>(rot, pos);

  try {
    if (!graph_) {
      graph_ = spark_dsg::readGraph(msg->layer_contents);
    } else {
      spark_dsg::updateGraph(*graph_, msg->layer_contents, true);
    }
  } catch (const std::exception& e) {
    // TODO(nathan) log to plugin
    ROS_ERROR_STREAM("Received invalid message: " << e.what());
    return;
  }

  initLayers();
  setLayerPoses();
  assignColorFunctions();

  for (auto& id_layer_pair : layers_) {
    auto& layer = id_layer_pair.second;
    layer.visual->setMessage(layer.config,
                             graph_->getLayer(id_layer_pair.first),
                             layer.node_color_callback.get());

    if (layer.config.use_bounding_box && layer.bbox_visual) {
      layer.bbox_visual->setMessage(layer.config,
                                    graph_->getLayer(id_layer_pair.first),
                                    layer.node_color_callback.get());
    }
  }
}

}  // namespace hydra

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hydra::SceneGraphDisplay, rviz::Display)
