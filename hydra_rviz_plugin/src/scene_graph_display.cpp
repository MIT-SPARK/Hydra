#include "hydra_rviz_plugin/scene_graph_display.h"

#include <ros/package.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <spark_dsg/graph_binary_serialization.h>
#include <yaml-cpp/yaml.h>

#include "hydra_rviz_plugin/layer_visual.h"

namespace hydra {

struct Pose {
  Pose(const Ogre::Quaternion& rot, const Ogre::Vector3& pos) : rot(rot), pos(pos) {}

  Ogre::Quaternion rot;
  Ogre::Vector3 pos;
};

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

  for (auto& id_layer_pair : layers_) {
    auto& layer = id_layer_pair.second;

    Ogre::Vector3 layer_pos(
        0.0f,
        0.0f,
        collapse_layers_->getBool()
            ? 0.0
            : layer.config.offset_scale * layer_height_->getFloat());
    layer_pos = last_pose_->rot * layer_pos + last_pose_->pos;
    layer.visual->setPose(last_pose_->rot, layer_pos);
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

  for (auto& id_layer_pair : layers_) {
    auto& layer = id_layer_pair.second;
    layer.visual->setMessage(layer.config, graph_->getLayer(id_layer_pair.first));
  }
}

}  // namespace hydra

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hydra::SceneGraphDisplay, rviz::Display)
