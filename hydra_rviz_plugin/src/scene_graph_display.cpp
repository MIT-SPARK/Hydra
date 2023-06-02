#include "hydra_rviz_plugin/scene_graph_display.h"

#include <ros/package.h>
#include <spark_dsg/graph_binary_serialization.h>
#include <yaml-cpp/yaml.h>

#include "hydra_rviz_plugin/layer_visual.h"

namespace hydra {

SceneGraphDisplay::SceneGraphDisplay() {}

SceneGraphDisplay::~SceneGraphDisplay() {}

#define PARSE_FIELD(yaml_node, config, name)                    \
  if (yaml_node[#name]) {                                       \
    config.name = yaml_node[#name].as<decltype(config.name)>(); \
  }                                                             \
  static_assert(true, "")

void SceneGraphDisplay::onInitialize() {
  rviz::MessageFilterDisplay<hydra_msgs::DsgUpdate>::onInitialize();

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
    }
  }
}

#undef PARSE_FIELD

void SceneGraphDisplay::reset() {
  rviz::MessageFilterDisplay<hydra_msgs::DsgUpdate>::reset();
  layer_visuals_.clear();
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

  try {
    if (!graph_) {
      graph_ = spark_dsg::readGraph(msg->layer_contents);
    } else {
      spark_dsg::updateGraph(*graph_, msg->layer_contents, true);
    }
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Received invalid message: " << e.what());
    return;
  }

  for (const auto layer : graph_->layer_ids) {
    auto iter = layer_visuals_.find(layer);
    if (iter == layer_visuals_.end()) {
      auto visual =
          std::make_unique<LayerVisual>(context_->getSceneManager(), scene_node_);
      iter = layer_visuals_.emplace(layer, std::move(visual)).first;
    }

    auto citer = default_configs_.find(layer);
    if (citer != default_configs_.end()) {
      iter->second->config = citer->second;
    }

    iter->second->setMessage(graph_->getLayer(layer));
  }
}

}  // namespace hydra

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hydra::SceneGraphDisplay, rviz::Display)
