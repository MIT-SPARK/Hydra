#include "hydra_rviz_plugin/scene_graph_display.h"

#include <spark_dsg/graph_binary_serialization.h>

#include "hydra_rviz_plugin/layer_visual.h"

namespace hydra {

SceneGraphDisplay::SceneGraphDisplay() {}

SceneGraphDisplay::~SceneGraphDisplay() {}

void SceneGraphDisplay::onInitialize() {
  rviz::MessageFilterDisplay<hydra_msgs::DsgUpdate>::onInitialize();
}

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

    iter->second->setMessage(graph_->getLayer(layer));
  }
}

}  // namespace hydra

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hydra::SceneGraphDisplay, rviz::Display)
