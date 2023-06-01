#include "hydra_rviz_plugin/scene_graph_display.h"

#include <spark_dsg/graph_binary_serialization.h>

#include "hydra_rviz_plugin/scene_graph_visual.h"

namespace hydra {

using SGViz = SceneGraphVisual;

SceneGraphDisplay::SceneGraphDisplay() {}

SceneGraphDisplay::~SceneGraphDisplay() {}

void SceneGraphDisplay::onInitialize() {
  rviz::MessageFilterDisplay<hydra_msgs::DsgUpdate>::onInitialize();
}

void SceneGraphDisplay::reset() {
  rviz::MessageFilterDisplay<hydra_msgs::DsgUpdate>::reset();
  visual_.reset();
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

  if (!visual_) {
    visual_ = std::make_unique<SGViz>(context_->getSceneManager(), scene_node_);
  }

  visual_->setMessage(*graph_);
}

}  // namespace hydra

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hydra::SceneGraphDisplay, rviz::Display)
