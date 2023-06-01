#pragma once
#include <hydra_msgs/DsgUpdate.h>
#include <spark_dsg/dynamic_scene_graph.h>
#include <rviz/message_filter_display.h>

#include <memory>

namespace hydra {

class SceneGraphVisual;

class SceneGraphDisplay : public rviz::MessageFilterDisplay<hydra_msgs::DsgUpdate> {
  Q_OBJECT
 public:
  SceneGraphDisplay();

  virtual ~SceneGraphDisplay();

 protected:
  virtual void onInitialize();

  virtual void reset();

 private:
  void processMessage(const hydra_msgs::DsgUpdate::ConstPtr& msg);

  spark_dsg::DynamicSceneGraph::Ptr graph_;
  std::unique_ptr<SceneGraphVisual> visual_;
};

}  // namespace hydra
