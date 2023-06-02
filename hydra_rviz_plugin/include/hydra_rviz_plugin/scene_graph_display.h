#pragma once
#include <hydra_msgs/DsgUpdate.h>
#include <rviz/message_filter_display.h>
#include <spark_dsg/dynamic_scene_graph.h>

#include <map>
#include <memory>

namespace hydra {

class LayerVisual;

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
  std::map<spark_dsg::LayerId, std::unique_ptr<LayerVisual>> layer_visuals_;
};

}  // namespace hydra
