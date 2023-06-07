#pragma once
#include <hydra_msgs/DsgUpdate.h>
#include <rviz/message_filter_display.h>
#include <spark_dsg/dynamic_scene_graph.h>

#include <map>
#include <memory>

#include "hydra_rviz_plugin/common.h"
#include "hydra_rviz_plugin/layer_config.h"

namespace rviz {
class BoolProperty;
class FloatProperty;
}  // namespace rviz

namespace hydra {

class LayerVisual;
class BoundingBoxVisual;
struct Pose;

struct LayerContainer {
  LayerContainer() = default;

  std::unique_ptr<LayerVisual> visual;
  std::unique_ptr<BoundingBoxVisual> bbox_visual;
  LayerConfig config;
  ColorFunctor::Ptr node_color_callback;
  Pose pose;
};

class SceneGraphDisplay : public rviz::MessageFilterDisplay<hydra_msgs::DsgUpdate> {
  Q_OBJECT
 public:
  SceneGraphDisplay();

  virtual ~SceneGraphDisplay();

 protected:
  virtual void onInitialize();

  virtual void reset();

 private Q_SLOTS:
  void updateProperties();

 private:
  void processMessage(const hydra_msgs::DsgUpdate::ConstPtr& msg);

  void initLayers();

  void assignColorFunctions();

  void readDefaults();

  void setLayerPoses();

  spark_dsg::DynamicSceneGraph::Ptr graph_;
  std::map<spark_dsg::LayerId, LayerConfig> default_configs_;
  std::map<spark_dsg::LayerId, LayerContainer> layers_;

  std::unique_ptr<rviz::BoolProperty> collapse_layers_;
  std::unique_ptr<rviz::FloatProperty> layer_height_;

  std::unique_ptr<Pose> last_pose_;
};

}  // namespace hydra
