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
class InterlayerEdgeVisual;
struct EdgeConfig;

struct LayerContainer {
  LayerContainer() = default;

  std::unique_ptr<LayerVisual> visual;
  std::unique_ptr<BoundingBoxVisual> bbox_visual;
  LayerConfig config;
  ColorFunctor::Ptr node_color_callback;
  Pose pose;
  Pose offset;
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

  bool readGraph(const hydra_msgs::DsgUpdate::ConstPtr& msg);

  bool readPose(const hydra_msgs::DsgUpdate::ConstPtr& msg);

  void initLayers();

  void assignColorFunctions();

  void readDefaults();

  void setLayerPoses();

  void updateLayerVisuals();

  void updateDynamicLayerVisuals();

  void updateInterlayerEdgeVisual();

  std::unique_ptr<Pose> last_pose_;
  spark_dsg::DynamicSceneGraph::Ptr graph_;

  std::unique_ptr<rviz::BoolProperty> collapse_layers_;
  std::unique_ptr<rviz::FloatProperty> layer_height_;

  std::map<LayerId, LayerConfig> default_configs_;
  std::map<LayerId, LayerContainer> layers_;

  std::map<LayerId, DynamicLayerConfig> default_dynamic_configs_;
  std::map<LayerId, std::map<LayerId, std::unique_ptr<DynamicLayerVisual>>>
      dynamic_layers_;

  std::unique_ptr<InterlayerEdgeVisual> interlayer_edges_;
  std::map<LayerPair, EdgeConfig> default_edge_configs_;
  std::map<LayerPair, EdgeConfig> edge_configs_;
};

}  // namespace hydra
