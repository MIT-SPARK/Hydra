#pragma once
#include "hydra_dsg_builder/dsg_lcd_detector.h"

#include <kimera_dsg_visualizer/dynamic_scene_graph_visualizer.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace hydra {
namespace lcd {

using kimera::ConfigManager;
using kimera::DynamicLayerConfig;
using kimera::DynamicSceneGraphVisualizer;
using kimera::LayerConfig;
using kimera::NodeColor;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;


class LcdVisualizer : public DynamicSceneGraphVisualizer {
 public:
  using DynamicLayerConfigManager = ConfigManager<DynamicLayerConfig>;

  static DynamicSceneGraph::LayerIds getDefaultLayerIds() {
    return {KimeraDsgLayers::OBJECTS, KimeraDsgLayers::PLACES};
  }

  LcdVisualizer(const ros::NodeHandle& nh, double radius);

  void setLcdDetector(DsgLcdDetector* detector);

 protected:
  virtual void redrawImpl(const std_msgs::Header& header, MarkerArray& msg) override;

  virtual void drawLayer(const std_msgs::Header& header,
                         const SceneGraphLayer& layer,
                         const LayerConfig& config,
                         MarkerArray& msg) override;

  virtual void drawLayerMeshEdges(const std_msgs::Header& header,
                                  LayerId layer_id,
                                  const std::string& ns,
                                  MarkerArray& msg) override;

  void drawAgent(const std_msgs::Header& header, MarkerArray& msg);

  std::optional<NodeId> getQueryNode() const;

  std::vector<NodeId> getMatchRoots(LayerId layer) const;

  std::set<NodeId> getMatchedNodes(LayerId layer) const;

  std::set<NodeId> getValidNodes(LayerId layer) const;

 private:
  DsgLcdDetector* lcd_detector_;

  double radius_;

  DynamicLayerConfigManager::Ptr agent_config_;
  NodeColor invalid_match_color_;
  NodeColor valid_match_color_;
  NodeColor default_graph_color_;
  NodeColor query_color_;
};

}  // namespace lcd
}  // namespace hydra
