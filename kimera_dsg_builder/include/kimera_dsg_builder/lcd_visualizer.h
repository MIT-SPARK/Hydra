#pragma once
#include "kimera_dsg_builder/dsg_lcd_module.h"

#include <kimera_dsg_visualizer/scene_graph_visualizer.h>

namespace kimera {
namespace lcd {

class LcdVisualizer : public SceneGraphVisualizer {
 public:
  using DynamicLayerConfigManager = ConfigManager<DynamicLayerConfig>;

  static SceneGraph::LayerIds getDefaultLayerIds() {
    return {KimeraDsgLayers::OBJECTS, KimeraDsgLayers::PLACES};
  }

  LcdVisualizer(const ros::NodeHandle& nh, double radius);

  void setLcdModule(DsgLcdModule* module);

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
  DsgLcdModule* lcd_module_;

  double radius_;

  DynamicLayerConfigManager::Ptr agent_config_;
  NodeColor invalid_match_color_;
  NodeColor valid_match_color_;
  NodeColor default_graph_color_;
  NodeColor query_color_;
};

}  // namespace lcd
}  // namespace kimera
