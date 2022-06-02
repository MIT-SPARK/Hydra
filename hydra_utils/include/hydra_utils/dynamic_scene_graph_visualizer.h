#pragma once
#include "hydra_utils/dsg_visualizer_plugin.h"
#include "hydra_utils/visualizer_types.h"

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <vector>

#include <pcl/PolygonMesh.h>

namespace hydra {

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

template <typename Config>
class ConfigManager {
 public:
  using Ptr = std::shared_ptr<ConfigManager<Config>>;
  using Server = dynamic_reconfigure::Server<Config>;

  ConfigManager(const ros::NodeHandle& nh, const std::string& ns)
      : nh_(nh, ns), changed_(true) {
    server_ = std::make_unique<Server>(nh_);
    server_->setCallback(boost::bind(&ConfigManager<Config>::update, this, _1, _2));
  }

  bool hasChange() { return changed_; }

  void clearChangeFlag() { changed_ = false; }

  const Config& get() const { return config_; };

 private:
  void update(Config& config, uint32_t) {
    config_ = config;
    changed_ = true;
  }

  ros::NodeHandle nh_;

  bool changed_;
  Config config_;

  std::unique_ptr<Server> server_;
};

void clearPrevMarkers(const std_msgs::Header& header,
                      const std::set<NodeId>& curr_nodes,
                      const std::string& ns,
                      std::set<NodeId>& prev_nodes,
                      MarkerArray& msg);

class DynamicSceneGraphVisualizer {
 public:
  using DynamicLayerConfigManager = ConfigManager<DynamicLayerConfig>;
  using VisualizerConfigManager = ConfigManager<VisualizerConfig>;
  using LayerConfigManager = ConfigManager<LayerConfig>;
  using ColormapConfigManager = ConfigManager<ColormapConfig>;

  DynamicSceneGraphVisualizer(const ros::NodeHandle& nh,
                              const DynamicSceneGraph::LayerIds& layer_ids);

  virtual ~DynamicSceneGraphVisualizer() = default;

  void addPlugin(const std::shared_ptr<DsgVisualizerPlugin>& plugin) {
    plugins_.push_back(plugin);
  }

  void start(bool periodic_redraw = false);

  bool redraw();

  inline void setGraphUpdated() { need_redraw_ = true; }

  void setGraph(const DynamicSceneGraph::Ptr& scene_graph, bool need_reset = true);

  void reset();

 protected:
  virtual void resetImpl(const std_msgs::Header& header, MarkerArray& msg);

  virtual void redrawImpl(const std_msgs::Header& header, MarkerArray& msg);

  virtual bool hasConfigChanged() const;

  virtual void clearConfigChangeFlags();

  virtual void drawLayer(const std_msgs::Header& header,
                         const SceneGraphLayer& layer,
                         const LayerConfig& config,
                         MarkerArray& msg);

  virtual void drawLayerMeshEdges(const std_msgs::Header& header,
                                  LayerId layer_id,
                                  const std::string& ns,
                                  MarkerArray& msg);

 protected:
  void deleteMultiMarker(const std_msgs::Header& header,
                         const std::string& ns,
                         MarkerArray& msg);

  void addMultiMarkerIfValid(const Marker& marker, MarkerArray& msg);

  void setupConfigs(const DynamicSceneGraph::LayerIds& layer_ids);

  void displayLoop(const ros::WallTimerEvent&);

  void deleteLayer(const std_msgs::Header& header,
                   const SceneGraphLayer& layer,
                   MarkerArray& msg);

  inline std::string getDynamicNodeNamespace(char layer_prefix) const {
    return dynamic_node_ns_prefix_ + layer_prefix;
  }

  inline std::string getDynamicEdgeNamespace(char layer_prefix) const {
    return dynamic_edge_ns_prefix_ + layer_prefix;
  }

  inline std::string getDynamicLabelNamespace(char layer_prefix) const {
    return dynamic_label_ns_prefix_ + layer_prefix;
  }

  inline std::string getLayerNodeNamespace(LayerId layer) const {
    return node_ns_prefix_ + std::to_string(layer);
  }

  inline std::string getLayerEdgeNamespace(LayerId layer) const {
    return edge_ns_prefix_ + std::to_string(layer);
  }

  inline std::string getLayerLabelNamespace(LayerId layer) const {
    return label_ns_prefix_ + std::to_string(layer);
  }

  inline std::string getLayerBboxNamespace(LayerId layer) const {
    return bbox_ns_prefix_ + std::to_string(layer);
  }

 private:
  const DynamicLayerConfig& getConfig(LayerId layer);

  void drawDynamicLayer(const std_msgs::Header& header,
                        const DynamicSceneGraphLayer& layer,
                        const DynamicLayerConfig& config,
                        const VisualizerConfig& viz_config,
                        MarkerArray& msg);

  void deleteLabel(const std_msgs::Header& header, char prefix, MarkerArray& msg);

  void deleteDynamicLayer(const std_msgs::Header& header,
                          char prefix,
                          MarkerArray& msg);

  void drawDynamicLayers(const std_msgs::Header& header, MarkerArray& msg);

 protected:
  DynamicSceneGraph::Ptr scene_graph_;

  ros::NodeHandle nh_;

  bool need_redraw_;
  bool periodic_redraw_;
  std::string world_frame_;
  std::string visualizer_ns_;
  std::string visualizer_layer_ns_;

  ros::WallTimer visualizer_loop_timer_;

  const std::string node_ns_prefix_ = "layer_nodes_";
  const std::string edge_ns_prefix_ = "layer_edges_";
  const std::string label_ns_prefix_ = "layer_labels_";
  const std::string bbox_ns_prefix_ = "layer_bounding_boxes_";
  const std::string mesh_edge_ns_ = "mesh_object_connections";
  const std::string interlayer_edge_ns_prefix_ = "interlayer_edges_";
  const LayerId mesh_edge_source_layer_ = KimeraDsgLayers::OBJECTS;
  const std::string dynamic_node_ns_prefix_ = "dynamic_nodes_";
  const std::string dynamic_edge_ns_prefix_ = "dynamic_edges_";
  const std::string dynamic_label_ns_prefix_ = "dynamic_label_";

  std::set<std::string> published_multimarkers_;
  std::map<LayerId, std::set<NodeId>> prev_labels_;
  std::map<LayerId, std::set<NodeId>> curr_labels_;
  std::map<LayerId, std::set<NodeId>> prev_bboxes_;
  std::map<LayerId, std::set<NodeId>> curr_bboxes_;

  std::map<LayerId, LayerConfigManager::Ptr> layer_configs_;
  VisualizerConfigManager::Ptr visualizer_config_;
  ColormapConfigManager::Ptr places_colormap_;

  ros::Publisher dsg_pub_;

  std::map<LayerId, DynamicLayerConfigManager::Ptr> dynamic_configs_;

  std::list<std::shared_ptr<DsgVisualizerPlugin>> plugins_;

  std::set<std::string> published_dynamic_labels_;

  ros::Publisher dynamic_layers_viz_pub_;
};

}  // namespace hydra
