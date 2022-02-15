#pragma once
#include "kimera_topology/configs.h"
#include "kimera_topology/gvd_visualization_utilities.h"

#include <hydra_utils/config.h>
#include <kimera_dsg_visualizer/visualizer_types.h>
#include <kimera_dsg_visualizer/visualizer_utils.h>

#include <dynamic_reconfigure/server.h>
#include <kimera_topology/GvdVisualizerConfig.h>

namespace kimera {
namespace topology {

using kimera_topology::GvdVisualizerConfig;
using RqtMutexPtr = std::unique_ptr<boost::recursive_mutex>;

struct TopologyVisualizerConfig {
  std::string world_frame = "world";
  std::string topology_marker_ns = "topology_graph";
  bool show_block_outlines = false;
  bool use_gvd_block_outlines = false;
  double outline_scale = 0.01;

  ColormapConfig colormap;
  GvdVisualizerConfig gvd;
  VisualizerConfig graph;
  LayerConfig graph_layer;
};

template <typename Visitor>
void visit_config(const Visitor& v, const TopologyVisualizerConfig& config) {
  config_parser::visit_config(v["world_frame"], config.world_frame);
  config_parser::visit_config(v["topology_marker_ns"], config.topology_marker_ns);
  config_parser::visit_config(v["show_block_outlines"], config.show_block_outlines);
  config_parser::visit_config(v["use_gvd_block_outlines"],
                              config.use_gvd_block_outlines);
  config_parser::visit_config(v["outline_scale"], config.outline_scale);
}

class TopologyServerVisualizer {
 public:
  explicit TopologyServerVisualizer(const std::string& ns);

  virtual ~TopologyServerVisualizer() = default;

  void visualize(const GraphExtractor& extractor,
                 const SceneGraphLayer& graph,
                 const Layer<GvdVoxel>& gvd,
                 const Layer<TsdfVoxel>& tsdf);

 private:
  void visualizeGraph(const SceneGraphLayer& graph);

  void visualizeGvd(const Layer<GvdVoxel>& gvd) const;

  void visualizeGvdEdges(const GraphExtractor& extractor,
                         const Layer<GvdVoxel>& gvd) const;

  void visualizeBlocks(const Layer<GvdVoxel>& gvd, const Layer<TsdfVoxel>& tsdf) const;

  void publishGraphLabels(const SceneGraphLayer& graph);

  void gvdConfigCb(GvdVisualizerConfig& config, uint32_t level);

  void graphConfigCb(LayerConfig& config, uint32_t level);

  void colormapCb(ColormapConfig& config, uint32_t level);

  void setupConfigServers();

  template <typename Config, typename Callback>
  void startRqtServer(const std::string& config_ns,
                      std::unique_ptr<dynamic_reconfigure::Server<Config>>& server,
                      const Callback& callback) {
    server.reset(new dynamic_reconfigure::Server<Config>(ros::NodeHandle(config_ns)));
    server->setCallback(boost::bind(callback, this, _1, _2));
  }

 private:
  ros::NodeHandle nh_;

  ros::Publisher gvd_viz_pub_;
  ros::Publisher graph_viz_pub_;
  ros::Publisher label_viz_pub_;
  ros::Publisher gvd_edge_viz_pub_;
  ros::Publisher block_viz_pub_;

  TopologyVisualizerConfig config_;
  std::set<int> previous_labels_;

  std::unique_ptr<dynamic_reconfigure::Server<GvdVisualizerConfig>> gvd_config_server_;
  std::unique_ptr<dynamic_reconfigure::Server<LayerConfig>> graph_config_server_;
  std::unique_ptr<dynamic_reconfigure::Server<ColormapConfig>> colormap_server_;
};

}  // namespace topology
}  // namespace kimera

DECLARE_CONFIG_OSTREAM_OPERATOR(kimera::topology, TopologyVisualizerConfig)
