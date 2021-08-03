#include "kimera_topology/topology_server_visualizer.h"
#include "kimera_topology/config_parser.h"

namespace kimera {
namespace topology {

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

TopologyServerVisualizer::TopologyServerVisualizer(const std::string& ns) : nh_(ns) {
  gvd_viz_pub_ = nh_.advertise<Marker>("gvd_viz", 1, true);
  gvd_edge_viz_pub_ = nh_.advertise<Marker>("gvd_edge_viz", 1, true);
  graph_viz_pub_ = nh_.advertise<MarkerArray>("graph_viz", 1, true);
  label_viz_pub_ = nh_.advertise<MarkerArray>("graph_label_viz", 1, true);
  block_viz_pub_ = nh_.advertise<Marker>("voxel_block_viz", 1, true);

  config_.graph.layer_z_step = 0;
  config_.graph.color_places_by_distance = true;

  config_.graph_layer = getLayerConfig("~/graph_visualizer");
  config_.colormap = getColormapConfig("~/visualizer_colormap");
  config_.gvd = getGvdVisualizerConfig("~/gvd_visualizer");

  fillTopologyServerVizConfig(nh_, config_);

  setupConfigServers();
}

void TopologyServerVisualizer::visualize(const GraphExtractor& extractor,
                                         const SceneGraphLayer& graph,
                                         const Layer<GvdVoxel>& gvd,
                                         const Layer<TsdfVoxel>& tsdf) {
  visualizeGraph(graph);
  visualizeGvd(gvd);
  visualizeGvdEdges(extractor, gvd);
  if (config_.show_block_outlines) {
    visualizeBlocks(gvd, tsdf);
  }
}

void TopologyServerVisualizer::visualizeGraph(const SceneGraphLayer& graph) {
  if (graph.nodes().empty()) {
    return;
  }

  ros::Time draw_time = ros::Time::now();
  MarkerArray markers;

  Marker node_marker =
      makeCentroidMarkers(config_.graph_layer, graph, config_.graph, config_.colormap);
  node_marker.header.stamp = draw_time;
  node_marker.header.frame_id = config_.world_frame;
  node_marker.ns = config_.topology_marker_ns + "_nodes";
  markers.markers.push_back(node_marker);

  if (!graph.edges().empty()) {
    Marker edge_marker = makeLayerEdgeMarkers(
        config_.graph_layer, graph, config_.graph, NodeColor::Zero());
    edge_marker.header.stamp =
        draw_time + ros::Duration(1.0e-2);  // potentially force draw order
    edge_marker.header.frame_id = config_.world_frame;
    edge_marker.ns = config_.topology_marker_ns + "_edges";
    markers.markers.push_back(edge_marker);
  }

  publishGraphLabels(graph);
  graph_viz_pub_.publish(markers);
}

void TopologyServerVisualizer::visualizeGvd(const Layer<GvdVoxel>& gvd) const {
  Marker msg;

  switch (static_cast<VisualizationType>(config_.gvd.visualization_type)) {
    case VisualizationType::ESDF_WITH_SLICE:
      msg = makeEsdfMarker(config_.gvd, config_.colormap, gvd);
      break;
    case VisualizationType::GVD:
      msg = makeGvdMarker(config_.gvd, config_.colormap, gvd);
      break;
    case VisualizationType::NONE:
    default:
      return;
  }

  if (!msg.points.size()) {
    return;
  }

  msg.header.frame_id = "world";
  msg.header.stamp = ros::Time::now();
  msg.ns = "gvd_visualizer";
  gvd_viz_pub_.publish(msg);
}

void TopologyServerVisualizer::visualizeBlocks(const Layer<GvdVoxel>& gvd,
                                               const Layer<TsdfVoxel>& tsdf) const {
  Marker msg;
  if (config_.use_gvd_block_outlines) {
    msg = makeBlocksMarker(gvd);
  } else {
    msg = makeBlocksMarker(tsdf);
  }

  msg.header.frame_id = "world";
  msg.header.stamp = ros::Time::now();
  msg.ns = "topology_server_blocks";
  block_viz_pub_.publish(msg);
}

void TopologyServerVisualizer::visualizeGvdEdges(const GraphExtractor& graph,
                                                 const Layer<GvdVoxel>& gvd) const {
  auto msg = makeGvdEdgeMarker(gvd, graph.getGvdEdgeInfo(), graph.getNodeRootMap());
  msg.header.frame_id = "world";
  msg.header.stamp = ros::Time::now();
  gvd_edge_viz_pub_.publish(msg);
}

void TopologyServerVisualizer::publishGraphLabels(const SceneGraphLayer& graph) {
  if (!config_.graph_layer.use_label) {
    return;
  }

  ros::Time draw_time = ros::Time::now();
  const std::string label_ns = config_.topology_marker_ns + "_labels";

  MarkerArray labels;
  for (const auto& id_node_pair : graph.nodes()) {
    const SceneGraphNode& node = *id_node_pair.second;
    Marker label = makeTextMarker(config_.graph_layer, node, config_.graph, label_ns);
    label.header.frame_id = "world";
    label.header.stamp = draw_time;
    labels.markers.push_back(label);
  }

  std::set<int> current_ids;
  for (const auto& label : labels.markers) {
    current_ids.insert(label.id);
  }

  std::set<int> ids_to_delete;
  for (auto previous : previous_labels_) {
    if (!current_ids.count(previous)) {
      ids_to_delete.insert(previous);
    }
  }
  previous_labels_ = current_ids;

  MarkerArray delete_markers;
  for (auto to_delete : ids_to_delete) {
    Marker delete_label;
    delete_label.action = Marker::DELETE;
    delete_label.id = to_delete;
    delete_label.ns = label_ns;
    delete_markers.markers.push_back(delete_label);
  }

  label_viz_pub_.publish(delete_markers);
  label_viz_pub_.publish(labels);
}

void TopologyServerVisualizer::graphConfigCb(LayerConfig& config, uint32_t) {
  config_.graph_layer = config;
}

void TopologyServerVisualizer::colormapCb(ColormapConfig& config, uint32_t) {
  config_.colormap = config;
}

void TopologyServerVisualizer::gvdConfigCb(GvdVisualizerConfig& config, uint32_t) {
  config_.gvd = config;
  config_.graph.places_colormap_min_distance = config.gvd_min_distance;
  config_.graph.places_colormap_max_distance = config.gvd_max_distance;
}

void TopologyServerVisualizer::setupConfigServers() {
  startRqtServer("~/gvd_visualizer",
                 gvd_config_server_,
                 gvd_rqt_mutex_,
                 config_.gvd,
                 &TopologyServerVisualizer::gvdConfigCb);

  startRqtServer("~/graph_visualizer",
                 graph_config_server_,
                 graph_rqt_mutex_,
                 config_.graph_layer,
                 &TopologyServerVisualizer::graphConfigCb);

  startRqtServer("~/visualizer_colormap",
                 colormap_server_,
                 colormap_rqt_mutex_,
                 config_.colormap,
                 &TopologyServerVisualizer::colormapCb);
}

}  // namespace topology
}  // namespace kimera
