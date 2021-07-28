#pragma once
#include "kimera_topology/config_helpers.h"
#include "kimera_topology/gvd_integrator.h"
#include "kimera_topology/gvd_visualization_utilities.h"

#include <dynamic_reconfigure/server.h>
#include <kimera_dsg_visualizer/visualizer_types.h>
#include <kimera_dsg_visualizer/visualizer_utils.h>
#include <kimera_topology/GvdVisualizerConfig.h>
#include <voxblox_ros/conversions.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox_ros/ros_params.h>

#include <glog/logging.h>
#include <ros/ros.h>

namespace kimera {
namespace topology {

using kimera_topology::GvdVisualizerConfig;

enum class VisualizationType : int {
  NONE = 0,
  ESDF_WITH_SLICE = 1,
  GVD = 2,
};

template <typename TsdfServerType>
class TopologyServer {
 public:
  using RqtServer = dynamic_reconfigure::Server<GvdVisualizerConfig>;
  using GraphRqtServer = dynamic_reconfigure::Server<LayerConfig>;
  using RqtMutexPtr = std::unique_ptr<boost::recursive_mutex>;

  explicit TopologyServer(const ros::NodeHandle& nh) : nh_(nh) {
    setupConfig();

    tsdf_server_.reset(new TsdfServerType(ros::NodeHandle(), nh_));

    tsdf_layer_ = tsdf_server_->getTsdfMapPtr()->getTsdfLayerPtr();
    CHECK_NOTNULL(tsdf_layer_);

    gvd_layer_.reset(
        new Layer<GvdVoxel>(tsdf_layer_->voxel_size(), tsdf_layer_->voxels_per_side()));
    mesh_layer_.reset(new MeshLayer(tsdf_layer_->block_size()));

    gvd_integrator_.reset(
        new GvdIntegrator(config_, tsdf_layer_, gvd_layer_, mesh_layer_));

    mesh_pub_ = nh_.advertise<voxblox_msgs::Mesh>("mesh", 1, true);

    gvd_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("gvd_viz", 1, true);
    gvd_edge_viz_pub_ =
        nh_.advertise<visualization_msgs::Marker>("gvd_edge_viz", 1, true);
    graph_viz_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>("graph_viz", 1, true);
    label_viz_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>("graph_label_viz", 1, true);

    setupGraphVisualizer();

    double update_period_s = 1.0;
    nh_.getParam("update_period_s", update_period_s);
    update_timer_ = nh_.createTimer(ros::Duration(update_period_s),
                                    [&](const ros::TimerEvent&) { runUpdate(); });
  }

  void spin() const { ros::spin(); }

 private:
  void setupGraphVisualizer() {
    graph_visualizer_config_.layer_z_step = 0;
    graph_visualizer_config_.color_places_by_distance = true;
    graph_config_ = getLayerConfig("~/graph_visualizer");

    graph_mutex_ = std::make_unique<boost::recursive_mutex>();
    graph_config_server_ = std::make_unique<GraphRqtServer>(
        *graph_mutex_, ros::NodeHandle("~/graph_visualizer"));
    {  // critical region for dynamic reconfigure
      boost::recursive_mutex::scoped_lock lock(*graph_mutex_);
      graph_config_server_->updateConfig(graph_config_);
    }

    graph_config_server_->setCallback(
        boost::bind(&TopologyServer::graphConfigCb, this, _1, _2));
  }

  void graphConfigCb(LayerConfig& config, uint32_t) { graph_config_ = config; }

  void parseBasisParam(const std::string& name, uint8_t& actual_value) {
    int value = actual_value;
    nh_.getParam(name, value);
    std::clamp(value, 0, 26);
    actual_value = static_cast<uint8_t>(value);
  }

  template <typename T>
  void parseLongParam(const std::string& name, T& actual_value) {
    int value = actual_value;
    nh_.getParam(name, value);
    actual_value = static_cast<T>(value);
  }

  void setupConfig() {
    std::string color_mode = "lambert_color";
    nh_.getParam("mesh_color_mode", color_mode);
    mesh_color_mode_ = voxblox::getColorModeFromString(color_mode);

    nh_.param<std::string>("world_frame", world_frame_, "world");

    // this intentionally disables marching cubes in the native voxblox server
    nh_.setParam("update_mesh_every_n_sec", 0.0);

    // TODO(nathan) move and lift to new function
    nh_.param<float>("max_distance_m", config_.max_distance_m, 2.0);
    nh_.param<float>("min_distance_m", config_.min_distance_m, 0.2);
    nh_.param<float>("min_diff_m", config_.min_diff_m, 1.0e-3);
    nh_.param<float>("min_weight", config_.min_weight, 1.0e-6);
    nh_.param("voronoi_min_distance_m", config_.voronoi_config.min_distance_m, 0.5);
    nh_.param("voronoi_neighbor_l1_separation",
              config_.voronoi_config.parent_l1_separation,
              5.0);
    nh_.param("voronoi_neighbor_cos_separation",
              config_.voronoi_config.parent_cos_angle_separation,
              5.0);
    nh_.param("num_buckets", config_.num_buckets, 20);
    nh_.param("multi_queue", config_.multi_queue, false);
    nh_.param("parent_derived_distance", config_.parent_derived_distance, true);

    parseBasisParam("min_basis_for_extraction", config_.min_basis_for_extraction);
    // TODO(nathan) this might be better as 1
    parseBasisParam("graph_extraction_min_extra_basis",
                    config_.graph_extractor_config.min_extra_basis);
    parseBasisParam("graph_extraction_min_vertex_basis",
                    config_.graph_extractor_config.min_vertex_basis);
    parseLongParam("graph_extraction_max_edge_deviation",
                   config_.graph_extractor_config.max_edge_deviation);
    parseLongParam("graph_extraction_max_edge_split_iterations",
                   config_.graph_extractor_config.max_edge_split_iterations);
    nh_.getParam("graph_extraction_add_cleared_indices",
                 config_.graph_extractor_config.add_cleared_indices_to_wavefront);

    bool use_angle = false;
    nh_.getParam("voronoi_use_angle", use_angle);
    if (use_angle) {
      config_.voronoi_config.mode = ParentUniquenessMode::L1_THEN_ANGLE;
    }

    config_server_.reset(new RqtServer(ros::NodeHandle("~/gvd_visualizer")));
    config_server_->setCallback(
        boost::bind(&TopologyServer::configUpdateCb, this, _1, _2));
  }

  void assignGraphPlacesColormap() {
    // TODO(nathan) consider separating colors into own rqt server
    graph_visualizer_config_.places_min_distance = gvd_viz_config_.min_distance;
    graph_visualizer_config_.places_max_distance = gvd_viz_config_.max_distance;
    graph_visualizer_config_.places_min_hue = viz_color_config_.min_hue;
    graph_visualizer_config_.places_max_hue = viz_color_config_.max_hue;
    graph_visualizer_config_.places_min_luminance = viz_color_config_.min_luminance;
    graph_visualizer_config_.places_max_luminance = viz_color_config_.max_luminance;
    graph_visualizer_config_.places_min_saturation = viz_color_config_.min_saturation;
    graph_visualizer_config_.places_max_saturation = viz_color_config_.max_saturation;
  }

  void configUpdateCb(GvdVisualizerConfig& config, uint32_t) {
    visualization_type_ = static_cast<VisualizationType>(config.visualization_type);
    gvd_viz_config_.mode = static_cast<GvdVisualizationMode>(config.gvd_mode);
    gvd_viz_config_.min_distance = config.gvd_min_distance;
    gvd_viz_config_.max_distance = config.gvd_max_distance;
    gvd_viz_config_.basis_threshold = config.basis_threshold;
    gvd_viz_config_.min_num_basis = config.min_num_basis;
    gvd_viz_config_.max_num_basis = config.max_num_basis;
    gvd_viz_config_.alpha = config.gvd_alpha;
    viz_color_config_.min_hue = config.min_hue;
    viz_color_config_.max_hue = config.max_hue;
    viz_color_config_.min_saturation = config.min_saturation;
    viz_color_config_.max_saturation = config.max_saturation;
    viz_color_config_.min_luminance = config.min_luminance;
    viz_color_config_.max_luminance = config.max_luminance;
    esdf_viz_config_.alpha = config.esdf_alpha;
    esdf_viz_config_.slice_height = config.slice_height;
    esdf_viz_config_.min_distance = config.esdf_min_distance;
    esdf_viz_config_.max_distance = config.esdf_max_distance;
    assignGraphPlacesColormap();
  }

  void visualizeGvd() {
    visualization_msgs::Marker marker;

    switch (visualization_type_) {
      case VisualizationType::ESDF_WITH_SLICE:
        marker = makeEsdfMarker(*gvd_layer_, viz_color_config_, esdf_viz_config_);
        break;
      case VisualizationType::GVD:
        marker = makeGvdMarker(*gvd_layer_, viz_color_config_, gvd_viz_config_);
        break;
      case VisualizationType::NONE:
      default:
        return;
    }

    if (!marker.points.size()) {
      return;
    }

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "gvd_visualizer";
    gvd_viz_pub_.publish(marker);
  }

  void visualizeGraph() {
    if (gvd_integrator_->getGraph().nodes().empty()) {
      return;
    }

    ros::Time draw_time = ros::Time::now();
    visualization_msgs::MarkerArray markers;

    visualization_msgs::Marker node_marker = makeCentroidMarkers(
        graph_config_, gvd_integrator_->getGraph(), graph_visualizer_config_);
    node_marker.header.stamp = draw_time;
    node_marker.header.frame_id = "world";
    node_marker.ns = "topology_graph_nodes";
    markers.markers.push_back(node_marker);

    if (!gvd_integrator_->getGraph().edges().empty()) {
      visualization_msgs::Marker edge_marker =
          makeLayerEdgeMarkers(graph_config_,
                               gvd_integrator_->getGraph(),
                               graph_visualizer_config_,
                               NodeColor::Zero());
      edge_marker.header.stamp =
          draw_time + ros::Duration(1.0e-2);  // potentially force draw order
      edge_marker.header.frame_id = "world";
      edge_marker.ns = "topology_graph_edges";
      markers.markers.push_back(edge_marker);
    }

    publishLabels();
    graph_viz_pub_.publish(markers);
  }

  void publishLabels() {
    if (!graph_config_.use_label) {
      return;
    }

    ros::Time draw_time = ros::Time::now();

    visualization_msgs::MarkerArray labels;
    for (const auto& id_node_pair : gvd_integrator_->getGraph().nodes()) {
      const SceneGraphNode& node = *id_node_pair.second;
      visualization_msgs::Marker label = makeTextMarker(
          graph_config_, node, graph_visualizer_config_, "topology_labels");
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

    visualization_msgs::MarkerArray delete_markers;
    for (auto to_delete : ids_to_delete) {
      visualization_msgs::Marker delete_label;
      delete_label.action = visualization_msgs::Marker::DELETE;
      delete_label.id = to_delete;
      delete_label.ns = "topology_labels";
      delete_markers.markers.push_back(delete_label);
    }

    label_viz_pub_.publish(delete_markers);
    label_viz_pub_.publish(labels);
  }

  void visualizeGvdEdges() {
    visualization_msgs::Marker marker =
        makeGvdEdgeMarker(*gvd_layer_,
                          gvd_integrator_->getGraphExtractor().getGvdEdgeInfo(),
                          gvd_integrator_->getGraphExtractor().getNodeIndexMap());
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    gvd_edge_viz_pub_.publish(marker);
  }

  void runUpdate() {
    if (!tsdf_layer_ || tsdf_layer_->getNumberOfAllocatedBlocks() == 0) {
      return;
    }

    gvd_integrator_->updateFromTsdfLayer(true);

    voxblox_msgs::Mesh mesh_msg;
    generateVoxbloxMeshMsg(mesh_layer_, mesh_color_mode_, &mesh_msg);
    mesh_msg.header.frame_id = world_frame_;
    mesh_pub_.publish(mesh_msg);

    visualizeGvd();
    visualizeGraph();
    visualizeGvdEdges();

    ROS_INFO_STREAM("Timings: " << std::endl << voxblox::timing::Timing::Print());
    // TODO(nathan) consider showing used memory
    // TODO(nathan) consider optionally disabling timing report
  }

  ros::Publisher gvd_viz_pub_;
  ros::Publisher graph_viz_pub_;
  ros::Publisher label_viz_pub_;
  ros::Publisher gvd_edge_viz_pub_;
  ros::Publisher mesh_pub_;

  std::set<int> previous_labels_;

  VisualizationType visualization_type_;
  dsg_utils::HlsColorMapConfig viz_color_config_;
  GvdVisualizationConfig gvd_viz_config_;
  EsdfVisualizationConfig esdf_viz_config_;
  std::unique_ptr<RqtServer> config_server_;

  RqtMutexPtr graph_mutex_;
  std::unique_ptr<GraphRqtServer> graph_config_server_;
  LayerConfig graph_config_;
  VisualizerConfig graph_visualizer_config_;

  voxblox::ColorMode mesh_color_mode_;
  std::string world_frame_;

  Layer<TsdfVoxel>* tsdf_layer_;
  Layer<GvdVoxel>::Ptr gvd_layer_;
  MeshLayer::Ptr mesh_layer_;

  GvdIntegratorConfig config_;

  std::unique_ptr<TsdfServerType> tsdf_server_;
  std::unique_ptr<GvdIntegrator> gvd_integrator_;

  ros::NodeHandle nh_;
  ros::Timer update_timer_;
};

}  // namespace topology
}  // namespace kimera
