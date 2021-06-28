#include "kimera_scene_graph/scene_graph_visualizer.h"
#include "kimera_scene_graph/visualizer_utils.h"

#include <kimera_pgmo/utils/CommonFunctions.h>
#include <mesh_msgs/TriangleMeshStamped.h>

#include <glog/logging.h>

#include <tf2_eigen/tf2_eigen.h>

namespace kimera {

using kimera_scene_graph::VisualizerConfig;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
using Node = SceneGraph::Node;

namespace {

#define READ_PARAM(nh, c, name)                                   \
  if (!nh.param(#name, c.name, c.name)) {                         \
    VLOG(1) << "failed to find param " #name " under namespace "  \
            << nh.getNamespace() << ". defaulting to " << c.name; \
  }                                                               \
  static_assert(true, "")

VisualizerConfig getVisualizerConfig(const std::string& visualizer_namespace) {
  ros::NodeHandle nh(visualizer_namespace);
  VisualizerConfig config;
  READ_PARAM(nh, config, layer_z_step);
  READ_PARAM(nh, config, mesh_edge_break_ratio);
  READ_PARAM(nh, config, mesh_layer_offset);
  READ_PARAM(nh, config, collapse_layers);
  READ_PARAM(nh, config, color_places_by_distance);
  READ_PARAM(nh, config, places_min_distance);
  READ_PARAM(nh, config, places_max_distance);
  READ_PARAM(nh, config, places_min_hue);
  READ_PARAM(nh, config, places_min_saturation);
  READ_PARAM(nh, config, places_min_luminance);
  READ_PARAM(nh, config, places_max_hue);
  READ_PARAM(nh, config, places_max_saturation);
  READ_PARAM(nh, config, places_max_luminance);
  return config;
}

LayerConfig getLayerConfig(const std::string& layer_namespace) {
  ros::NodeHandle nh(layer_namespace);
  LayerConfig config;
  READ_PARAM(nh, config, visualize);
  READ_PARAM(nh, config, z_offset_scale);
  READ_PARAM(nh, config, marker_scale);
  READ_PARAM(nh, config, marker_alpha);
  READ_PARAM(nh, config, use_sphere_marker);
  READ_PARAM(nh, config, use_label);
  READ_PARAM(nh, config, label_height);
  READ_PARAM(nh, config, label_scale);
  READ_PARAM(nh, config, use_bounding_box);
  READ_PARAM(nh, config, bounding_box_alpha);
  READ_PARAM(nh, config, use_edge_source);
  READ_PARAM(nh, config, interlayer_edge_scale);
  READ_PARAM(nh, config, interlayer_edge_alpha);
  READ_PARAM(nh, config, interlayer_edge_use_color);
  READ_PARAM(nh, config, interlayer_edge_insertion_skip);
  READ_PARAM(nh, config, intralayer_edge_scale);
  READ_PARAM(nh, config, intralayer_edge_alpha);
  READ_PARAM(nh, config, intralayer_edge_use_color);
  READ_PARAM(nh, config, intralayer_edge_insertion_skip);
  return config;
}

#undef READ_PARAM

}  // namespace

SceneGraphVisualizer::SceneGraphVisualizer(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& nh_private,
    const SceneGraph::LayerIds& layer_ids)
    : nh_(nh),
      nh_private_(nh_private),
      need_redraw_(false),
      world_frame_("world"),
      visualizer_ns_("~/visualizer"),
      visualizer_layer_ns_("~/visualizer/layer") {
  nh_private_.param("world_frame", world_frame_, world_frame_);
  nh_private_.param("visualizer_ns", visualizer_ns_, visualizer_ns_);
  nh_private_.param(
      "visualizer_layer_ns", visualizer_layer_ns_, visualizer_layer_ns_);

  semantic_instance_centroid_pub_ =
      nh_private_.advertise<MarkerArray>("semantic_instance_centroid", 1, true);
  bounding_box_pub_ =
      nh_private_.advertise<MarkerArray>("bounding_boxes", 1, true);
  text_markers_pub_ =
      nh_private_.advertise<MarkerArray>("instance_ids", 1, true);
  edges_centroid_pcl_pub_ =
      nh_private_.advertise<MarkerArray>("edges_centroid_pcl", 1, true);
  edges_node_node_pub_ =
      nh_private_.advertise<MarkerArray>("edges_node_node", 1, true);
  wall_pub_ = nh_private_.advertise<mesh_msgs::TriangleMeshStamped>("wall_mesh", 1, true);
  semantic_mesh_pub_ = nh_private_.advertise<mesh_msgs::TriangleMeshStamped>(
      "semantic_mesh", 1, true);
  rgb_mesh_pub_ = nh_private_.advertise<mesh_msgs::TriangleMeshStamped>(
      "rgb_mesh", 1, true);

  setupDynamicReconfigure(layer_ids);

  double visualizer_loop_period = 1.0e-1;
  nh_private_.param(
      "visualizer_loop_period", visualizer_loop_period, visualizer_loop_period);
  visualizer_loop_timer_ =
      nh_.createTimer(ros::Duration(visualizer_loop_period),
                      &SceneGraphVisualizer::displayLoop,
                      this);
}

void SceneGraphVisualizer::setupDynamicReconfigure(
    const SceneGraph::LayerIds& layer_ids) {
  visualizer_config_ = getVisualizerConfig(visualizer_ns_);

  // required for "safely" updating the rqt server
  config_server_mutex_ = std::make_unique<boost::recursive_mutex>();
  config_server_ = std::make_unique<RqtServer>(*config_server_mutex_,
                                               ros::NodeHandle(visualizer_ns_));
  {  // critical region for dynamic reconfigure (probably unneeded)
    boost::recursive_mutex::scoped_lock lock(*config_server_mutex_);
    config_server_->updateConfig(visualizer_config_);
  }

  config_server_->setCallback(
      boost::bind(&SceneGraphVisualizer::configUpdateCb, this, _1, _2));

  for (const auto& layer : layer_ids) {
    const std::string layer_ns = visualizer_layer_ns_ + std::to_string(layer);
    layer_configs_[layer] = getLayerConfig(layer_ns);
    // required for "safely" updating the rqt server
    layer_config_server_mutexes_[layer] =
        std::make_unique<boost::recursive_mutex>();
    layer_config_servers_[layer] = std::make_unique<LayerRqtServer>(
        *layer_config_server_mutexes_.at(layer), ros::NodeHandle(layer_ns));

    {  // critical region for dynamic reconfigure (probably unneeded)
      boost::recursive_mutex::scoped_lock lock(
          *layer_config_server_mutexes_.at(layer));
      layer_config_servers_.at(layer)->updateConfig(layer_configs_.at(layer));
    }

    layer_config_cb_[layer] = boost::bind(
        &SceneGraphVisualizer::layerConfigUpdateCb, this, layer, _1, _2);
    layer_config_servers_.at(layer)->setCallback(layer_config_cb_[layer]);
  }
}

void SceneGraphVisualizer::configUpdateCb(VisualizerConfig& config, uint32_t) {
  visualizer_config_ = config;
  need_redraw_ = true;
}

void SceneGraphVisualizer::layerConfigUpdateCb(LayerId layer_id,
                                               LayerConfig& config,
                                               uint32_t) {
  layer_configs_[layer_id] = config;
  need_redraw_ = true;
}

void SceneGraphVisualizer::visualize(
    const DynamicSceneGraph::Ptr& scene_graph) {
  if (scene_graph == nullptr || scene_graph->empty()) {
    ROS_WARN("Request to visualize empty scene graph, skipping.");
    return;
  }
  // TODO(nathan) a mutex here might not be a bad idea
  scene_graph_ = scene_graph;
  need_redraw_ = true;
}

void SceneGraphVisualizer::displayLoop(const ros::TimerEvent&) {
  if (!scene_graph_) {
    return;
  }

  if (!need_redraw_) {
    return;
  }

  need_redraw_ = false;
  if (scene_graph_->hasLayer(scene_graph_->mesh_layer_id)) {
    visualizeMesh(*(scene_graph_->getMesh()), false);
  }
  displayLayers(*scene_graph_);
  displayEdges(*scene_graph_);
}

void SceneGraphVisualizer::fillHeader(Marker& marker,
                                      const ros::Time& current_time) const {
  marker.header.stamp = current_time;
  marker.header.frame_id = world_frame_;
}

Marker makeDeleteMarker(LayerId layer_id, const std::string& marker_ns) {
  Marker marker;
  marker.action = Marker::DELETE;
  marker.id = layer_id;
  marker.ns = marker_ns;
  return marker;
}

void SceneGraphVisualizer::handleCentroids(const SceneGraphLayer& layer,
                                           const LayerConfig& config,
                                           const ros::Time& current_time,
                                           MarkerArray& markers) const {
  const std::string ns = "layer_centroids";
  if (config.visualize) {
    Marker marker = makeCentroidMarkers(
        config, layer, visualizer_config_, std::nullopt, ns);
    fillHeader(marker, current_time);
    markers.markers.push_back(marker);
  } else {
    Marker delete_marker = makeDeleteMarker(layer.id, ns);
    fillHeader(delete_marker, current_time);
    markers.markers.push_back(delete_marker);
  }
}

void SceneGraphVisualizer::handleMeshEdges(const SceneGraphLayer& layer,
                                           const LayerConfig& config,
                                           const ros::Time& current_time,
                                           MarkerArray& markers) const {
  if (layer.id != to_underlying(KimeraDsgLayers::OBJECTS)) {
    return;
  }

  const std::string ns = "mesh_layer_edges";
  if (config.visualize) {
    Marker marker = makeMeshEdgesMarker(
        config, visualizer_config_, *scene_graph_, layer, ns);
    fillHeader(marker, current_time);
    markers.markers.push_back(marker);
  } else {
    Marker delete_marker = makeDeleteMarker(layer.id, ns);
    fillHeader(delete_marker, current_time);
    markers.markers.push_back(delete_marker);
  }
}

void SceneGraphVisualizer::handleLabels(const SceneGraphLayer& layer,
                                        const LayerConfig& config,
                                        const ros::Time& current_time,
                                        MarkerArray& markers) const {
  const std::string ns = "layer_" + std::to_string(layer.id) + "_text";

  for (const auto& id_node_pair : layer.nodes) {
    const Node& node = *id_node_pair.second;

    if (config.visualize && config.use_label) {
      Marker marker = makeTextMarker(config, node, visualizer_config_, ns);
      fillHeader(marker, current_time);
      markers.markers.push_back(marker);
    } else {
      Marker delete_marker = makeDeleteMarker(node.id, ns);
      fillHeader(delete_marker, current_time);
      markers.markers.push_back(delete_marker);
    }
  }
}

void SceneGraphVisualizer::handleBoundingBoxes(const SceneGraphLayer& layer,
                                               const LayerConfig& config,
                                               const ros::Time& current_time,
                                               MarkerArray& markers) const {
  const std::string ns =
      "layer_" + std::to_string(layer.id) + "_bounding_boxes";

  for (const auto& id_node_pair : layer.nodes) {
    const Node& node = *id_node_pair.second;

    if (config.visualize && config.use_bounding_box) {
      try {
        node.attributes<ObjectNodeAttributes>();
      } catch (const std::bad_cast&) {
        ROS_ERROR("Bounding boxes enabled for non-object layer");
        return;
      }

      Marker marker =
          makeBoundingBoxMarker(config, node, visualizer_config_, ns);
      fillHeader(marker, current_time);
      markers.markers.push_back(marker);
    } else {
      Marker delete_marker = makeDeleteMarker(node.id, ns);
      fillHeader(delete_marker, current_time);
      markers.markers.push_back(delete_marker);
    }
  }
}

void SceneGraphVisualizer::displayLayers(const SceneGraph& scene_graph) const {
  MarkerArray layer_centroids;
  MarkerArray text_markers;
  MarkerArray line_assoc_markers;
  MarkerArray bounding_boxes;

  // TODO(nathan) book-keeping for mutating scene graph
  ros::Time current_time = ros::Time::now();
  for (const auto& id_layer_pair : scene_graph.layers) {
    if (!layer_configs_.count(id_layer_pair.first)) {
      LOG(WARNING) << "failed to find config for layer " << id_layer_pair.first;
      continue;
    }

    LayerConfig config = layer_configs_.at(id_layer_pair.first);
    const SceneGraphLayer& layer = *(id_layer_pair.second);

    handleCentroids(layer, config, current_time, layer_centroids);
    handleLabels(layer, config, current_time, text_markers);
    handleBoundingBoxes(layer, config, current_time, text_markers);
    handleMeshEdges(layer, config, current_time, line_assoc_markers);
  }

  if (!layer_centroids.markers.empty()) {
    semantic_instance_centroid_pub_.publish(layer_centroids);
  }
  if (!bounding_boxes.markers.empty()) {
    bounding_box_pub_.publish(bounding_boxes);
  }
  if (!text_markers.markers.empty()) {
    text_markers_pub_.publish(text_markers);
  }
  if (!line_assoc_markers.markers.empty()) {
    edges_centroid_pcl_pub_.publish(line_assoc_markers);
  }
}

MarkerArray getDeleteAllMarker() {
  Marker delete_marker;
  delete_marker.action = Marker::DELETEALL;

  MarkerArray markers;
  markers.markers.push_back(delete_marker);
  return markers;
}

void SceneGraphVisualizer::clear() {
  // TODO(nathan) this causes an exception publishing
  scene_graph_.reset();
  ros::Time curr_time = ros::Time::now();

  {  // scope limiting marker
    MarkerArray marker = getDeleteAllMarker();
    fillHeader(marker.markers.front(), curr_time);
    semantic_instance_centroid_pub_.publish(marker);
  }
  {  // scope limiting marker
    MarkerArray marker = getDeleteAllMarker();
    fillHeader(marker.markers.front(), curr_time);
    bounding_box_pub_.publish(marker);
  }
  {  // scope limiting marker
    MarkerArray marker = getDeleteAllMarker();
    fillHeader(marker.markers.front(), curr_time);
    edges_centroid_pcl_pub_.publish(marker);
  }
  {  // scope limiting marker
    MarkerArray marker = getDeleteAllMarker();
    fillHeader(marker.markers.front(), curr_time);
    edges_node_node_pub_.publish(marker);
  }
  {  // scope limiting marker
    MarkerArray marker = getDeleteAllMarker();
    fillHeader(marker.markers.front(), curr_time);
    text_markers_pub_.publish(marker);
  }
  {  // scope limiting marker
    MarkerArray marker = getDeleteAllMarker();
    fillHeader(marker.markers.front(), curr_time);
    wall_pub_.publish(marker);
  }

  ros::spinOnce();
}

void SceneGraphVisualizer::displayEdges(const SceneGraph& scene_graph) const {
  ros::Time current_time = ros::Time::now();

  MarkerArray edge_markers =
      makeGraphEdgeMarkers(scene_graph, layer_configs_, visualizer_config_);
  for (auto& marker : edge_markers.markers) {
    fillHeader(marker, current_time);
  }

  for (const auto& id_layer_pair : scene_graph.layers) {
    if (id_layer_pair.second->numEdges() == 0) {
      continue;  // skip empty layer to avoid rviz errors
    }

    if (!layer_configs_.count(id_layer_pair.first)) {
      LOG(WARNING) << "Failed to find config for layer " << id_layer_pair.first;
      continue;
    }

    LayerConfig config = layer_configs_.at(id_layer_pair.first);
    Marker layer_edge_marker = makeLayerEdgeMarkers(
        config, *(id_layer_pair.second), visualizer_config_, NodeColor::Zero());
    fillHeader(layer_edge_marker, current_time);
    edge_markers.markers.push_back(layer_edge_marker);
  }

  edges_node_node_pub_.publish(edge_markers);
}

void SceneGraphVisualizer::visualizeMesh(const pcl::PolygonMesh& mesh,
                                         bool rgb_mesh) const {
  mesh_msgs::TriangleMeshStamped msg;
  msg.header.frame_id = world_frame_;
  msg.header.stamp = ros::Time::now();  // TODO(nathan) unify time
  msg.mesh = kimera_pgmo::PolygonMeshToTriangleMeshMsg(mesh);

  if (rgb_mesh) {
    rgb_mesh_pub_.publish(msg);
  } else {
    semantic_mesh_pub_.publish(msg);
  }
}

void SceneGraphVisualizer::visualizeWalls(const pcl::PolygonMesh& mesh) const {
  if (mesh.polygons.empty()) {
    return;  // better to not publish an empty marker
  }

  if (!layer_configs_.count(to_underlying(KimeraDsgLayers::PLACES))) {
    LOG(WARNING) << "Failed to find config for places layer";
    return;
  }

  mesh_msgs::TriangleMeshStamped msg;
  msg.header.frame_id = world_frame_;
  msg.header.stamp = ros::Time::now();  // TODO(nathan) unify time
  msg.mesh = kimera_pgmo::PolygonMeshToTriangleMeshMsg(mesh);

  LayerConfig config =
      layer_configs_.at(to_underlying(KimeraDsgLayers::PLACES));
  adjustMesh(config, visualizer_config_, msg.mesh);

  wall_pub_.publish(msg);
}

}  // namespace kimera
