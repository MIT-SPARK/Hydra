#include "kimera_scene_graph/scene_graph_visualizer.h"
#include "kimera_scene_graph/visualizer_utils.h"

#include <glog/logging.h>

#include <tf2_eigen/tf2_eigen.h>

namespace kimera {

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
using Node = SceneGraph::Node;

namespace {

// TODO(nathan) read in dynamically
LayerConfig getLayerConfig(LayerId layer_id, double layer_step) {
  LayerConfig config;
  switch (static_cast<KimeraDsgLayers>(layer_id)) {
    case KimeraDsgLayers::BUILDINGS:
      config.z_offset = 3.2 * layer_step;
      config.marker_scale = 1.0;
      config.marker_alpha = 1.0;
      config.use_sphere_marker = false;
      config.edge_alpha = 1.0;
      config.edge_scale = 0.1;
      config.use_label = true;
      config.text_height = 1.25;
      config.text_scale = 1.0;
      config.intralayer_edge_insertion_skip = 0;
      config.interlayer_edge_insertion_skip = 0;
      config.interlayer_edge_use_color = false;
      config.bounding_box_alpha = 0.5;
      config.use_edge_source = true;
      break;
    case KimeraDsgLayers::ROOMS:
      config.z_offset = 2.5 * layer_step;
      config.marker_scale = 0.6;
      config.marker_alpha = 0.8;
      config.edge_alpha = 0.8;
      config.use_sphere_marker = false;
      config.edge_scale = 0.1;
      config.use_label = true;
      config.text_height = 1.0;
      config.text_scale = 0.5;
      config.intralayer_edge_insertion_skip = 0;
      config.interlayer_edge_insertion_skip = 40;
      config.interlayer_edge_use_color = true;
      config.bounding_box_alpha = 0.5;
      config.use_edge_source = true;
      break;
    case KimeraDsgLayers::PLACES:
      config.z_offset = 1.9 * layer_step;
      config.marker_scale = 0.15;
      config.marker_alpha = 0.5;
      config.use_sphere_marker = false;
      config.edge_alpha = 0.4;
      config.edge_scale = 0.03;
      config.use_label = false;
      config.text_height = 1.0;
      config.text_scale = 0.5;
      config.intralayer_edge_insertion_skip = 0;
      config.interlayer_edge_insertion_skip = 0;
      config.interlayer_edge_use_color = true;
      config.bounding_box_alpha = 0.5;
      config.use_edge_source = false;
      break;
    case KimeraDsgLayers::OBJECTS:
      config.z_offset = 1.2 * layer_step;
      config.marker_scale = 0.25;
      config.marker_alpha = 0.7;
      config.use_sphere_marker = true;
      config.edge_alpha = 0.6;
      config.edge_scale = 0.03;
      config.use_label = false;
      config.text_height = 0.5;
      config.text_scale = 0.25;
      config.intralayer_edge_insertion_skip = 0;
      config.interlayer_edge_insertion_skip = 400;
      config.interlayer_edge_use_color = true;
      config.bounding_box_alpha = 0.6;
      config.use_edge_source = true;
      break;
    default:
      break;
  }

  return config;
}

}  // namespace

SceneGraphVisualizer::SceneGraphVisualizer(const ros::NodeHandle& nh,
                                           const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  // Params
  nh_private_.param("world_frame", world_frame_, world_frame_);
  nh_private_.param("layer_step_z", layer_step_z_, layer_step_z_);
  nh_private_.param("edge_alpha", edge_alpha_, edge_alpha_);

  // Publishers
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
  wall_pub_ = nh_private_.advertise<Marker>("wall_mesh", 1, true);
  semantic_mesh_pub_ =
      nh_private_.advertise<voxblox_msgs::Mesh>("semantic_mesh", 1, true);
  rgb_mesh_pub_ =
      nh_private_.advertise<voxblox_msgs::Mesh>("rgb_mesh", 1, true);
}

void SceneGraphVisualizer::visualizeImpl(const SceneGraph& scene_graph) const {
  if (scene_graph.empty()) {
    ROS_WARN("Request to visualize empty scene graph, skipping.");
    return;
  }

  ROS_INFO("Requested Scene-Graph visualization");
  VLOG(1) << "Displaying centroids";
  displayLayers(scene_graph);

  VLOG(1) << "Displaying edges";
  displayEdges(scene_graph);
}

void SceneGraphVisualizer::fillHeader(Marker& marker,
                                      const ros::Time& current_time) const {
  marker.header.stamp = current_time;
  marker.header.frame_id = world_frame_;
}

void SceneGraphVisualizer::displayLayers(const SceneGraph& scene_graph) const {
  MarkerArray layer_centroids;
  MarkerArray text_markers;
  MarkerArray line_assoc_markers;
  MarkerArray bounding_boxes;

  ros::Time current_time = ros::Time::now();
  for (const auto& id_layer_pair : scene_graph.layers) {
    LayerConfig config = getLayerConfig(id_layer_pair.first, layer_step_z_);
    const SceneGraphLayer& layer = *(id_layer_pair.second);
    VLOG(1) << "Making markers for layer " << layer.id;

    Marker centroids = makeCentroidMarkers(config, layer);
    fillHeader(centroids, current_time);
    layer_centroids.markers.push_back(centroids);

    if (layer.id == to_underlying(KimeraDsgLayers::OBJECTS)) {
      Marker mesh_edges =
          makeMeshEdgesMarker(config, layer, 0.7 * layer_step_z_, 0.0);
      fillHeader(mesh_edges, current_time);
      line_assoc_markers.markers.push_back(mesh_edges);
    }

    const std::string layer_text_ns =
        "layer_" + std::to_string(layer.id) + "_text";
    for (const auto& id_node_pair : layer.nodes) {
      const Node& node = *id_node_pair.second;

      if (config.use_label) {
        Marker node_text = makeTextMarker(config, node, layer_text_ns);
        fillHeader(node_text, current_time);
        text_markers.markers.push_back(node_text);
      }

      if (layer.id == to_underlying(KimeraDsgLayers::OBJECTS)) {
        Marker bounding_box = makeBoundingBoxMarker(config, node);
        fillHeader(bounding_box, current_time);
        bounding_boxes.markers.push_back(bounding_box);
      }
    }
  }

  if (!layer_centroids.markers.empty()) {
    semantic_instance_centroid_pub_.publish(layer_centroids);
  }
  if (!bounding_boxes.markers.empty()) {
    LOG(WARNING) << "Publishing " << bounding_boxes.markers.size()
                 << " markers";
    bounding_box_pub_.publish(bounding_boxes);
  }
  if (!text_markers.markers.empty()) {
    text_markers_pub_.publish(text_markers);
  }
  if (!line_assoc_markers.markers.empty()) {
    edges_centroid_pcl_pub_.publish(line_assoc_markers);
  }
}

void SceneGraphVisualizer::displayEdges(const SceneGraph& scene_graph) const {
  ros::Time current_time = ros::Time::now();

  MarkerArray edge_markers;
  std::map<LayerId, LayerConfig> configs;
  for (const auto& id_layer_pair : scene_graph.layers) {
    LayerConfig config = getLayerConfig(id_layer_pair.first, layer_step_z_);
    configs[id_layer_pair.first] = config;

    if (id_layer_pair.second->numEdges() == 0) {
      continue;  // skip empty layer to avoid rviz errors
    }

    Marker layer_edge_marker = makeLayerEdgeMarkers(
        config, *(id_layer_pair.second), NodeColor::Zero());
    fillHeader(layer_edge_marker, current_time);
    edge_markers.markers.push_back(layer_edge_marker);
  }

  Marker edge_marker = makeGraphEdgeMarkers(scene_graph, configs, 0.04);
  fillHeader(edge_marker, current_time);
  edge_markers.markers.push_back(edge_marker);

  edges_node_node_pub_.publish(edge_markers);
}

Marker SceneGraphVisualizer::makeGraphEdgeMarkers(
    const SceneGraph& graph,
    const std::map<LayerId, LayerConfig>& configs,
    double scale) const {
  Marker marker;
  marker.type = Marker::LINE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = "graph_edges";
  marker.scale.x = scale;
  fillPoseWithIdentity(marker.pose);

  std::map<LayerId, size_t> num_since_last_insertion;

  for (const auto& edge : graph.inter_layer_edges) {
    const Node& source = *(graph.getNode(edge.second.source));
    const Node& target = *(graph.getNode(edge.second.target));

    // parent is always source
    // TODO(nathan) make the above statement an invariant
    if (num_since_last_insertion.count(source.layer) == 0) {
      num_since_last_insertion[source.layer] = 0;
    }

    size_t num_between_insertions =
        configs.at(source.layer).interlayer_edge_insertion_skip;
    if (num_since_last_insertion[source.layer] >= num_between_insertions) {
      num_since_last_insertion[source.layer] = 0;
    } else {
      num_since_last_insertion[source.layer]++;
      continue;
    }

    Eigen::Vector3d source_pos = source.attributes().position;
    source_pos(2) += configs.at(source.layer).z_offset;
    Eigen::Vector3d target_pos = target.attributes().position;
    target_pos(2) += configs.at(target.layer).z_offset;

    geometry_msgs::Point source_point;
    tf2::convert(source_pos, source_point);
    marker.points.push_back(source_point);

    geometry_msgs::Point target_point;
    tf2::convert(target_pos, target_point);
    marker.points.push_back(target_point);

    NodeColor edge_color;
    if (configs.at(source.layer).interlayer_edge_use_color) {
      if (configs.at(source.layer).use_edge_source) {
        // TODO(nathan) this might not be a safe cast in general
        edge_color = source.attributes<SemanticNodeAttributes>().color;
      } else {
        // TODO(nathan) this might not be a safe cast in general
        edge_color = target.attributes<SemanticNodeAttributes>().color;
      }
    } else {
      edge_color = NodeColor::Zero();
    }

    marker.colors.push_back(
        makeColorMsg(edge_color, configs.at(source.layer).edge_alpha));
    marker.colors.push_back(
        makeColorMsg(edge_color, configs.at(source.layer).edge_alpha));
  }

  return marker;
}

voxblox::ColorMode getValidColorMode(const voxblox::Mesh& mesh,
                                     voxblox::ColorMode color_mode) {
  if (color_mode == voxblox::ColorMode::kColor ||
      color_mode == voxblox::ColorMode::kLambertColor) {
    if (!mesh.hasColors()) {
      return voxblox::kGray;
    }
  }

  if (color_mode == voxblox::ColorMode::kNormals ||
      color_mode == voxblox::ColorMode::kLambert ||
      color_mode == voxblox::ColorMode::kLambertColor) {
    if (!mesh.hasNormals()) {
      return voxblox::kColor;
    }
  }

  return color_mode;
}

void SceneGraphVisualizer::visualizeMesh(voxblox::MeshLayer* mesh,
                                         voxblox::ColorMode color_mode,
                                         bool rgb_mesh) const {
  CHECK(mesh);
  voxblox_msgs::Mesh msg;
  voxblox::generateVoxbloxMeshMsg(mesh, color_mode, &msg);
  msg.header.frame_id = world_frame_;
  msg.header.stamp = ros::Time::now();  // TODO(nathan) unify time

  if (rgb_mesh) {
    rgb_mesh_pub_.publish(msg);
  } else {
    semantic_mesh_pub_.publish(msg);
  }
}

void SceneGraphVisualizer::visualizeWalls(const voxblox::Mesh& mesh) const {
  if (!mesh.hasVertices()) {
    return;  // better to not publish an empty marker
  }

  LayerConfig config =
      getLayerConfig(to_underlying(KimeraDsgLayers::PLACES), layer_step_z_);
  voxblox::ColorMode mode =
      getValidColorMode(mesh, voxblox::ColorMode::kLambertColor);
  Marker msg = makeMeshMarker(config, mesh, mode);
  ros::Time curr_time = ros::Time::now();
  fillHeader(msg, curr_time);  // TODO(nathan) unify time

  wall_pub_.publish(msg);
}

}  // namespace kimera
