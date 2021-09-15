#include "kimera_dsg_builder/visualizer_plugins.h"
#include "kimera_dsg_builder/minimum_spanning_tree.h"
#include "kimera_dsg_builder/timing_utilities.h"

#include <kimera_dsg_visualizer/colormap_utils.h>
#include <kimera_dsg_visualizer/visualizer_utils.h>

#include <tf2_eigen/tf2_eigen.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace kimera {

using dsg_utils::makeColorMsg;
using kimera_pgmo::DeformationGraph;
using kimera_pgmo::DeformationGraphPtr;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

#define READ_PARAM(nh, variable) nh.getParam(#variable, variable)

PMGraphPluginConfig::PMGraphPluginConfig(const ros::NodeHandle& nh) {
  READ_PARAM(nh, mesh_edge_scale);
  READ_PARAM(nh, mesh_edge_alpha);
  READ_PARAM(nh, mesh_marker_scale);
  READ_PARAM(nh, mesh_marker_alpha);

  // purple
  std::vector<double> leaf_color_float{0.662, 0.0313, 0.7607};
  READ_PARAM(nh, leaf_color_float);
  if (leaf_color_float.size() != 3) {
    throw std::runtime_error("color size must be 3!");
  }
  leaf_color << 255 * leaf_color_float[0], 255 * leaf_color_float[1],
      255 * leaf_color_float[2];

  // grey
  std::vector<double> interior_color_float{0.3333, 0.3764, 0.4509};
  READ_PARAM(nh, interior_color_float);
  if (interior_color_float.size() != 3) {
    throw std::runtime_error("color size must be 3!");
  }
  interior_color << 255 * interior_color_float[0], 255 * interior_color_float[1],
      255 * interior_color_float[2];

  layer_config = getLayerConfig(nh.resolveName("graph"));
}

MeshPlaceConnectionsPlugin::MeshPlaceConnectionsPlugin(const ros::NodeHandle& nh,
                                                       const std::string& name,
                                                       char vertex_prefix,
                                                       const DeformationGraphPtr& graph)
    : DsgVisualizerPlugin(nh, name),
      vertex_prefix_(vertex_prefix),
      deformation_graph_(graph),
      config_(nh) {
  marker_pub_ = nh_.advertise<MarkerArray>("places_mesh_connection_viz", 10);
}

inline void fillPoseWithIdentity(Marker& marker) {
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  tf2::convert(origin, marker.pose.position);
  tf2::convert(Eigen::Quaterniond::Identity(), marker.pose.orientation);
}

MarkerArray makeLeafEdges(const PMGraphPluginConfig& config,
                          char vertex_prefix,
                          const MinimumSpanningTreeInfo& mst_info,
                          const DeformationGraph& dgraph,
                          const SceneGraphLayer& layer) {
  MarkerArray markers;
  if (!dgraph.hasVertexKey(vertex_prefix)) {
    return markers;
  }

  Marker edges;
  edges.type = Marker::LINE_LIST;
  edges.action = Marker::ADD;
  edges.id = 0;
  edges.ns = "places_mesh_graph_leaf_edges";

  Marker vertices;
  vertices.type = Marker::CUBE_LIST;
  vertices.action = Marker::ADD;
  vertices.id = 0;
  vertices.ns = "places_mesh_graph_vertices";

  edges.scale.x = config.mesh_edge_scale;
  edges.color = makeColorMsg(config.leaf_color, config.mesh_edge_alpha);
  vertices.scale.x = config.mesh_marker_scale;
  vertices.scale.y = config.mesh_marker_scale;
  vertices.scale.z = config.mesh_marker_scale;
  vertices.color = makeColorMsg(config.leaf_color, config.mesh_marker_alpha);

  fillPoseWithIdentity(edges);
  fillPoseWithIdentity(vertices);

  std::vector<gtsam::Point3> vertex_positions =
      dgraph.getInitialPositionsVertices(vertex_prefix);

  std::set<size_t> seen;
  for (const auto& id_node_pair : layer.nodes()) {
    if (!mst_info.leaves.count(id_node_pair.first)) {
      continue;
    }

    const PlaceNodeAttributes& attrs =
        id_node_pair.second->attributes<PlaceNodeAttributes>();

    geometry_msgs::Point start;
    tf2::convert(attrs.position, start);

    for (const auto vertex : attrs.pcl_mesh_connections) {
      if (vertex >= vertex_positions.size()) {
        continue;
      }

      const gtsam::Point3& pos = vertex_positions.at(vertex);
      geometry_msgs::Point end;
      end.x = pos.x();
      end.y = pos.y();
      end.z = pos.z();
      edges.points.push_back(start);
      edges.points.push_back(end);

      if (seen.count(vertex)) {
        continue;
      }

      seen.insert(vertex);
      vertices.points.push_back(end);
    }
  }

  markers.markers.push_back(edges);
  markers.markers.push_back(vertices);
  return markers;
}

Marker makeMstEdges(const PMGraphPluginConfig& config,
                    const MinimumSpanningTreeInfo& mst_info,
                    const SceneGraphLayer& layer) {
  Marker edges;
  edges.type = Marker::LINE_LIST;
  edges.action = Marker::ADD;
  edges.id = 0;
  edges.ns = "places_mesh_graph_mst_edges";

  edges.scale.x = config.mesh_edge_scale;
  edges.color =
      makeColorMsg(NodeColor::Zero(), config.layer_config.intralayer_edge_alpha);

  fillPoseWithIdentity(edges);

  for (const auto& edge : mst_info.edges) {
    Eigen::Vector3d start_pos = layer.getPosition(edge.source);
    geometry_msgs::Point source;
    tf2::convert(start_pos, source);

    Eigen::Vector3d end_pos = layer.getPosition(edge.target);
    geometry_msgs::Point target;
    tf2::convert(end_pos, target);

    edges.points.push_back(source);
    edges.points.push_back(target);
  }

  return edges;
}

void MeshPlaceConnectionsPlugin::draw(const DynamicSceneGraph& graph) {
  if (!graph.hasLayer(KimeraDsgLayers::PLACES)) {
    return;
  }

  if (!graph.hasMesh()) {
    return;
  }

  const SceneGraphLayer& layer = *graph.getLayer(KimeraDsgLayers::PLACES);

  MinimumSpanningTreeInfo mst_info;
  {  // timing scope
    ScopedTimer timer("visualizer/mst_creation", true, 2, false);
    mst_info = getMinimumSpanningEdges(layer);
  }  // timing scope

  MarkerArray msg =
      makeLeafEdges(config_, vertex_prefix_, mst_info, *deformation_graph_, layer);

  VisualizerConfig viz_config;
  viz_config.layer_z_step = 0.0;

  Marker node_marker = makeCentroidMarkers(config_.layer_config,
                                           layer,
                                           viz_config,
                                           "place_mesh_graph_nodes",
                                           [&](const SceneGraphNode& node) {
                                             if (mst_info.leaves.count(node.id)) {
                                               return config_.leaf_color;
                                             }
                                             return config_.interior_color;
                                           });
  if (!node_marker.points.empty()) {
    msg.markers.push_back(node_marker);
  }

  Marker mst_edge_marker = makeMstEdges(config_, mst_info, layer);
  if (!mst_edge_marker.points.empty()) {
    msg.markers.push_back(mst_edge_marker);
  }

  ros::Time curr_time = ros::Time::now();
  for (auto& marker : msg.markers) {
    marker.header.stamp = curr_time;
    marker.header.frame_id = config_.frame;
  }

  if (!msg.markers.empty()) {
    marker_pub_.publish(msg);
  }
}

}  // namespace kimera
