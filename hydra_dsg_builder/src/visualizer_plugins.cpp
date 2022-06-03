/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra_dsg_builder/visualizer_plugins.h"

#include <hydra_utils/colormap_utils.h>
#include <hydra_utils/visualizer_utils.h>

#include <mesh_msgs/TriangleMesh.h>
#include <mesh_msgs/TriangleMeshStamped.h>

#include <tf2_eigen/tf2_eigen.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace hydra {

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

  invalid_color << 64, 235, 52;

  // grey
  std::vector<double> interior_color_float{0.3333, 0.3764, 0.4509};
  READ_PARAM(nh, interior_color_float);
  if (interior_color_float.size() != 3) {
    throw std::runtime_error("color size must be 3!");
  }
  interior_color << 255 * interior_color_float[0], 255 * interior_color_float[1],
      255 * interior_color_float[2];

  // TODO(nathan) fix the config for this
}

#undef READ_PARAM

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

MeshPlaceConnectionsPlugin::MeshPlaceConnectionsPlugin(const ros::NodeHandle& nh,
                                                       const std::string& name)
    : DsgVisualizerPlugin(nh, name),
      config_(nh),
      published_nodes_(false),
      published_edges_(false) {
  marker_pub_ = nh_.advertise<MarkerArray>("places_mesh_connection_viz", 10);
}

void MeshPlaceConnectionsPlugin::draw(const std_msgs::Header& header,
                                      const DynamicSceneGraph& graph) {
  if (!graph.hasLayer(DsgLayers::PLACES)) {
    return;
  }

  if (!graph.hasMesh()) {
    return;
  }

  const auto& layer = graph.getLayer(DsgLayers::PLACES);

  MinimumSpanningTreeInfo mst_info = getMinimumSpanningEdges(layer);

  VisualizerConfig viz_config;
  viz_config.layer_z_step = 0.0;

  MarkerArray msg;

  Marker node_marker =
      makeCentroidMarkers(header,
                          config_.layer_config,
                          layer,
                          viz_config,
                          "place_mesh_graph_nodes",
                          [&](const SceneGraphNode& node) {
                            if (mst_info.counts.at(node.id) == 0) {
                              return config_.invalid_color;
                            } else if (mst_info.leaves.count(node.id)) {
                              return config_.leaf_color;
                            } else {
                              return config_.interior_color;
                            }
                          });
  if (!node_marker.points.empty()) {
    msg.markers.push_back(node_marker);
    published_nodes_ = true;
  }

  Marker mst_edge_marker = makeMstEdges(config_, mst_info, layer);
  if (!mst_edge_marker.points.empty()) {
    mst_edge_marker.header = header;
    msg.markers.push_back(mst_edge_marker);
    published_edges_ = true;
  }

  if (!msg.markers.empty()) {
    marker_pub_.publish(msg);
  }
}

void MeshPlaceConnectionsPlugin::reset(const std_msgs::Header& header,
                                       const DynamicSceneGraph&) {
  MarkerArray msg;
  if (published_nodes_) {
    msg.markers.push_back(makeDeleteMarker(header, 0, "place_mesh_graph_nodes"));
    published_nodes_ = false;
  }

  if (published_edges_) {
    msg.markers.push_back(makeDeleteMarker(header, 0, "places_mesh_graph_mst_edges"));
    published_edges_ = false;
  }

  if (!msg.markers.empty()) {
    marker_pub_.publish(msg);
  }
}

PlacesFactorGraphViz::PlacesFactorGraphViz(const ros::NodeHandle& nh)
    : nh_(nh), config_(nh) {
  marker_pub_ = nh_.advertise<MarkerArray>("places_factor_graph", 10);
}

void PlacesFactorGraphViz::draw(char vertex_prefix,
                                const SceneGraphLayer& places,
                                const MinimumSpanningTreeInfo& mst_info,
                                const DeformationGraph& deformations) {
  VisualizerConfig viz_config;
  viz_config.layer_z_step = 0.0;

  std_msgs::Header header;
  header.frame_id = "world";
  header.stamp = ros::Time::now();

  MarkerArray msg =
      makeLeafEdges(config_, vertex_prefix, mst_info, deformations, places);
  for (auto& marker : msg.markers) {
    marker.header = header;
  }

  Marker node_marker = makeCentroidMarkers(
      header,
      config_.layer_config,
      places,
      viz_config,
      "place_factor_graph_nodes",
      [&](const SceneGraphNode& node) {
        if (!node.hasSiblings()) {
          VLOG(3) << "Invalid node: " << NodeSymbol(node.id).getLabel();
          return config_.invalid_color;
        } else if (mst_info.leaves.count(node.id)) {
          return config_.leaf_color;
        } else {
          return config_.interior_color;
        }
      });
  if (!node_marker.points.empty()) {
    msg.markers.push_back(node_marker);
  }

  Marker mst_edge_marker = makeMstEdges(config_, mst_info, places);
  if (!mst_edge_marker.points.empty()) {
    mst_edge_marker.header = header;
    msg.markers.push_back(mst_edge_marker);
  }

  if (!msg.markers.empty()) {
    marker_pub_.publish(msg);
  }
}

PlaceParentsPlugin::PlaceParentsPlugin(const ros::NodeHandle& nh,
                                       const std::string& name)
    : DsgVisualizerPlugin(nh, name),
      config_(nh),
      published_nodes_(false),
      published_edges_(false) {
  marker_pub_ = nh_.advertise<MarkerArray>("places_parents", 10);
}

void PlaceParentsPlugin::draw(const std_msgs::Header& header,
                              const DynamicSceneGraph& graph) {
  if (!graph.hasLayer(DsgLayers::PLACES)) {
    return;
  }

  const auto& layer = graph.getLayer(DsgLayers::PLACES);

  VisualizerConfig viz_config;
  viz_config.layer_z_step = 0.0;

  MarkerArray msg;

  Marker nodes = makeCentroidMarkers(
      header,
      config_.layer_config,
      layer,
      viz_config,
      "places_parent_graph_nodes",
      [&](const SceneGraphNode&) { return config_.interior_color; });

  if (!nodes.points.empty()) {
    msg.markers.push_back(nodes);
    published_nodes_ = true;
  }

  Marker edges;
  edges.type = Marker::LINE_LIST;
  edges.action = Marker::ADD;
  edges.id = 0;
  edges.ns = "places_parent_edges";
  edges.scale.x = config_.mesh_edge_scale;
  edges.color =
      makeColorMsg(config_.interior_color, config_.layer_config.intralayer_edge_alpha);

  Marker parents;
  parents.type = Marker::CUBE_LIST;
  parents.action = Marker::ADD;
  parents.id = 0;
  parents.ns = "places_parents";
  parents.scale.x = config_.mesh_marker_scale;
  parents.scale.y = config_.mesh_marker_scale;
  parents.scale.z = config_.mesh_marker_scale;
  parents.color = makeColorMsg(config_.leaf_color, config_.mesh_marker_alpha);

  fillPoseWithIdentity(edges);
  fillPoseWithIdentity(parents);

  for (const auto& id_node_pair : layer.nodes()) {
    auto& attrs = id_node_pair.second->attributes<PlaceNodeAttributes>();
    geometry_msgs::Point start;
    tf2::convert(attrs.position, start);

    for (const auto& info : attrs.voxblox_mesh_connections) {
      geometry_msgs::Point end;
      Eigen::Vector3d pos = Eigen::Map<const Eigen::Vector3d>(info.voxel_pos);
      tf2::convert(pos, end);
      edges.points.push_back(start);
      edges.points.push_back(end);
      parents.points.push_back(end);
    }
  }

  if (!edges.points.empty()) {
    edges.header = header;
    parents.header = header;
    msg.markers.push_back(edges);
    msg.markers.push_back(parents);
    published_edges_ = true;
  }

  if (!msg.markers.empty()) {
    marker_pub_.publish(msg);
  }
}

void PlaceParentsPlugin::reset(const std_msgs::Header& header,
                               const DynamicSceneGraph&) {
  MarkerArray msg;
  if (published_nodes_) {
    msg.markers.push_back(makeDeleteMarker(header, 0, "places_parent_graph_nodes"));
    published_nodes_ = false;
  }

  if (published_edges_) {
    msg.markers.push_back(makeDeleteMarker(header, 0, "places_parent_edges"));
    msg.markers.push_back(makeDeleteMarker(header, 0, "places_parents"));
    published_edges_ = false;
  }

  if (!msg.markers.empty()) {
    marker_pub_.publish(msg);
  }
}

}  // namespace hydra
