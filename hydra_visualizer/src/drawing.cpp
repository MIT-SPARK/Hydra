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
#include "hydra_visualizer/drawing.h"

#include <glog/logging.h>
#include <spark_dsg/dynamic_scene_graph.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>
#include <spark_dsg/printing.h>

#include <random>

#include <tf2_eigen/tf2_eigen.hpp>

#include "hydra_visualizer/color/colormap_utilities.h"

namespace hydra::visualizer {
using namespace spark_dsg;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace {

// TODO(nathan) not needed anymore
inline void fillPoseWithIdentity(geometry_msgs::msg::Pose& pose) {
  Eigen::Vector3d identity_pos = Eigen::Vector3d::Zero();
  tf2::convert(identity_pos, pose.position);
  tf2::convert(Eigen::Quaterniond::Identity(), pose.orientation);
}

inline void convertVec3f(const Eigen::Vector3f& v, geometry_msgs::msg::Point& p) {
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
}

}  // namespace

struct JitterGenerator {
  JitterGenerator() : gen(rd()) {}

  double getJitter(const std::string& ns, NodeId node) {
    auto iter = jitters.find(ns);
    if (iter == jitters.end()) {
      iter = jitters.emplace(ns, std::unordered_map<NodeId, double>()).first;
    }

    auto& ns_jitters = iter->second;
    auto node_jitter = ns_jitters.find(node);
    if (node_jitter == ns_jitters.end()) {
      node_jitter = ns_jitters.emplace(node, dist(gen)).first;
    }

    return node_jitter->second;
  }

  std::random_device rd;
  std::mt19937 gen;
  std::uniform_real_distribution<double> dist{-1.0, 1.0};
  std::map<std::string, std::unordered_map<NodeId, double>> jitters;
};

void drawBoundingBox(const spark_dsg::BoundingBox& bbox,
                     const std_msgs::msg::ColorRGBA& color,
                     Marker& marker) {
  // marker.header = header;
  // marker.type = Marker::LINE_LIST;
  // marker.action = visualization_msgs::Marker::ADD;
  // marker.id = 0;
  // marker.ns = ns;
  // marker.scale.x = info.config.bounding_box_scale;
  // fillPoseWithIdentity(marker.pose);

  const static std::array<size_t, 8> remapping{0, 1, 3, 2, 4, 5, 7, 6};
  const auto corners = bbox.corners();

  for (size_t c = 0; c < remapping.size(); ++c) {
    // edges are 1-bit pertubations
    size_t x_neighbor = c | 0x01;
    size_t y_neighbor = c | 0x02;
    size_t z_neighbor = c | 0x04;
    if (c != x_neighbor) {
      convertVec3f(corners[remapping[c]], marker.points.emplace_back());
      convertVec3f(corners[remapping[x_neighbor]], marker.points.emplace_back());
      marker.colors.push_back(color);
      marker.colors.push_back(color);
    }

    if (c != y_neighbor) {
      convertVec3f(corners[remapping[c]], marker.points.emplace_back());
      convertVec3f(corners[remapping[y_neighbor]], marker.points.emplace_back());
      marker.colors.push_back(color);
      marker.colors.push_back(color);
    }

    if (c != z_neighbor) {
      convertVec3f(corners[remapping[c]], marker.points.emplace_back());
      convertVec3f(corners[remapping[z_neighbor]], marker.points.emplace_back());
      marker.colors.push_back(color);
      marker.colors.push_back(color);
    }
  }
}

MarkerArray makeLayerBoundingBoxes(const std_msgs::msg::Header& header,
                                   const LayerInfo& info,
                                   const SceneGraphLayer& layer,
                                   const std::string& ns) {
  // we only draw edges if the graph is not collapsed but the boxes are
  const bool draw_edges =
      info.config.bounding_boxes.collapse && info.z_offset >= 1.0e-6;

  MarkerArray markers;
  markers.markers.resize(draw_edges ? 2 : 1);

  auto& marker = markers.markers[0];
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = ns;
  marker.scale.x = info.config.bounding_boxes.scale;

  fillPoseWithIdentity(marker.pose);
  marker.pose.position.z += info.config.bounding_boxes.collapse ? 0 : info.z_offset;
  marker.points.reserve(24 * layer.numNodes());
  marker.colors.reserve(24 * layer.numNodes());

  Marker* edges = nullptr;
  if (draw_edges) {
    edges = &markers.markers[1];
    edges->header = header;
    edges->type = Marker::LINE_LIST;
    edges->action = Marker::ADD;
    edges->id = 1;
    edges->ns = ns;
    edges->scale.x = info.config.bounding_boxes.edge_scale;
    fillPoseWithIdentity(edges->pose);
    edges->points.reserve(8 * layer.numNodes());
    edges->colors.reserve(8 * layer.numNodes());
  }

  for (const auto& [node_id, node] : layer.nodes()) {
    if (info.filter && !info.filter(*node)) {
      continue;
    }

    const auto attrs = node->tryAttributes<SemanticNodeAttributes>();
    if (!attrs) {
      LOG_FIRST_N(WARNING, 5) << "Unable to draw node '" << NodeSymbol(node_id).str()
                              << "' bounding box";
      continue;
    }

    const auto color =
        makeColorMsg(info.node_color(*node), info.config.bounding_boxes.alpha);
    size_t offset = marker.points.size();
    drawBoundingBox(attrs->bounding_box, color, marker);

    if (edges) {
      geometry_msgs::msg::Point node_centroid;
      tf2::convert(attrs->position, node_centroid);
      node_centroid.z += info.z_offset;

      geometry_msgs::msg::Point center_point;
      tf2::convert(attrs->position, center_point);
      center_point.z += info.config.bounding_boxes.edge_break_ratio * info.z_offset;

      edges->points.push_back(node_centroid);
      edges->colors.push_back(color);
      edges->points.push_back(center_point);
      edges->colors.push_back(color);

      for (size_t i = 0; i < 8; ++i) {
        edges->colors.push_back(color);
      }

      // top box corners appear as the fourth to last and last edge
      // bottom corners account for 8 edges
      edges->points.push_back(center_point);
      edges->points.push_back(marker.points.at(offset + 16));
      edges->points.push_back(center_point);
      edges->points.push_back(marker.points.at(offset + 17));
      edges->points.push_back(center_point);
      edges->points.push_back(marker.points.at(offset + 22));
      edges->points.push_back(center_point);
      edges->points.push_back(marker.points.at(offset + 23));
    }
  }

  return markers;
}

Marker makeLayerEllipseBoundaries(const std_msgs::msg::Header& header,
                                  const LayerInfo& info,
                                  const SceneGraphLayer& layer,
                                  const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = ns;
  marker.scale.x = info.config.boundaries.wireframe_scale;
  fillPoseWithIdentity(marker.pose);
  marker.pose.position.z += info.config.boundaries.collapse ? 0.0 : info.z_offset;

  geometry_msgs::msg::Point last_point;
  std_msgs::msg::ColorRGBA color;

  for (const auto& [node_id, node] : layer.nodes()) {
    const auto attrs = node->tryAttributes<Place2dNodeAttributes>();
    if (!attrs) {
      LOG_FIRST_N(WARNING, 5) << "Unable to draw node '" << NodeSymbol(node_id).str()
                              << "' boundary ellipse";
      continue;
    }

    if (attrs->boundary.size() <= 1) {
      continue;
    }

    color = makeColorMsg(info.node_color(*node), info.config.boundaries.ellipse_alpha);
    const auto pos = attrs->position;
    last_point.x = attrs->ellipse_matrix_expand(0, 0) + attrs->ellipse_centroid(0);
    last_point.y = attrs->ellipse_matrix_expand(1, 0) + attrs->ellipse_centroid(1);
    last_point.z = pos.z();

    int npts = 20;
    for (int ix = 1; ix < npts + 1; ++ix) {
      marker.points.push_back(last_point);
      marker.colors.push_back(color);

      float t = ix * 2 * M_PI / npts;
      Eigen::Vector2d p2 =
          attrs->ellipse_matrix_expand * Eigen::Vector2d(cos(t), sin(t));
      last_point.x = p2(0) + attrs->ellipse_centroid(0);
      last_point.y = p2(1) + attrs->ellipse_centroid(1);
      last_point.z = pos.z();

      marker.points.push_back(last_point);
      marker.colors.push_back(color);
    }
  }

  return marker;
}

Marker makeLayerPolygonEdges(const std_msgs::msg::Header& header,
                             const LayerInfo& info,
                             const SceneGraphLayer& layer,
                             const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = ns;
  marker.scale.x = info.config.boundaries.wireframe_scale;
  fillPoseWithIdentity(marker.pose);

  for (const auto& [node_id, node] : layer.nodes()) {
    const auto attrs = node->tryAttributes<Place2dNodeAttributes>();
    if (!attrs) {
      LOG_FIRST_N(WARNING, 5) << "Unable to draw node '" << NodeSymbol(node_id).str()
                              << "' boundary polygon";
      continue;
    }

    if (attrs->boundary.size() <= 1) {
      continue;
    }

    const auto pos = attrs->position;
    geometry_msgs::msg::Point node_point;
    tf2::convert(pos, node_point);
    node_point.z += info.z_offset;
    const auto color =
        makeColorMsg(info.node_color(*node), info.config.boundaries.alpha);

    for (size_t i = 0; i < attrs->boundary.size(); ++i) {
      geometry_msgs::msg::Point boundary_point;
      tf2::convert(attrs->boundary[i], boundary_point);
      boundary_point.z = pos.z();

      marker.points.push_back(boundary_point);
      marker.colors.push_back(color);
      marker.points.push_back(node_point);
      marker.colors.push_back(color);
    }
  }

  return marker;
}

Marker makeLayerPolygonBoundaries(const std_msgs::msg::Header& header,
                                  const LayerInfo& info,
                                  const SceneGraphLayer& layer,
                                  const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = ns;
  marker.scale.x = info.config.boundaries.wireframe_scale;

  fillPoseWithIdentity(marker.pose);
  marker.pose.position.z += info.config.boundaries.collapse ? 0.0 : info.z_offset;

  for (const auto& [node_id, node] : layer.nodes()) {
    const auto attrs = node->tryAttributes<Place2dNodeAttributes>();
    if (!attrs || attrs->boundary.size() <= 1) {
      continue;
    }

    const auto pos = attrs->position;

    std_msgs::msg::ColorRGBA color;
    if (info.config.boundaries.use_node_color) {
      color = makeColorMsg(info.node_color(*node), info.config.boundaries.alpha);
    } else {
      color = makeColorMsg(Color(), info.config.boundaries.alpha);
    }

    geometry_msgs::msg::Point last_point;
    tf2::convert(attrs->boundary.back(), last_point);
    last_point.z = pos.z();

    for (size_t i = 0; i < attrs->boundary.size(); ++i) {
      marker.points.push_back(last_point);
      marker.colors.push_back(color);

      tf2::convert(attrs->boundary[i], last_point);
      last_point.z = pos.z();
      marker.points.push_back(last_point);
      marker.colors.push_back(color);
    }
  }

  return marker;
}

MarkerArray makeEllipsoidMarkers(const std_msgs::msg::Header& header,
                                 const LayerInfo& info,
                                 const SceneGraphLayer& layer,
                                 const std::string& ns) {
  size_t id = 0;
  MarkerArray msg;
  for (const auto& [node_id, node] : layer.nodes()) {
    const auto& attrs = node->attributes<PlaceNodeAttributes>();
    if (attrs.real_place) {
      continue;
    }

    Marker marker;
    marker.header = header;
    marker.type = Marker::SPHERE;
    marker.action = Marker::ADD;
    marker.id = id++;
    marker.ns = ns;

    marker.scale.x = attrs.frontier_scale.x();
    marker.scale.y = attrs.frontier_scale.y();
    marker.scale.z = attrs.frontier_scale.z();

    tf2::convert(attrs.position, marker.pose.position);
    tf2::convert(attrs.orientation, marker.pose.orientation);

    marker.pose.position.z += info.z_offset;
    marker.color = makeColorMsg(info.node_color(*node), info.config.nodes.alpha);
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray makeLayerNodeTextMarkers(const std_msgs::msg::Header& header,
                                     const LayerInfo& info,
                                     const SceneGraphLayer& layer,
                                     const std::string& ns) {
  MarkerArray msg;
  if (!info.node_text) {
    LOG(WARNING) << "Missing node text function!";
    return msg;
  }

  for (const auto& [node_id, node] : layer.nodes()) {
    if (info.filter && !info.filter(*node)) {
      continue;
    }

    auto& marker = msg.markers.emplace_back();
    marker.header = header;
    marker.ns = ns;
    marker.id = node->id;
    marker.type = Marker::TEXT_VIEW_FACING;
    marker.action = Marker::ADD;

    const auto name = info.node_text(*node);
    if (name.empty()) {
      continue;
    }

    marker.text = name.empty() ? NodeSymbol(node->id).str() : name;
    marker.scale.z = info.config.text.scale;
    marker.color = makeColorMsg(info.text_color());

    fillPoseWithIdentity(marker.pose);
    tf2::convert(node->attributes().position, marker.pose.position);
    marker.pose.position.z += info.config.text.height;
    if (!info.config.text.collapse) {
      marker.pose.position.z += info.z_offset;
    }

    if (info.config.text.add_jitter) {
      static JitterGenerator jitters;
      const auto z_jitter =
          info.config.text.jitter_scale * jitters.getJitter(ns, node_id);
      marker.pose.position.z += z_jitter;
    }
  }

  return msg;
}

Marker makeLayerNodeMarkers(const std_msgs::msg::Header& header,
                            const LayerInfo& info,
                            const SceneGraphLayer& layer,
                            const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.type = info.config.nodes.use_sphere ? Marker::SPHERE_LIST : Marker::CUBE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = ns;

  marker.scale.x = info.config.nodes.scale;
  marker.scale.y = info.config.nodes.scale;
  marker.scale.z = info.config.nodes.scale;

  fillPoseWithIdentity(marker.pose);

  marker.points.reserve(layer.numNodes());
  marker.colors.reserve(layer.numNodes());
  for (const auto& [node_id, node] : layer.nodes()) {
    if (info.filter && !info.filter(*node)) {
      continue;
    }

    geometry_msgs::msg::Point node_centroid;
    tf2::convert(node->attributes().position, node_centroid);
    node_centroid.z += info.z_offset;
    marker.points.push_back(node_centroid);

    const auto desired_color = info.node_color(*node);
    marker.colors.push_back(makeColorMsg(desired_color, info.config.nodes.alpha));
  }

  return marker;
}

Marker makeLayerEdgeMarkers(const std_msgs::msg::Header& header,
                            const LayerInfo& info,
                            const SceneGraphLayer& layer,
                            const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.id = 0;
  marker.ns = ns;

  marker.action = Marker::ADD;
  marker.scale.x = info.config.edges.scale;
  fillPoseWithIdentity(marker.pose);
  if (!info.config.edges.draw) {
    return marker;
  }

  size_t num_seen = 0;
  for (const auto& [key, edge] : layer.edges()) {
    const auto& source_node = layer.getNode(edge.source);
    const auto& target_node = layer.getNode(edge.target);
    if (info.filter && (!info.filter(source_node) || !info.filter(target_node))) {
      continue;
    }

    bool should_skip = num_seen % (info.config.edges.insertion_skip + 1);
    ++num_seen;
    if (should_skip) {
      continue;
    }

    geometry_msgs::msg::Point source;
    tf2::convert(source_node.attributes().position, source);
    source.z += info.z_offset;
    marker.points.push_back(source);

    geometry_msgs::msg::Point target;
    tf2::convert(target_node.attributes().position, target);
    target.z += info.z_offset;
    marker.points.push_back(target);

    const auto [color_source, color_target] = info.edge_color(edge);
    marker.colors.push_back(makeColorMsg(color_source, info.config.edges.alpha));
    marker.colors.push_back(makeColorMsg(color_target, info.config.edges.alpha));
  }

  return marker;
}

// NOTE(nathan) this reuses the normal node text infrastructure, which is mostly fine
// because the two are mutually exclusive
Marker makeLayerTextMarker(const std_msgs::msg::Header& header,
                           const LayerInfo& info,
                           const SceneGraphLayer& layer,
                           const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.type = Marker::TEXT_VIEW_FACING;
  marker.ns = ns;
  marker.id = 0;
  marker.action = Marker::ADD;
  marker.scale.z = info.config.text.scale;
  marker.color = makeColorMsg(info.text_color());

  std::optional<uint64_t> best_stamp;
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  for (const auto& [node_id, node] : layer.nodes()) {
    const auto& attrs = node->attributes();
    if (!best_stamp || attrs.last_update_time_ns >= best_stamp.value()) {
      best_stamp = attrs.last_update_time_ns;
      pos = attrs.position;
      if (info.node_text) {
        marker.text = info.node_text(*node);
      }
    }
  }

  if (marker.text.empty()) {
    std::stringstream ss;
    ss << layer.id;
    marker.text = ss.str();
  }

  fillPoseWithIdentity(marker.pose);
  tf2::convert(pos, marker.pose.position);
  marker.pose.position.z += info.z_offset + info.config.text.height;
  return marker;
}

kimera_pgmo_msgs::msg::Mesh makeMeshMsg(const std_msgs::msg::Header& header,
                                        const spark_dsg::Mesh& mesh,
                                        const std::string& ns,
                                        MeshColoring::Ptr coloring) {
  kimera_pgmo_msgs::msg::Mesh msg;
  msg.header = header;
  msg.ns = ns;

  // Setup default coloring (which is mesh color if available)
  if (!coloring && !mesh.has_colors) {
    UniformMeshColoring::Config config{spark_dsg::Color::gray()};
    coloring = std::make_shared<UniformMeshColoring>(config);
  }

  MeshColorAdapter adapter(mesh, coloring);
  msg.vertices.resize(mesh.points.size());
  for (size_t i = 0; i < mesh.points.size(); ++i) {
    auto& vertex = msg.vertices[i];
    tf2::convert(mesh.points[i].cast<double>().eval(), vertex.pos);
    vertex.has_color = true;
    vertex.color = visualizer::makeColorMsg(adapter.getVertexColor(i));
  }

  msg.triangles.resize(mesh.faces.size());
  for (size_t i = 0; i < mesh.faces.size(); ++i) {
    const auto& face = mesh.faces[i];
    auto& triangle = msg.triangles[i].vertex_indices;
    triangle[0] = face[0];
    triangle[1] = face[1];
    triangle[2] = face[2];
  }

  return msg;
}

}  // namespace hydra::visualizer
