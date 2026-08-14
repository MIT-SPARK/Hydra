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
#include "hydra_ros/places/gvd_visualization_utilities.h"

#include <config_utilities/config.h>
#include <config_utilities/types/enum.h>
#include <hydra_visualizer/color/colormap_utilities.h>
#include <hydra_visualizer/drawing.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include "hydra_ros/visualizer/voxel_drawing.h"

namespace hydra {

using places::GvdGraph;
using places::GvdLayer;
using places::GvdVoxel;
using spark_dsg::Color;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using visualizer::DiscreteColormap;
using visualizer::RangeColormap;

using EdgeMap = std::unordered_map<uint64_t, std::unordered_set<uint64_t>>;

namespace {

spark_dsg::Color getGvdColor(const GvdVisualizerConfig& config,
                             const RangeColormap& colors,
                             double distance,
                             uint8_t num_basis_points) {
  switch (static_cast<GvdVisualizationMode>(config.gvd_mode)) {
    case GvdVisualizationMode::BASIS_POINTS:
      return colors(static_cast<double>(num_basis_points),
                    static_cast<double>(config.min_num_basis),
                    static_cast<double>(config.max_num_basis));
    case GvdVisualizationMode::DISTANCE:
    case GvdVisualizationMode::DEFAULT:
    default:
      return colors(distance, config.gvd_min_distance, config.gvd_max_distance);
  }
}

std::unordered_set<uint64_t>& getNodeSet(EdgeMap& edge_map, uint64_t node) {
  auto iter = edge_map.find(node);
  if (iter == edge_map.end()) {
    iter = edge_map.emplace(node, std::unordered_set<uint64_t>()).first;
  }
  return iter->second;
}

MarkerArray drawWireframe(const GvdGraph& graph,
                          const GvdVisualizerConfig& config,
                          const std::string& ns,
                          size_t marker_id,
                          const std::function<Color(const GvdGraph::Node&)>& coloring) {
  MarkerArray marker;
  if (graph.uncompressed().empty()) {
    return marker;
  }

  const Eigen::Vector3d p_identity = Eigen::Vector3d::Zero();
  const Eigen::Quaterniond q_identity = Eigen::Quaterniond::Identity();
  {  // scope to make handling stuff a little easier
    Marker nodes;
    nodes.type = Marker::SPHERE_LIST;
    nodes.id = marker_id;
    nodes.ns = ns + "_nodes";
    nodes.action = Marker::ADD;
    nodes.scale.x = config.gvd_graph_scale;
    nodes.scale.y = config.gvd_graph_scale;
    nodes.scale.z = config.gvd_graph_scale;
    tf2::convert(p_identity, nodes.pose.position);
    tf2::convert(q_identity, nodes.pose.orientation);
    marker.markers.push_back(nodes);
  }

  {  // scope to make handling stuff a little easier
    Marker edges;
    edges.type = Marker::LINE_LIST;
    edges.id = marker_id;
    edges.ns = ns + "_edges";
    edges.action = Marker::ADD;
    edges.scale.x = config.gvd_graph_scale;
    tf2::convert(p_identity, edges.pose.position);
    tf2::convert(q_identity, edges.pose.orientation);
    marker.markers.push_back(edges);
  }

  auto& nodes = marker.markers[0];
  auto& edges = marker.markers[1];

  EdgeMap seen_edges;
  for (const auto& [node_id, node] : graph.uncompressed()) {
    geometry_msgs::msg::Point node_centroid;
    tf2::convert(node.info.position.cast<double>().eval(), node_centroid);
    nodes.points.push_back(node_centroid);
    const auto color = coloring(node);
    nodes.colors.push_back(visualizer::makeColorMsg(color, config.gvd_alpha));

    auto& curr_seen = getNodeSet(seen_edges, node_id);
    for (const auto sibling : node.siblings) {
      if (curr_seen.count(sibling)) {
        continue;
      }

      curr_seen.insert(sibling);
      getNodeSet(seen_edges, sibling).insert(node_id);

      edges.points.push_back(nodes.points.back());
      edges.colors.push_back(nodes.colors.back());

      const auto& other = graph.uncompressed().at(sibling);
      geometry_msgs::msg::Point neighbor_centroid;
      tf2::convert(other.info.position.cast<double>().eval(), neighbor_centroid);
      edges.points.push_back(neighbor_centroid);
      const auto sibling_color = coloring(other);
      edges.colors.push_back(visualizer::makeColorMsg(sibling_color, config.gvd_alpha));
    }
  }

  return marker;
}

}  // namespace

void declare_config(GvdVisualizerConfig& config) {
  using namespace config;
  name("GvdVisualizerConfig");
  field(config.block_outline_scale, "block_outline_scale");
  field(config.gvd_alpha, "gvd_alpha");
  field(config.gvd_min_distance, "gvd_min_distance");
  field(config.gvd_max_distance, "gvd_max_distance");
  field(config.basis_threshold, "basis_threshold");
  field(config.min_num_basis, "min_num_basis");
  field(config.max_num_basis, "max_num_basis");
  enum_field(config.gvd_mode,
             "gvd_mode",
             {{GvdVisualizationMode::DEFAULT, "DEFAULT"},
              {GvdVisualizationMode::DISTANCE, "DISTANCE"},
              {GvdVisualizationMode::BASIS_POINTS, "BASIS_POINTS"}});
  field(config.gvd_graph_scale, "gvd_graph_scale");
  field(config.esdf_alpha, "esdf_alpha");
  field(config.slice_height, "slice_height");
  field(config.esdf_distance, "esdf_distance");
}

Marker drawEsdf(const GvdVisualizerConfig& config,
                const RangeColormap& cmap,
                const Eigen::Isometry3d& pose,
                const GvdLayer& layer,
                const std::string& ns) {
  VoxelSliceConfig slice{config.slice_height, true};
  return drawVoxelSlice<GvdVoxel>(
      slice,
      std_msgs::msg::Header(),
      layer,
      pose,
      [](const auto& voxel) { return voxel.observed; },
      [&](const auto& voxel) {
        return visualizer::makeColorMsg(cmap(voxel.distance, 0, config.esdf_distance),
                                        config.esdf_alpha);
      },
      ns);
}

Marker drawGvd(const GvdVisualizerConfig& config,
               const RangeColormap& colors,
               const GvdLayer& layer,
               const std::string& ns) {
  Marker marker;
  marker.type = Marker::CUBE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = ns;

  Eigen::Vector3d identity_pos = Eigen::Vector3d::Zero();
  tf2::convert(identity_pos, marker.pose.position);
  tf2::convert(Eigen::Quaterniond::Identity(), marker.pose.orientation);

  marker.scale.x = layer.voxel_size;
  marker.scale.y = layer.voxel_size;
  marker.scale.z = layer.voxel_size;

  for (const auto& block : layer) {
    for (size_t i = 0; i < block.numVoxels(); ++i) {
      const auto& voxel = block.getVoxel(i);
      if (!voxel.observed || voxel.num_extra_basis < config.basis_threshold) {
        continue;
      }

      const Eigen::Vector3d voxel_pos = block.getVoxelPosition(i).cast<double>();
      geometry_msgs::msg::Point marker_pos;
      tf2::convert(voxel_pos, marker_pos);
      marker.points.push_back(marker_pos);

      const auto color =
          getGvdColor(config, colors, voxel.distance, voxel.num_extra_basis);
      marker.colors.push_back(visualizer::makeColorMsg(color, config.gvd_alpha));
    }
  }

  return marker;
}

Marker drawGvdSurface(const GvdVisualizerConfig& config,
                      const RangeColormap& colors,
                      const GvdLayer& layer,
                      const std::string& ns) {
  Marker marker;
  marker.type = Marker::CUBE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = ns;

  Eigen::Vector3d identity_pos = Eigen::Vector3d::Zero();
  tf2::convert(identity_pos, marker.pose.position);
  tf2::convert(Eigen::Quaterniond::Identity(), marker.pose.orientation);

  marker.scale.x = layer.voxel_size;
  marker.scale.y = layer.voxel_size;
  marker.scale.z = layer.voxel_size;

  for (const auto& block : layer) {
    for (size_t i = 0; i < block.numVoxels(); ++i) {
      const auto& voxel = block.getVoxel(i);
      if (!voxel.on_surface) {
        continue;
      }

      Eigen::Vector3d voxel_pos = block.getVoxelPosition(i).cast<double>();
      geometry_msgs::msg::Point marker_pos;
      tf2::convert(voxel_pos, marker_pos);
      marker.points.push_back(marker_pos);

      const auto dist = voxel.distance;
      const auto color = colors(dist, -0.4, 0.4);
      marker.colors.push_back(visualizer::makeColorMsg(color, config.gvd_alpha));
    }
  }

  return marker;
}

MarkerArray drawGvdWireframe(const GvdGraph& graph,
                             const GvdVisualizerConfig& config,
                             const RangeColormap& colors,
                             const std::string& ns,
                             size_t marker_id) {
  return drawWireframe(graph, config, ns, marker_id, [&](const auto& node) {
    return getGvdColor(config, colors, node.info.distance, node.info.num_basis_points);
  });
}

MarkerArray drawGvdActive(const GvdGraph& graph,
                          const GvdVisualizerConfig& config,
                          const std::string& ns,
                          size_t marker_id) {
  return drawWireframe(graph, config, ns, marker_id, [&](const auto& node) {
    return node.archived ? Color::black() : Color::green();
  });
}

MarkerArray drawGvdActiveCluster(const GvdGraph& graph,
                                 const GvdVisualizerConfig& config,
                                 const std::string& ns,
                                 size_t marker_id) {
  return drawWireframe(graph, config, ns, marker_id, [&](const auto& node) {
    auto parent = graph.remapping().at(node.id);
    const auto active = !graph.compressed().at(parent).archived();
    return active ? Color::green() : Color::black();
  });
}

MarkerArray drawGvdClusters(const GvdGraph& graph,
                            const GvdVisualizerConfig& config,
                            const std::string& ns,
                            const DiscreteColormap& colormap,
                            size_t marker_id) {
  std::map<uint64_t, size_t> color_indices;
  for (const auto& [node_id, node] : graph.compressed()) {
    size_t max_color = 0;
    std::set<size_t> seen_colors;
    for (const auto sibling : node.siblings) {
      const auto iter = color_indices.find(sibling);
      if (iter == color_indices.end()) {
        continue;
      }

      seen_colors.insert(iter->second);
      if (iter->second > max_color) {
        max_color = iter->second;
      }
    }

    if (seen_colors.empty()) {
      color_indices[node_id] = 0;
      continue;
    }

    bool found_color = false;
    for (size_t i = 0; i < max_color; ++i) {
      if (!seen_colors.count(i)) {
        color_indices[node_id] = i;
        found_color = true;
        break;
      }
    }

    if (found_color) {
      continue;
    }

    color_indices[node_id] = max_color + 1;
  }

  std::map<uint64_t, spark_dsg::Color> colors;
  for (const auto& [node_id, color_id] : color_indices) {
    colors[node_id] = colormap(color_id);
  }

  return drawWireframe(graph, config, ns, marker_id, [&](const auto& node) {
    auto iter = graph.remapping().find(node.id);
    if (iter == graph.remapping().end()) {
      return Color::black();
    } else {
      return colors.at(iter->second);
    }
  });
}

MarkerArray drawCompressedGraph(const places::GraphExtractor::LocalGraph& graph,
                                const GvdVisualizerConfig& config,
                                const visualizer::RangeColormap& cmap,
                                const std::string& ns,
                                double node_scale,
                                double edge_scale,
                                double alpha,
                                size_t marker_id) {
  MarkerArray marker;
  if (!graph.num_nodes()) {
    return marker;
  }

  marker.markers.reserve(2);

  {  // scope limiting node marker validity
    auto& nodes = marker.markers.emplace_back();
    nodes.type = Marker::CUBE_LIST;
    nodes.id = marker_id;
    nodes.ns = ns + "_nodes";
    nodes.action = Marker::ADD;
    nodes.scale.x = node_scale;
    nodes.scale.y = node_scale;
    nodes.scale.z = node_scale;

    for (const auto& [node_id, node] : graph) {
      const auto& attrs = node.attributes();
      geometry_msgs::msg::Point node_centroid;
      tf2::convert(attrs.position.cast<double>().eval(), nodes.points.emplace_back());

      const auto dist = attrs.distance;
      const auto color = cmap(dist, config.gvd_min_distance, config.gvd_max_distance);
      nodes.colors.push_back(visualizer::makeColorMsg(color, alpha));
    }
  }  // end scope limiting node marker validity

  if (!graph.num_edges()) {
    return marker;
  }

  auto& edges = marker.markers.emplace_back();
  edges.type = Marker::LINE_LIST;
  edges.id = marker_id;
  edges.ns = ns + "_edges";
  edges.action = Marker::ADD;
  edges.scale.x = edge_scale;

  for (const auto& [key, info] : graph.edges()) {
    const auto [source, target] = key;
    const auto& source_pos = graph.at(source).position;
    const auto& target_pos = graph.at(target).position;
    tf2::convert(source_pos, edges.points.emplace_back());
    tf2::convert(target_pos, edges.points.emplace_back());

    const auto dist = info ? info->weight : 0.0;
    const auto color = cmap(dist, config.gvd_min_distance, config.gvd_max_distance);
    const auto color_msg = visualizer::makeColorMsg(color, alpha);
    edges.colors.push_back(color_msg);
    edges.colors.push_back(color_msg);
  }

  return marker;
}

}  // namespace hydra
