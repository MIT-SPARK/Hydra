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
#include "hydra_ros/visualizer/gvd_visualization_utilities.h"

#include <tf2_eigen/tf2_eigen.h>

#include <random>

#include "hydra_ros/visualizer/colormap_utilities.h"

namespace hydra {

using places::GvdGraph;
using places::GvdVoxel;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
using voxblox::BlockIndexList;
using voxblox::FloatingPoint;
using voxblox::Layer;
using voxblox::MeshLayer;
using voxblox::TsdfVoxel;

MarkerGroupPub::MarkerGroupPub(const ros::NodeHandle& nh) : nh_(nh) {}

void MarkerGroupPub::publish(const std::string& name,
                             const MarkerCallback& func) const {
  publish(name, [&func](MarkerArray& msg) {
    Marker marker;
    msg.markers.push_back(marker);
    return func(msg.markers.back());
  });
}

void MarkerGroupPub::publish(const std::string& name, const ArrayCallback& func) const {
  auto iter = pubs_.find(name);
  if (iter == pubs_.end()) {
    iter = pubs_.emplace(name, nh_.advertise<MarkerArray>(name, 1, true)).first;
  }

  if (!iter->second.getNumSubscribers()) {
    return;  // avoid doing computation if we don't need to publish
  }

  MarkerArray msg;
  if (func(msg)) {
    iter->second.publish(msg);
  }
}

double computeRatio(double min, double max, double value) {
  double ratio = (value - min) / (max - min);
  ratio = !std::isfinite(ratio) ? 0.0 : ratio;
  return ratio;
}

double getRatioFromDistance(const GvdVisualizerConfig& config, const GvdVoxel& voxel) {
  return computeRatio(config.gvd_min_distance, config.gvd_max_distance, voxel.distance);
}

double getRatioFromBasisPoints(const GvdVisualizerConfig& config,
                               const GvdVoxel& voxel) {
  return computeRatio(static_cast<double>(config.min_num_basis),
                      static_cast<double>(config.max_num_basis),
                      static_cast<double>(voxel.num_extra_basis));
}

double getRatio(const GvdVisualizerConfig& config, const GvdVoxel& voxel) {
  switch (static_cast<GvdVisualizationMode>(config.gvd_mode)) {
    case GvdVisualizationMode::BASIS_POINTS:
      return getRatioFromBasisPoints(config, voxel);
    case GvdVisualizationMode::DISTANCE:
    case GvdVisualizationMode::DEFAULT:
    default:
      return getRatioFromDistance(config, voxel);
  }
  return 0.0;
}

Marker makeGvdMarker(const GvdVisualizerConfig& config,
                     const ColormapConfig& colors,
                     const Layer<GvdVoxel>& layer) {
  BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);

  Marker marker;
  marker.type = Marker::CUBE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = "gvd_markers";

  Eigen::Vector3d identity_pos = Eigen::Vector3d::Zero();
  tf2::convert(identity_pos, marker.pose.position);
  tf2::convert(Eigen::Quaterniond::Identity(), marker.pose.orientation);

  marker.scale.x = layer.voxel_size();
  marker.scale.y = layer.voxel_size();
  marker.scale.z = layer.voxel_size();

  for (const auto& idx : blocks) {
    const auto& block = layer.getBlockByIndex(idx);
    for (size_t i = 0; i < block.num_voxels(); ++i) {
      const auto& voxel = block.getVoxelByLinearIndex(i);
      if (!voxel.observed || voxel.num_extra_basis < config.basis_threshold) {
        continue;
      }

      Eigen::Vector3d voxel_pos =
          block.computeCoordinatesFromLinearIndex(i).cast<double>();
      geometry_msgs::Point marker_pos;
      tf2::convert(voxel_pos, marker_pos);
      marker.points.push_back(marker_pos);

      double ratio = getRatio(config, voxel);
      NodeColor color = dsg_utils::interpolateColorMap(colors, ratio);

      std_msgs::ColorRGBA color_msg = dsg_utils::makeColorMsg(color, config.gvd_alpha);
      marker.colors.push_back(color_msg);
    }
  }

  return marker;
}

Marker makeErrorMarker(const GvdVisualizerConfig& config,
                       const ColormapConfig& colors,
                       const Layer<GvdVoxel>& lhs,
                       const Layer<GvdVoxel>& rhs,
                       double threshold) {
  Marker marker;
  marker.type = Marker::CUBE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = "error_locations";

  Eigen::Vector3d identity_pos = Eigen::Vector3d::Zero();
  tf2::convert(identity_pos, marker.pose.position);
  tf2::convert(Eigen::Quaterniond::Identity(), marker.pose.orientation);

  marker.scale.x = lhs.voxel_size();
  marker.scale.y = lhs.voxel_size();
  marker.scale.z = lhs.voxel_size();

  BlockIndexList blocks;
  lhs.getAllAllocatedBlocks(&blocks);

  for (const auto& idx : blocks) {
    if (!rhs.hasBlock(idx)) {
      continue;
    }

    const auto& lhs_block = lhs.getBlockByIndex(idx);
    const auto& rhs_block = rhs.getBlockByIndex(idx);

    for (size_t i = 0; i < lhs_block.num_voxels(); ++i) {
      const auto& lvoxel = lhs_block.getVoxelByLinearIndex(i);
      const auto& rvoxel = rhs_block.getVoxelByLinearIndex(i);

      if (!lvoxel.observed || !rvoxel.observed) {
        continue;
      }

      const double error = std::abs(lvoxel.distance - rvoxel.distance);
      if (error <= threshold) {
        continue;
      }

      double ratio = computeRatio(0, 10, error);
      NodeColor color = dsg_utils::interpolateColorMap(colors, ratio);

      Eigen::Vector3d voxel_pos =
          lhs_block.computeCoordinatesFromLinearIndex(i).cast<double>();
      geometry_msgs::Point marker_pos;
      tf2::convert(voxel_pos, marker_pos);
      marker.points.push_back(marker_pos);

      std_msgs::ColorRGBA color_msg = dsg_utils::makeColorMsg(color, config.gvd_alpha);
      marker.colors.push_back(color_msg);
    }
  }

  return marker;
}

Marker makeSurfaceVoxelMarker(const GvdVisualizerConfig& config,
                              const ColormapConfig& colors,
                              const Layer<GvdVoxel>& layer) {
  BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);

  Marker marker;
  marker.type = Marker::CUBE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = "surface_markers";

  Eigen::Vector3d identity_pos = Eigen::Vector3d::Zero();
  tf2::convert(identity_pos, marker.pose.position);
  tf2::convert(Eigen::Quaterniond::Identity(), marker.pose.orientation);

  marker.scale.x = layer.voxel_size();
  marker.scale.y = layer.voxel_size();
  marker.scale.z = layer.voxel_size();

  for (const auto& idx : blocks) {
    const auto& block = layer.getBlockByIndex(idx);
    for (size_t i = 0; i < block.num_voxels(); ++i) {
      const auto& voxel = block.getVoxelByLinearIndex(i);
      if (!voxel.on_surface) {
        continue;
      }

      Eigen::Vector3d voxel_pos =
          block.computeCoordinatesFromLinearIndex(i).cast<double>();
      geometry_msgs::Point marker_pos;
      tf2::convert(voxel_pos, marker_pos);
      marker.points.push_back(marker_pos);

      double ratio = computeRatio(-0.4, 0.4, voxel.distance);
      NodeColor color = dsg_utils::interpolateColorMap(colors, ratio);

      std_msgs::ColorRGBA color_msg = dsg_utils::makeColorMsg(color, config.gvd_alpha);
      marker.colors.push_back(color_msg);
    }
  }

  return marker;
}

Marker makeEsdfMarker(const GvdVisualizerConfig& config,
                      const ColormapConfig& colors,
                      const Layer<GvdVoxel>& layer) {
  BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);

  Marker marker;
  marker.type = Marker::CUBE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = "esdf_slice_markers";

  Eigen::Vector3d identity_pos = Eigen::Vector3d::Zero();
  tf2::convert(identity_pos, marker.pose.position);
  tf2::convert(Eigen::Quaterniond::Identity(), marker.pose.orientation);

  marker.scale.x = layer.voxel_size();
  marker.scale.y = layer.voxel_size();
  marker.scale.z = layer.voxel_size();

  const FloatingPoint voxel_size = layer.voxel_size();
  const FloatingPoint half_voxel_size = voxel_size / 2.0;
  // rounds down and points the slice at the middle of the nearest voxel boundary
  const FloatingPoint slice_height =
      std::floor(config.slice_height / voxel_size) * voxel_size + half_voxel_size;

  for (const auto& idx : blocks) {
    const auto& block = layer.getBlockByIndex(idx);
    for (size_t i = 0; i < block.num_voxels(); ++i) {
      const auto& voxel = block.getVoxelByLinearIndex(i);
      if (!voxel.observed) {
        continue;
      }

      Eigen::Vector3d voxel_pos =
          block.computeCoordinatesFromLinearIndex(i).cast<double>();
      if (voxel_pos(2) < slice_height - half_voxel_size ||
          voxel_pos(2) > slice_height + half_voxel_size) {
        continue;
      }

      geometry_msgs::Point marker_pos;
      tf2::convert(voxel_pos, marker_pos);
      marker.points.push_back(marker_pos);

      double ratio = computeRatio(
          config.esdf_min_distance, config.esdf_max_distance, voxel.distance);
      NodeColor color = dsg_utils::interpolateColorMap(colors, ratio);

      std_msgs::ColorRGBA color_msg = dsg_utils::makeColorMsg(color, config.esdf_alpha);
      marker.colors.push_back(color_msg);
    }
  }

  return marker;
}

inline Eigen::Vector3d getOffset(double side_length,
                                 bool x_high,
                                 bool y_high,
                                 bool z_high) {
  Eigen::Vector3d offset;
  offset(0) = x_high ? side_length : 0.0;
  offset(1) = y_high ? side_length : 0.0;
  offset(2) = z_high ? side_length : 0.0;
  return offset;
}

geometry_msgs::Point getPointFromMatrix(const Eigen::MatrixXd& matrix, int col) {
  geometry_msgs::Point point;
  tf2::convert(matrix.block<3, 1>(0, col).eval(), point);
  return point;
}

void fillMarkerFromBlock(Marker& marker,
                         const Eigen::Vector3d& position,
                         double side_length) {
  Eigen::MatrixXd corners(3, 8);
  for (int c = 0; c < corners.cols(); ++c) {
    // x: lsb, y: second lsb, z: third lsb
    corners.block<3, 1>(0, c) =
        position +
        getOffset(side_length, ((c & 0x01) != 0), ((c & 0x02) != 0), ((c & 0x04) != 0));
  }

  for (int c = 0; c < corners.cols(); ++c) {
    // edges are 1-bit pertubations
    int x_neighbor = c | 0x01;
    int y_neighbor = c | 0x02;
    int z_neighbor = c | 0x04;
    if (c != x_neighbor) {
      marker.points.push_back(getPointFromMatrix(corners, c));
      marker.points.push_back(getPointFromMatrix(corners, x_neighbor));
    }
    if (c != y_neighbor) {
      marker.points.push_back(getPointFromMatrix(corners, c));
      marker.points.push_back(getPointFromMatrix(corners, y_neighbor));
    }
    if (c != z_neighbor) {
      marker.points.push_back(getPointFromMatrix(corners, c));
      marker.points.push_back(getPointFromMatrix(corners, z_neighbor));
    }
  }
}

template <typename LayerType>
Marker makeBlocksMarkerImpl(const LayerType& layer, double scale) {
  Marker marker;
  marker.type = Marker::LINE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.color.r = 0.662;
  marker.color.g = 0.0313;
  marker.color.b = 0.7607;
  marker.color.a = 0.8;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;

  Eigen::Vector3d identity_pos = Eigen::Vector3d::Zero();
  tf2::convert(identity_pos, marker.pose.position);
  tf2::convert(Eigen::Quaterniond::Identity(), marker.pose.orientation);

  BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);

  for (const auto& idx : blocks) {
    const auto& block = layer.getBlockByIndex(idx);
    Eigen::Vector3f block_pos = block.origin();
    fillMarkerFromBlock(marker, block_pos.cast<double>(), block.block_size());
  }

  return marker;
}

Marker makeMeshBlocksMarker(const MeshLayer& layer, double scale) {
  Marker marker;
  marker.type = Marker::LINE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;

  Eigen::Vector3d identity_pos = Eigen::Vector3d::Zero();
  tf2::convert(identity_pos, marker.pose.position);
  tf2::convert(Eigen::Quaterniond::Identity(), marker.pose.orientation);

  BlockIndexList blocks;
  layer.getAllAllocatedMeshes(&blocks);

  std_msgs::ColorRGBA good;
  good.r = 0.2;
  good.g = 1.0;
  good.b = 0.2;
  good.a = 0.8;

  std_msgs::ColorRGBA bad;
  bad.r = 1.0;
  bad.g = 0.2;
  bad.b = 0.2;
  bad.a = 0.8;

  for (const auto& idx : blocks) {
    const auto block = layer.getMeshPtrByIndex(idx);
    Eigen::Vector3f block_pos = block->origin;
    fillMarkerFromBlock(marker, block_pos.cast<double>(), block->block_size);

    while (marker.colors.size() < marker.points.size()) {
      marker.colors.push_back(block->vertices.size() != 0 ? good : bad);
    }
  }

  return marker;
}

Marker makeBlocksMarker(const Layer<TsdfVoxel>& layer, double scale) {
  return makeBlocksMarkerImpl(layer, scale);
}

Marker makeBlocksMarker(const Layer<GvdVoxel>& layer, double scale) {
  return makeBlocksMarkerImpl(layer, scale);
}

std_msgs::ColorRGBA makeGvdColor(const GvdVisualizerConfig& config,
                                 const ColormapConfig& colors,
                                 double distance,
                                 uint8_t num_basis_points) {
  double ratio;
  double alpha;
  double alpha_diff = config.gvd_alpha - config.gvd_min_alpha;
  switch (static_cast<GvdVisualizationMode>(config.gvd_mode)) {
    case GvdVisualizationMode::BASIS_POINTS:
      ratio = computeRatio(static_cast<double>(config.min_num_basis),
                           static_cast<double>(config.max_num_basis),
                           static_cast<double>(num_basis_points));
      alpha = config.gvd_min_alpha + ratio * alpha_diff;
      break;
    case GvdVisualizationMode::DISTANCE:
    case GvdVisualizationMode::DEFAULT:
    default:
      ratio = computeRatio(config.gvd_min_distance, config.gvd_max_distance, distance);
      alpha = config.gvd_alpha;
      break;
  }

  NodeColor color = dsg_utils::interpolateColorMap(colors, ratio);
  return dsg_utils::makeColorMsg(color, alpha);
}

using EdgeMap = std::unordered_map<uint64_t, std::unordered_set<uint64_t>>;

std::unordered_set<uint64_t>& getNodeSet(EdgeMap& edge_map, uint64_t node) {
  auto iter = edge_map.find(node);
  if (iter == edge_map.end()) {
    iter = edge_map.emplace(node, std::unordered_set<uint64_t>()).first;
  }
  return iter->second;
}

MarkerArray makeGvdGraphMarkers(const GvdGraph& graph,
                                const GvdVisualizerConfig& config,
                                const ColormapConfig& colors,
                                const std::string& ns,
                                size_t marker_id) {
  MarkerArray marker;
  if (graph.empty()) {
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
  for (const auto& id_node_pair : graph.nodes()) {
    geometry_msgs::Point node_centroid;
    tf2::convert(id_node_pair.second.position, node_centroid);
    nodes.points.push_back(node_centroid);
    nodes.colors.push_back(makeGvdColor(config,
                                        colors,
                                        id_node_pair.second.distance,
                                        id_node_pair.second.num_basis_points));

    auto& curr_seen = getNodeSet(seen_edges, id_node_pair.first);
    for (const auto sibling : id_node_pair.second.siblings) {
      if (curr_seen.count(sibling)) {
        continue;
      }

      curr_seen.insert(sibling);
      getNodeSet(seen_edges, sibling).insert(id_node_pair.first);

      edges.points.push_back(nodes.points.back());
      edges.colors.push_back(nodes.colors.back());

      const auto& other = *graph.getNode(sibling);
      geometry_msgs::Point neighbor_centroid;
      tf2::convert(other.position, neighbor_centroid);
      edges.points.push_back(neighbor_centroid);
      edges.colors.push_back(
          makeGvdColor(config, colors, other.distance, other.num_basis_points));
    }
  }

  return marker;
}

size_t fillColors(const CompressedNodeMap& clusters,
                  std::map<uint64_t, size_t>& colors) {
  for (const auto& id_node_pair : clusters) {
    size_t max_color = 0;
    std::set<size_t> seen_colors;
    for (const auto sibling : id_node_pair.second.siblings) {
      const auto iter = colors.find(sibling);
      if (iter == colors.end()) {
        continue;
      }

      seen_colors.insert(iter->second);
      if (iter->second > max_color) {
        max_color = iter->second;
      }
    }

    if (seen_colors.empty()) {
      colors[id_node_pair.first] = 0;
      continue;
    }

    bool found_color = false;
    for (size_t i = 0; i < max_color; ++i) {
      if (!seen_colors.count(i)) {
        colors[id_node_pair.first] = i;
        found_color = true;
        break;
      }
    }

    if (found_color) {
      continue;
    }

    colors[id_node_pair.first] = max_color + 1;
  }

  size_t num_colors = 0;
  for (const auto& id_color_pair : colors) {
    if (id_color_pair.second > num_colors) {
      num_colors = id_color_pair.second;
    }
  }

  return num_colors + 1;
}

MarkerArray showGvdClusters(const GvdGraph& graph,
                            const CompressedNodeMap& clusters,
                            const std::unordered_map<uint64_t, uint64_t>& remapping,
                            const GvdVisualizerConfig& config,
                            const ColormapConfig& colormap,
                            const std::string& ns,
                            size_t marker_id) {
  MarkerArray marker;
  if (graph.empty()) {
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

  std::map<uint64_t, size_t> color_mapping;
  const size_t num_colors = fillColors(clusters, color_mapping);
  std::vector<std_msgs::ColorRGBA> colors;
  for (size_t i = 0; i < num_colors; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(num_colors);
    const auto color = dsg_utils::interpolateColorMap(colormap, ratio);
    colors.push_back(dsg_utils::makeColorMsg(color, config.gvd_alpha));
  }

  EdgeMap seen_edges;
  for (const auto& id_node_pair : graph.nodes()) {
    geometry_msgs::Point node_centroid;
    tf2::convert(id_node_pair.second.position, node_centroid);
    nodes.points.push_back(node_centroid);
    if (remapping.count(id_node_pair.first)) {
      const auto cluster_id = remapping.at(id_node_pair.first);
      const auto& cluster_color = colors.at(color_mapping.at(cluster_id));
      nodes.colors.push_back(cluster_color);
    } else {
      nodes.colors.push_back(
          dsg_utils::makeColorMsg(NodeColor(0, 0, 0), config.gvd_alpha));
    }

    auto& curr_seen = getNodeSet(seen_edges, id_node_pair.first);
    for (const auto sibling : id_node_pair.second.siblings) {
      if (curr_seen.count(sibling)) {
        continue;
      }

      curr_seen.insert(sibling);
      getNodeSet(seen_edges, sibling).insert(id_node_pair.first);

      edges.points.push_back(nodes.points.back());
      edges.colors.push_back(nodes.colors.back());

      const auto& other = *graph.getNode(sibling);
      geometry_msgs::Point neighbor_centroid;
      tf2::convert(other.position, neighbor_centroid);
      edges.points.push_back(neighbor_centroid);

      if (remapping.count(sibling)) {
        const auto& neighbor_cluster = remapping.at(sibling);
        const auto& neighbor_color = colors.at(color_mapping.at(neighbor_cluster));
        edges.colors.push_back(neighbor_color);
      } else {
        edges.colors.push_back(
            dsg_utils::makeColorMsg(NodeColor(0, 0, 0), config.gvd_alpha));
      }
    }
  }

  return marker;
}

MarkerArray makePlaceSpheres(const std_msgs::Header& header,
                             const SceneGraphLayer& layer,
                             const std::string& ns,
                             double alpha) {
  MarkerArray spheres;
  size_t id = 0;
  for (const auto& id_node_pair : layer.nodes()) {
    const auto& attrs = id_node_pair.second->attributes<PlaceNodeAttributes>();

    Marker marker;
    marker.header = header;
    marker.type = Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id;
    marker.ns = ns;

    marker.scale.x = 2 * attrs.distance;
    marker.scale.y = 2 * attrs.distance;
    marker.scale.z = 2 * attrs.distance;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    tf2::convert(id_node_pair.second->attributes().position, marker.pose.position);

    NodeColor desired_color(255, 0, 0);
    marker.color = dsg_utils::makeColorMsg(desired_color, alpha);
    spheres.markers.push_back(marker);
    ++id;
  }

  return spheres;
}

}  // namespace hydra
