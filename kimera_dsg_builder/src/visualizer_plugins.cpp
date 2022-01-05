#include "kimera_dsg_builder/visualizer_plugins.h"
#include "kimera_dsg_builder/timing_utilities.h"

#include <kimera_dsg_visualizer/colormap_utils.h>
#include <kimera_dsg_visualizer/visualizer_utils.h>

#include <mesh_msgs/TriangleMesh.h>
#include <mesh_msgs/TriangleMeshStamped.h>

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

  invalid_color << 64, 235, 52;

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

#undef READ_PARAM

PgmoMeshPlugin::PgmoMeshPlugin(const ros::NodeHandle& nh, const std::string& name)
    : DsgVisualizerPlugin(nh, name) {
  // namespacing gives us a reasonable topic
  mesh_pub_ = nh_.advertise<mesh_msgs::TriangleMeshStamped>("", 1, true);
}

void PgmoMeshPlugin::draw(const std_msgs::Header& header,
                          const DynamicSceneGraph& graph) {
  if (!graph.hasMesh()) {
    return;
  }

  if (!graph.getMeshVertices()->size()) {
    return;
  }

  mesh_msgs::TriangleMeshStamped msg;
  msg.header = header;

  // vertices and meshes are guaranteed to not be null (from hasMesh)
  msg.mesh = kimera_pgmo::PolygonMeshToTriangleMeshMsg(*graph.getMeshVertices(),
                                                       *graph.getMeshFaces());
  mesh_pub_.publish(msg);
}

void PgmoMeshPlugin::reset(const std_msgs::Header& header, const DynamicSceneGraph&) {
  mesh_msgs::TriangleMeshStamped msg;
  msg.header = header;
  mesh_pub_.publish(msg);
}

VoxbloxMeshPlugin::VoxbloxMeshPlugin(const ros::NodeHandle& nh, const std::string& name)
    : DsgVisualizerPlugin(nh, name) {
  // namespacing gives us a reasonable topic
  mesh_pub_ = nh_.advertise<voxblox_msgs::Mesh>("", 1, true);
}

voxblox::Point getPoint(const pcl::PointXYZRGBA& point) {
  voxblox::Point vox_point;
  vox_point << point.x, point.y, point.z;
  return vox_point;
}

voxblox::BlockIndex getBlockIndex(const voxblox::Point& point) {
  voxblox::BlockIndex index;
  index << std::floor(point.x()), std::floor(point.y()), std::floor(point.z());
  return index;
}

struct BestIndex {
  bool valid;
  voxblox::BlockIndex index;
};

bool vectorHasNegativeElement(const voxblox::Point& point) {
  return (point.array() < 0.0).any();
}

BestIndex getBestBlockIndex(const pcl::PointCloud<pcl::PointXYZRGBA>& cloud,
                            const std::vector<uint32_t>& indices) {
  const voxblox::Point point1 = getPoint(cloud.at(indices.at(0)));
  const voxblox::Point point2 = getPoint(cloud.at(indices.at(1)));
  const voxblox::Point point3 = getPoint(cloud.at(indices.at(2)));

  const voxblox::BlockIndex index1 = getBlockIndex(point1);
  const voxblox::BlockIndex index2 = getBlockIndex(point2);
  const voxblox::BlockIndex index3 = getBlockIndex(point3);

  // TODO(nathan) this is ugly, fix
  voxblox::BlockIndex best_index;
  best_index << std::min(index1.x(), std::min(index2.x(), index3.x())),
      std::min(index1.y(), std::min(index2.y(), index3.y())),
      std::min(index1.z(), std::min(index2.z(), index3.z()));

  const voxblox::Point norm1 = (point1 - best_index.cast<float>()) / 2.0;
  if (norm1.squaredNorm() > 1.0 || (norm1.array() < 0).any()) {
    LOG(ERROR) << "best index: " << best_index.transpose() << " is invalid";
    return {false, voxblox::BlockIndex::Zero()};
  }

  const voxblox::Point norm2 = (point2 - best_index.cast<float>()) / 2.0;
  if (norm2.squaredNorm() > 1.0 || (norm2.array() < 0).any()) {
    LOG(ERROR) << "best index: " << best_index.transpose() << " is invalid";
    return {false, voxblox::BlockIndex::Zero()};
  }

  const voxblox::Point norm3 = (point3 - best_index.cast<float>()) / 2.0;
  if (norm3.squaredNorm() > 1.0 || (norm3.array() < 0).any()) {
    LOG(ERROR) << "best index: " << best_index.transpose() << " is invalid";
    return {false, voxblox::BlockIndex::Zero()};
  }

  return {true, best_index};
}

void VoxbloxMeshPlugin::draw(const std_msgs::Header& header,
                             const DynamicSceneGraph& graph) {
  if (!graph.hasMesh()) {
    return;
  }

  voxblox_msgs::Mesh msg;
  msg.header = header;
  msg.block_edge_length = 1.0;

  voxblox::AnyIndexHashMapType<size_t>::type block_to_index;

  const auto& vertices = *graph.getMeshVertices();
  const auto& faces = *graph.getMeshFaces();

  curr_blocks_.clear();

  for (const auto& face : faces) {
    const auto block_idx_result = getBestBlockIndex(vertices, face.vertices);
    if (!block_idx_result.valid) {
      continue;
    }

    const auto block_idx = block_idx_result.index;
    if (!block_to_index.count(block_idx)) {
      block_to_index[block_idx] = msg.mesh_blocks.size();
      voxblox_msgs::MeshBlock block_msg;
      block_msg.index[0] = block_idx.x();
      block_msg.index[1] = block_idx.y();
      block_msg.index[2] = block_idx.z();
      msg.mesh_blocks.push_back(block_msg);
      curr_blocks_.push_back(block_idx);
    }

    auto& block = msg.mesh_blocks.at(block_to_index.at(block_idx));
    const voxblox::Point block_pos = block_idx.cast<float>();

    for (const auto& vertex_idx : face.vertices) {
      const auto& point = vertices.at(vertex_idx);
      voxblox::Point vox_point;
      vox_point << point.x, point.y, point.z;
      const voxblox::Point normalized_point = (vox_point - block_pos) / 2.0f;

      const uint16_t max_value = std::numeric_limits<uint16_t>::max();
      block.x.push_back(max_value * normalized_point.x());
      block.y.push_back(max_value * normalized_point.y());
      block.z.push_back(max_value * normalized_point.z());
      block.r.push_back(point.r);
      block.g.push_back(point.g);
      block.b.push_back(point.b);
    }
  }

  mesh_pub_.publish(msg);
}

void VoxbloxMeshPlugin::reset(const std_msgs::Header& header,
                              const DynamicSceneGraph&) {
  if (curr_blocks_.empty()) {
    return;
  }

  voxblox_msgs::Mesh msg;
  msg.header = header;
  msg.block_edge_length = 1.0;

  for (const auto& block_idx : curr_blocks_) {
    voxblox_msgs::MeshBlock block_msg;
    block_msg.index[0] = block_idx.x();
    block_msg.index[1] = block_idx.y();
    block_msg.index[2] = block_idx.z();
    msg.mesh_blocks.push_back(block_msg);
  }

  curr_blocks_.clear();
  mesh_pub_.publish(msg);
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
  if (!graph.hasLayer(KimeraDsgLayers::PLACES)) {
    return;
  }

  if (!graph.hasMesh()) {
    return;
  }

  const SceneGraphLayer& layer = *graph.getLayer(KimeraDsgLayers::PLACES);

  MinimumSpanningTreeInfo mst_info;
  {  // timing scope
    ScopedTimer timer("visualizer/mst_creation", header.stamp.toNSec(), true, 2, false);
    mst_info = getMinimumSpanningEdges(layer);
  }  // timing scope

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

}  // namespace kimera
