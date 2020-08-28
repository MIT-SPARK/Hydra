#pragma once

#include <voxblox_ros/mesh_vis.h>
#include <voxblox_skeleton/skeleton.h>

#include "kimera_scene_graph/scene_graph_layer.h"
#include "kimera_scene_graph/utils/voxblox_to_pcl.h"
#include "kimera_semantics/common.h"

#include <glog/logging.h>

namespace kimera {

namespace utils {

inline void fillMarkerWithMesh(const vxb::Mesh& mesh,
                               const vxb::ColorMode& color_mode,
                               visualization_msgs::Marker* marker,
                               const float& z_shift = 0.0) {
  CHECK_NOTNULL(marker);
  marker->header.stamp = ros::Time::now();
  marker->ns = "mesh";
  marker->scale.x = 1;
  marker->scale.y = 1;
  marker->scale.z = 1;
  marker->pose.orientation.x = 0;
  marker->pose.orientation.y = 0;
  marker->pose.orientation.z = 0;
  marker->pose.orientation.w = 1;
  marker->type = visualization_msgs::Marker::TRIANGLE_LIST;

  if (!mesh.hasVertices()) {
    return;
  }
  // Check that we can actually do the color stuff.
  if (color_mode == vxb::kColor || color_mode == vxb::kLambertColor) {
    CHECK(mesh.hasColors());
  }
  if (color_mode == vxb::kNormals || color_mode == vxb::kLambert ||
      color_mode == vxb::kLambertColor) {
    CHECK(mesh.hasNormals());
  }

  for (size_t i = 0u; i < mesh.vertices.size(); i++) {
    geometry_msgs::Point point_msg;
    tf::pointEigenToMsg(mesh.vertices[i].cast<double>(), point_msg);
    point_msg.z += z_shift;
    marker->points.push_back(point_msg);
    marker->colors.push_back(vxb::getVertexColor(mesh, color_mode, i));
  }
}

inline NodeColor vxbColorToNodeColor(const vxb::Color& color) {
  NodeColor node_color;
  node_color.r = color.r;
  node_color.g = color.g;
  node_color.b = color.b;
  return node_color;
}

inline void convertLayerToSkeleton(const SceneGraphLayer& scene_graph_layer,
                                   vxb::SparseSkeletonGraph* skeleton) {
  CHECK_NOTNULL(skeleton);
  skeleton->clear();
  // Keep track of ids btw vxb and scene graph
  std::map<NodeId, int64_t> node_id_to_vxb_id;

  // Add vertices
  const auto& node_map_ = scene_graph_layer.getNodeIdMap();
  vxb::SkeletonVertex skeleton_vtx;
  for (const std::pair<NodeId, SceneGraphNode>& vtx : node_map_) {
    CHECK_EQ(vtx.first, vtx.second.node_id_);
    skeleton_vtx.room_id = vtx.second.parent_edge_.start_node_id_;
    // This is the ESDF distance (set to random).
    skeleton_vtx.distance = 1u;
    // Set all to be in the same subgraph... Perhaps we should be able to
    // set different subgraph ids depending on whether the node is isolated.
    skeleton_vtx.subgraph_id = 1u;
    // No need to update this, it will be automatically done once we add the
    // edges (and we will not have inter-layer edges).
    DCHECK_EQ(skeleton_vtx.edge_list.size(), 0u);
    skeleton_vtx.point = kimeraPointToVxbPoint(vtx.second.attributes_.position_);
    // Vxb sets the vertex id, so it will be overwritten...
    // skeleton_vtx.vertex_id = vtx.second.node_id_;
    const auto& vxb_id = skeleton->addVertex(skeleton_vtx);
    node_id_to_vxb_id[vtx.second.node_id_] = vxb_id;
  }
  CHECK_EQ(skeleton->getVertexMap().size(),
           scene_graph_layer.getNumberOfNodes())
      << "The skeleton and the scene graph layer should have the same "
         "number of nodes!";

  // Add edges
  // TODO(Toni): perhaps check for self-edges?
  const auto& intra_layer_edge_map_ = scene_graph_layer.getEdgeIdMap();
  vxb::SkeletonEdge skeleton_edge;
  for (const std::pair<EdgeId, SceneGraphEdge>& edge : intra_layer_edge_map_) {
    CHECK_EQ(edge.first, edge.second.edge_id_);
    CHECK(edge.second.start_layer_id_ == scene_graph_layer.getLayerId());
    CHECK(edge.second.end_layer_id_ == scene_graph_layer.getLayerId());
    // Vxb sets the edge id, so it will be overwritten...
    // skeleton_edge.edge_id = edge.second.node_id_;
    skeleton_edge.start_vertex =
        node_id_to_vxb_id.at(edge.second.start_node_id_);
    skeleton_edge.end_vertex = node_id_to_vxb_id.at(edge.second.end_node_id_);
    skeleton_edge.start_distance = 1u;
    skeleton_edge.end_distance = 1u;
    // These ar filled when calling add Edge
    // skeleton_edge.start_point;
    // skeleton_edge.end_point;
    skeleton->addEdge(skeleton_edge);
  }
  CHECK_EQ(skeleton->getEdgeMap().size(), scene_graph_layer.getNumberOfEdges())
      << "The skeleton and the scene graph layer should have the same "
         "number of edges!";
}

}  // namespace utils

}  // namespace kimera
