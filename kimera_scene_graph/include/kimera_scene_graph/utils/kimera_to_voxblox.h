#pragma once

#include <voxblox_ros/mesh_vis.h>
#include <voxblox_skeleton/skeleton.h>

#include "kimera_scene_graph/scene_node.h"
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
  node_color[0] = color.r;
  node_color[1] = color.g;
  node_color[2] = color.b;
  return node_color;
}

}

}  // namespace kimera
