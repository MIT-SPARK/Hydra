#pragma once

#include <string>

#include "kimera_semantics/common.h"

#include <visualization_msgs/Marker.h>
#include <algorithm>
#include <limits>

#include <glog/logging.h>

#include <voxblox_ros/mesh_vis.h>
#include <voxblox/mesh/mesh.h>

namespace kimera {

/**
 * @brief fillMarkerWithMesh Get a ROS viz msg marker with the mesh, and
 * optionally add a shift in the z axis (for the scene graph visualization).
 * @param mesh Actual semantic mesh
 * @param color_mode How to color the mesh
 * @param marker Output generated viz msg marker
 * @param z_shift Shift in the z axis
 */
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

}  // namespace kimera
