#pragma once

#include <string>

#include <kimera_semantics/common.h>

#include "kimera_scene_graph/common.h"

#include <voxblox_skeleton/skeleton.h>

namespace kimera {

inline ColorPointCloud::Ptr createPclCloudFromSkeleton(
    const vxb::SparseSkeletonGraph& sparse_skeleton_graph,
    std::map<int, int64_t>* cloud_to_graph_ids,
    std::vector<int64_t>* vertex_ids) {
  ColorPointCloud::Ptr skeleton_graph_cloud(new ColorPointCloud);
  // Get a list of all vertices
  std::vector<int64_t> vtx_ids;
  sparse_skeleton_graph.getAllVertexIds(&vtx_ids);
  skeleton_graph_cloud->resize(vtx_ids.size());
  size_t i = 0u;
  for (const int64_t& vertex_id : vtx_ids) {
    const vxb::SkeletonVertex& vertex =
        sparse_skeleton_graph.getVertex(vertex_id);
    ColorPoint point;
    point.x = vertex.point.x();
    point.y = vertex.point.y();
    point.z = vertex.point.z();
    skeleton_graph_cloud->at(i) = point;
    if (cloud_to_graph_ids) (*cloud_to_graph_ids)[i] = vertex_id;
    ++i;
  }
  if (vertex_ids) *vertex_ids = vtx_ids;
  return skeleton_graph_cloud;
}

inline vxb::Point pclPointToVxbPoint(const Point& pcl_point) {
  vxb::Point vxb_point;
  vxb_point[0] = pcl_point.x;
  vxb_point[1] = pcl_point.y;
  vxb_point[2] = pcl_point.z;
  return vxb_point;
}

}  // namespace kimera
