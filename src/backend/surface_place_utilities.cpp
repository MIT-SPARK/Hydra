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
#include "hydra/backend/surface_place_utilities.h"

#include <glog/logging.h>

namespace hydra::utils {

void getPlace2dAndNeighors(const SceneGraphLayer& places_layer,
                           std::vector<std::pair<NodeId, Place2d>>& place_2ds,
                           std::map<NodeId, std::set<NodeId>>& node_neighbors) {
  for (auto& [node_id, node] : places_layer.nodes()) {
    auto attrs = node->tryAttributes<Place2dNodeAttributes>();
    if (!attrs) {
      continue;
    }

    if (attrs->need_finish_merge) {
      Place2d p;
      p.indices.insert(p.indices.end(),
                       attrs->pcl_mesh_connections.begin(),
                       attrs->pcl_mesh_connections.end());
      place_2ds.push_back(std::pair(node_id, p));
      node_neighbors.insert({node_id, node->siblings()});
    }
  }
}

void computeAttributeUpdates(const spark_dsg::Mesh& mesh,
                             const double connection_ellipse_scale_factor,
                             std::vector<std::pair<NodeId, Place2d>>& place_2ds,
                             std::vector<std::pair<NodeId, Place2d>>& nodes_to_update) {
  for (auto& [node_id, place] : place_2ds) {
    addRectInfo(mesh, connection_ellipse_scale_factor, place);
    addBoundaryInfo(mesh, place);
    place.updateIndexBounds();
    nodes_to_update.push_back({node_id, place});
  }
}

// TODO(nathan) these get dropped when we rebase on the active DSG stuff
[[deprecated]] void getNecessaryUpdates(
    const spark_dsg::Mesh&,
    size_t,
    double,
    double,
    std::vector<std::pair<NodeId, Place2d>>&,
    std::vector<std::pair<NodeId, Place2d>>&,
    std::vector<std::pair<NodeId, std::vector<Place2d>>>&) {}

[[deprecated]] std::map<std::tuple<size_t, size_t, size_t, size_t>, double>
buildEdgeMap(const std::vector<std::pair<NodeId, std::vector<Place2d>>>&,
             double,
             double) {
  return {};
}

[[deprecated]] NodeSymbol insertNewNodes(
    const std::vector<std::pair<NodeId, std::vector<Place2d>>>&,
    const double,
    const double,
    NodeSymbol,
    DynamicSceneGraph&,
    std::map<std::tuple<size_t, size_t>, NodeId>&) {
  return 0;
}

[[deprecated]] void addNewNodeEdges(
    const std::vector<std::pair<NodeId, std::vector<Place2d>>>,
    const std::map<std::tuple<size_t, size_t, size_t, size_t>, double>,
    const std::map<std::tuple<size_t, size_t>, NodeId>,
    DynamicSceneGraph&) {}

void updateExistingNodes(const std::vector<std::pair<NodeId, Place2d>>& nodes_to_update,
                         DynamicSceneGraph& graph) {
  for (const auto& [node_id, place] : nodes_to_update) {
    auto& attrs = graph.getNode(node_id).attributes<Place2dNodeAttributes>();
    attrs.position = place.centroid.cast<double>();

    attrs.boundary = place.boundary;
    attrs.pcl_boundary_connections.clear();
    attrs.pcl_boundary_connections.insert(attrs.pcl_boundary_connections.begin(),
                                          place.boundary_indices.begin(),
                                          place.boundary_indices.end());
    attrs.ellipse_matrix_compress = place.ellipse_matrix_compress;
    attrs.ellipse_matrix_expand = place.ellipse_matrix_expand;
    attrs.ellipse_centroid(0) = place.ellipse_centroid(0);
    attrs.ellipse_centroid(1) = place.ellipse_centroid(1);
    attrs.ellipse_centroid(2) = attrs.position.z();
    attrs.pcl_min_index = place.min_mesh_index;
    attrs.pcl_max_index = place.max_mesh_index;

    attrs.pcl_mesh_connections.clear();
    attrs.pcl_mesh_connections.insert(
        attrs.pcl_mesh_connections.begin(), place.indices.begin(), place.indices.end());

    attrs.need_finish_merge = false;
  }
}

void reallocateMeshPoints(const spark_dsg::Mesh& mesh,
                          Place2dNodeAttributes& attrs1,
                          Place2dNodeAttributes& attrs2) {
  Eigen::Vector2d delta = attrs2.position.head(2) - attrs1.position.head(2);
  Eigen::Vector2d d = attrs1.position.head(2) + delta / 2;

  std::vector<size_t> p1_new_indices;
  std::vector<size_t> p2_new_indices;

  for (auto midx : attrs1.pcl_mesh_connections) {
    Eigen::Vector2d p = mesh.points.at(midx).head(2).cast<double>();
    if ((p - d).dot(delta) > 0) {
      p2_new_indices.push_back(midx);
    } else {
      p1_new_indices.push_back(midx);
    }
  }
  for (auto midx : attrs2.pcl_mesh_connections) {
    Eigen::Vector2d p = mesh.points.at(midx).head(2).cast<double>();
    if ((p - d).dot(delta) > 0) {
      p2_new_indices.push_back(midx);
    } else {
      p1_new_indices.push_back(midx);
    }
  }

  if (p1_new_indices.size() == 0 || p2_new_indices.size() == 0) {
    LOG(ERROR) << "Reallocating mesh points would make empty place. Skippings.";
    return;
  }

  std::sort(p1_new_indices.begin(), p1_new_indices.end());
  auto last = std::unique(p1_new_indices.begin(), p1_new_indices.end());
  p1_new_indices.erase(last, p1_new_indices.end());

  std::sort(p2_new_indices.begin(), p2_new_indices.end());
  last = std::unique(p2_new_indices.begin(), p2_new_indices.end());
  p2_new_indices.erase(last, p2_new_indices.end());

  attrs1.pcl_mesh_connections = p1_new_indices;
  attrs2.pcl_mesh_connections = p2_new_indices;

  // Say there are active mesh indices if either involved node has them.
  // In theory we could actually check if any of the reallocated vertices changes a
  // place's activeness for a small speed improvement, but not sure how much it
  // matters
  attrs1.has_active_mesh_indices =
      attrs1.has_active_mesh_indices || attrs2.has_active_mesh_indices;
  attrs2.has_active_mesh_indices =
      attrs1.has_active_mesh_indices || attrs2.has_active_mesh_indices;
}

}  // namespace hydra::utils
