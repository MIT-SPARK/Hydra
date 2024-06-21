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

namespace hydra::utils {

void getPlace2dAndNeighors(const SceneGraphLayer& places_layer,
                           std::vector<std::pair<NodeId, Place2d>>& place_2ds,
                           std::map<NodeId, std::set<NodeId>>& node_neighbors) {
  for (auto& id_node_pair : places_layer.nodes()) {
    auto& attrs = id_node_pair.second->attributes<Place2dNodeAttributes>();
    if (attrs.need_finish_merge) {
      Place2d p;
      p.indices.insert(p.indices.end(),
                       attrs.pcl_mesh_connections.begin(),
                       attrs.pcl_mesh_connections.end());
      place_2ds.push_back(std::pair(id_node_pair.first, p));
      node_neighbors.insert({id_node_pair.first, id_node_pair.second->siblings()});
    }
  }
}

void getNecessaryUpdates(
    const spark_dsg::Mesh& mesh,
    size_t min_points,
    double min_size,
    double connection_ellipse_scale_factor,
    std::vector<std::pair<NodeId, Place2d>>& place_2ds,
    std::vector<std::pair<NodeId, Place2d>>& nodes_to_update,
    std::vector<std::pair<NodeId, std::vector<Place2d>>>& nodes_to_add) {
  for (auto& id_place_pair : place_2ds) {
    addRectInfo(mesh.points, connection_ellipse_scale_factor, id_place_pair.second);

    if (id_place_pair.second.indices.size() > min_points &&
        id_place_pair.second.cut_plane.norm() > min_size) {
      std::vector<Place2d> split_places =
          decomposePlace(mesh.points,
                         id_place_pair.second,
                         min_size,
                         min_points,
                         connection_ellipse_scale_factor);
      for (Place2d& p : split_places) {
        addBoundaryInfo(mesh.points, p);
      }
      nodes_to_add.push_back(std::pair(id_place_pair.first, split_places));
    } else {
      addBoundaryInfo(mesh.points, id_place_pair.second);
      size_t min_ix = SIZE_MAX;
      size_t max_ix = 0;
      for (auto midx : id_place_pair.second.indices) {
        min_ix = std::min(min_ix, midx);
        max_ix = std::max(max_ix, midx);
      }
      id_place_pair.second.min_mesh_index = min_ix;
      id_place_pair.second.max_mesh_index = max_ix;
      nodes_to_update.push_back(id_place_pair);
    }
  }
}

std::map<std::tuple<size_t, size_t, size_t, size_t>, double> buildEdgeMap(
    const std::vector<std::pair<NodeId, std::vector<Place2d>>>& nodes_to_add,
    double place_overlap_threshold,
    double place_neighbor_z_diff) {
  std::map<std::tuple<size_t, size_t, size_t, size_t>, double> edge_map;
  // compute which pairs of new nodes will need to have an edge added
  // Currently checks all split merged nodes. Maybe just check neighbors-of-neighbors?
  for (size_t og_ix = 0; og_ix < nodes_to_add.size(); ++og_ix) {
    for (size_t split_ix = 0; split_ix < nodes_to_add.at(og_ix).second.size();
         ++split_ix) {
      Place2d p1 = nodes_to_add.at(og_ix).second.at(split_ix);
      for (size_t og_jx = 0; og_jx <= og_ix; ++og_jx) {
        for (size_t split_jx = 0; split_jx < nodes_to_add.at(og_jx).second.size();
             ++split_jx) {
          Place2d p2 = nodes_to_add.at(og_jx).second.at(split_jx);
          double weight;
          bool connected = shouldAddPlaceConnection(
              p1, p2, place_overlap_threshold, place_neighbor_z_diff, weight);
          edge_map.insert({std::make_tuple(og_ix, split_ix, og_jx, split_jx),
                           connected ? weight : 0});
        }
      }
    }
  }
  return edge_map;
}

void updateExistingNodes(const std::vector<std::pair<NodeId, Place2d>>& nodes_to_update,
                         DynamicSceneGraph& graph) {
  for (auto& id_place_pair : nodes_to_update) {
    auto& attrs =
        graph.getNode(id_place_pair.first).attributes<Place2dNodeAttributes>();
    Place2d place = id_place_pair.second;
    pcl::PointXYZ centroid;
    place.centroid.get(centroid);
    attrs.position << centroid.x, centroid.y, centroid.z;

    attrs.boundary = place.boundary;
    attrs.pcl_boundary_connections.clear();
    attrs.pcl_boundary_connections.insert(attrs.pcl_boundary_connections.begin(),
                                          place.boundary_indices.begin(),
                                          place.boundary_indices.end());
    attrs.ellipse_matrix_compress = place.ellipse_matrix_compress;
    attrs.ellipse_matrix_expand = place.ellipse_matrix_expand;
    attrs.ellipse_centroid(0) = place.ellipse_centroid(0);
    attrs.ellipse_centroid(1) = place.ellipse_centroid(1);
    attrs.ellipse_centroid(2) = centroid.z;
    attrs.pcl_min_index = place.min_mesh_index;
    attrs.pcl_max_index = place.max_mesh_index;

    attrs.pcl_mesh_connections.clear();
    attrs.pcl_mesh_connections.insert(
        attrs.pcl_mesh_connections.begin(), place.indices.begin(), place.indices.end());

    attrs.need_finish_merge = false;
  }
}

NodeSymbol insertNewNodes(
    const std::vector<std::pair<NodeId, std::vector<Place2d>>>& nodes_to_add,
    const double place_overlap_threshold,
    const double place_max_neighbor_z_diff,
    NodeSymbol next_node_symbol,
    DynamicSceneGraph& graph,
    std::map<std::tuple<size_t, size_t>, NodeId>& new_id_map) {
  // insert new nodes that needed to be split
  int og_ix = 0;
  for (auto& id_places_pair : nodes_to_add) {
    const auto& node = graph.getNode(id_places_pair.first);
    auto& attrs_og = node.attributes<Place2dNodeAttributes>();

    int split_ix = 0;
    for (Place2d place : id_places_pair.second) {
      NodeSymbol node_id_for_place = next_node_symbol++;
      Place2dNodeAttributes::Ptr attrs = std::make_unique<Place2dNodeAttributes>();
      pcl::PointXYZ centroid;
      place.centroid.get(centroid);
      attrs->position << centroid.x, centroid.y, centroid.z;
      attrs->is_active = false;

      attrs->semantic_label = attrs_og.semantic_label;
      attrs->name = NodeSymbol(node_id_for_place).getLabel();
      attrs->boundary = place.boundary;
      attrs->pcl_boundary_connections.insert(attrs->pcl_boundary_connections.begin(),
                                             place.boundary_indices.begin(),
                                             place.boundary_indices.end());
      attrs->ellipse_matrix_compress = place.ellipse_matrix_compress;
      attrs->ellipse_matrix_expand = place.ellipse_matrix_expand;
      attrs->ellipse_centroid(0) = place.ellipse_centroid(0);
      attrs->ellipse_centroid(1) = place.ellipse_centroid(1);
      attrs->ellipse_centroid(2) = centroid.z;
      attrs->pcl_min_index = place.min_mesh_index;
      attrs->pcl_max_index = place.max_mesh_index;

      attrs->pcl_mesh_connections.insert(attrs->pcl_mesh_connections.begin(),
                                         place.indices.begin(),
                                         place.indices.end());
      attrs->color = attrs_og.color;

      attrs->has_active_mesh_indices = attrs_og.has_active_mesh_indices;
      attrs->need_cleanup_splitting = true;
      attrs->need_finish_merge = false;

      graph.emplaceNode(DsgLayers::MESH_PLACES, node_id_for_place, std::move(attrs));
      new_id_map.insert({std::make_tuple(og_ix, split_ix), node_id_for_place});

      auto& attrs_added =
          graph.getNode(node_id_for_place).attributes<Place2dNodeAttributes>();
      // Check for connections between this place and the original place's siblings
      std::vector<std::pair<NodeId, NodeId>> edges_to_add;
      for (NodeId neighbor : node.siblings()) {
        EdgeAttributes ea;
        Place2dNodeAttributes attrs_neighbor =
            graph.getNode(neighbor).attributes<Place2dNodeAttributes>();

        if (!attrs_neighbor.need_finish_merge &&
            shouldAddPlaceConnection(attrs_added,
                                     attrs_neighbor,
                                     place_overlap_threshold,
                                     place_max_neighbor_z_diff,
                                     ea)) {
          edges_to_add.push_back({neighbor, node_id_for_place});
        }
      }

      for (const auto& [source, target] : edges_to_add) {
        graph.insertEdge(source, target);  // TODO add edge attributes
      }

      ++split_ix;
    }

    ++og_ix;
    graph.removeNode(id_places_pair.first);
  }
  return next_node_symbol;
}

void addNewNodeEdges(
    const std::vector<std::pair<NodeId, std::vector<Place2d>>> nodes_to_add,
    const std::map<std::tuple<size_t, size_t, size_t, size_t>, double> edge_map,
    const std::map<std::tuple<size_t, size_t>, NodeId> new_id_map,
    DynamicSceneGraph& graph) {
  // It looks like this loop would take a long time, but all of these sizes should be
  // extremely small (e.g. 2ish)
  for (size_t og_ix = 0; og_ix < nodes_to_add.size(); ++og_ix) {
    for (size_t split_ix = 0; split_ix < nodes_to_add.at(og_ix).second.size();
         ++split_ix) {
      Place2d p1 = nodes_to_add.at(og_ix).second.at(split_ix);
      for (size_t og_jx = 0; og_jx <= og_ix; ++og_jx) {
        for (size_t split_jx = 0; split_jx < nodes_to_add.at(og_jx).second.size();
             ++split_jx) {
          double weight =
              edge_map.at(std::make_tuple(og_ix, split_ix, og_jx, split_jx));
          if (weight > 0) {
            NodeId n1 = new_id_map.at(std::make_tuple(og_ix, split_ix));
            NodeId n2 = new_id_map.at(std::make_tuple(og_jx, split_jx));
            EdgeAttributes ea;
            ea.weight = weight;
            ea.weighted = true;
            graph.insertEdge(n1, n2, ea.clone());
          }
        }
      }
    }
  }
}

void reallocateMeshPoints(const std::vector<Place2d::PointT>& points,
                          Place2dNodeAttributes& attrs1,
                          Place2dNodeAttributes& attrs2) {
  Eigen::Vector2d delta = attrs2.position.head(2) - attrs1.position.head(2);
  Eigen::Vector2d d = attrs1.position.head(2) + delta / 2;

  std::vector<Place2d::Index> p1_new_indices;
  std::vector<Place2d::Index> p2_new_indices;

  for (auto midx : attrs1.pcl_mesh_connections) {
    // pcl::PointXYZRGBA pclp = points.at(midx);
    // Eigen::Vector2d p(pclp.x, pclp.y);
    Eigen::Vector2d p = points.at(midx).head(2).cast<double>();
    if ((p - d).dot(delta) > 0) {
      p2_new_indices.push_back(midx);
    } else {
      p1_new_indices.push_back(midx);
    }
  }
  for (auto midx : attrs2.pcl_mesh_connections) {
    // pcl::PointXYZRGBA pclp = points.at(midx);
    // Eigen::Vector2d p(pclp.x, pclp.y);
    Eigen::Vector2d p = points.at(midx).head(2).cast<double>();
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
  // place's activeness for a small speed improvement, but not sure how much it matters
  attrs1.has_active_mesh_indices =
      attrs1.has_active_mesh_indices || attrs2.has_active_mesh_indices;
  attrs2.has_active_mesh_indices =
      attrs1.has_active_mesh_indices || attrs2.has_active_mesh_indices;
}

}  // namespace hydra::utils
