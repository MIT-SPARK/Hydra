#include "kimera_batch_dsg/wall_finder.h"
#include "kimera_batch_dsg/common.h"
#include "kimera_batch_dsg/pcl_conversion.h"
#include "kimera_batch_dsg/pcl_types.h"

#include <kimera_dsg/node_attributes.h>
#include <pcl/search/kdtree.h>

namespace kimera {

using NodeColor = SemanticNodeAttributes::ColorVector;
using Node = SceneGraph::Node;

pcl::PolygonMesh::Ptr findWalls(const SubMesh& mesh, const SceneGraph& scene_graph) {
  // Create cloud of the places so we can do nearest neighbor queries
  CHECK(scene_graph.hasLayer(KimeraDsgLayers::PLACES));
  const SceneGraphLayer& layer = *(scene_graph.getLayer(KimeraDsgLayers::PLACES));
  if (layer.nodes().empty()) {
    LOG(WARNING) << "Places layer empty: cannot cluster wall vertices to rooms";
    return nullptr;
  }

  PclLayer<ColorPointCloud> pcl_layer = convertLayerToPcl<ColorPointCloud>(layer);

  Eigen::MatrixXd normals = getNormals(*mesh.mesh, *mesh.vertices);
  ColorPointCloud segmented_vertices(*mesh.vertices);

  pcl::KdTreeFLANN<ColorPoint> kdtree;
  kdtree.setInputCloud(pcl_layer.cloud);

  static constexpr int K = 30;
  std::vector<int> nn_indices(K);
  std::vector<float> nn_squared_distances(K);

  size_t unlabeled_vertices = 0;
  size_t default_vertices = 0;
  size_t labeled_vertices = 0;
  for (size_t idx = 0u; idx < segmented_vertices.size(); idx++) {
    int nn_size = kdtree.nearestKSearch(
        segmented_vertices.at(idx), K, nn_indices, nn_squared_distances);
    if (nn_size == 0) {
      LOG(ERROR) << "No nearest place to wall vertex found!";
      unlabeled_vertices++;
      continue;
    }

    CHECK_GT(nn_indices.size(), 0u);
    CHECK_GT(nn_squared_distances.size(), 0u);
    CHECK_EQ(nn_indices.size(), nn_squared_distances.size());

    std::map<NodeId, double> room_id_votes;

    size_t num_far_away = 0u;
    size_t num_outside = 0u;
    size_t num_valid = 0u;
    for (int i = 0u; i < nn_size; i++) {
      // Avoid voters that are very far away
      const double kMaxSquaredDistance = std::pow(5.0, 2);
      const double nn_squared_distance = nn_squared_distances.at(i);
      if (nn_squared_distance >= kMaxSquaredDistance) {
        num_far_away++;
        continue;
      }

      if (nn_squared_distance <= 0.0) {
        LOG(ERROR) << "Invalid NN distance: " << nn_squared_distance;
        continue;
      }

      CHECK_LT(nn_indices.at(i), static_cast<int>(pcl_layer.cloud_to_layer_ids.size()));
      auto nearest_node_idx = pcl_layer.cloud_to_layer_ids[nn_indices.at(i)];
      const Node& nearest_node = *(scene_graph.getNode(nearest_node_idx));
      Eigen::Vector3d node_pos = nearest_node.attributes().position;

      Eigen::Vector3d direction = node_pos - toEigen(segmented_vertices.at(idx));
      double cos_normal_angle = normals.block<3, 1>(idx, 0).dot(direction);
      if (cos_normal_angle <= 0.0) {
        num_outside++;
        continue;
      }

      if (!nearest_node.hasParent()) {
        continue;
      }

      num_valid++;

      NodeId room_id = *(nearest_node.getParent());
      if (!room_id_votes.count(room_id)) {
        room_id_votes[room_id] = 0;
      }

      // Weight the vote by the dot product
      room_id_votes[room_id] += cos_normal_angle / nn_squared_distance;
    }

    ColorPoint& segmented_point = segmented_vertices.at(idx);

    NodeId max_room_id = 0;
    double max_room_id_votes = 0.0;
    for (const auto& kv : room_id_votes) {
      if (kv.second > max_room_id_votes) {
        max_room_id = kv.first;
        max_room_id_votes = kv.second;
      }
    }

    if (room_id_votes.empty() || !scene_graph.hasNode(max_room_id)) {
      if (!room_id_votes.empty()) {
        LOG_FIRST_N(WARNING, 10) << "Room ID: " << NodeSymbol(max_room_id)
                                 << " -> raw: " << max_room_id << " is not in graph";
      }
      default_vertices++;
      segmented_point.r = 0;
      segmented_point.g = 0;
      segmented_point.b = 0;
      continue;
    }

    labeled_vertices++;
    const Node& room_node = *(scene_graph.getNode(max_room_id));
    NodeColor room_color = room_node.attributes<SemanticNodeAttributes>().color;
    segmented_point.r = room_color(0);
    segmented_point.g = room_color(1);
    segmented_point.b = room_color(2);

    // TODO(Toni): Ideally we should update the scene graph at this stage
  }
  VLOG(1) << "Clustered wall vertices: " << labeled_vertices << " (labeled) "
          << unlabeled_vertices << " (unlabeled) " << default_vertices << " (default)";

  pcl::PolygonMesh::Ptr segmented_mesh(new pcl::PolygonMesh());
  pcl::toPCLPointCloud2(segmented_vertices, segmented_mesh->cloud);
  segmented_mesh->polygons = mesh.mesh->polygons;
  return segmented_mesh;
}

}  // namespace kimera
