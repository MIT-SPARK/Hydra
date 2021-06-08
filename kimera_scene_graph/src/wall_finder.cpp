#include "kimera_scene_graph/wall_finder.h"
#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/pcl_conversion.h"

#include <pcl/search/kdtree.h>

#include <voxblox/mesh/mesh.h>

namespace kimera {

namespace vxb = voxblox;
using Node = SceneGraph::Node;

EnclosingWallFinder::EnclosingWallFinder(const std::string& world_frame)
    : world_frame_(world_frame) {}

void EnclosingWallFinder::findWalls(const vxb::Mesh& walls_mesh,
                                    const SceneGraph& scene_graph,
                                    vxb::Mesh* segmented_walls_mesh) {
  CHECK_NOTNULL(segmented_walls_mesh);
  CHECK(walls_mesh.hasNormals());
  *segmented_walls_mesh = walls_mesh;

  // Create cloud of the skeleton graph so we can do nearest neighbor queries
  // between a wall vertex and the nearest place, make sure indices are 1-to-1
  CHECK(scene_graph.hasLayer(to_underlying(KimeraDsgLayers::PLACES)));
  const SceneGraphLayer& layer =
      *(scene_graph.getLayer(to_underlying(KimeraDsgLayers::PLACES)));
  PclLayer<ColorPointCloud> pcl_layer =
      convertLayerToPcl<ColorPointCloud>(layer);

  pcl::KdTreeFLANN<ColorPoint> kdtree;
  kdtree.setInputCloud(pcl_layer.cloud);
  static constexpr int K = 30;
  std::vector<int> nn_indices(K);
  std::vector<float> nn_squared_distances(K);

  // For each wall vertex, query the nearest place.
  // The nearest place room id will be assigned to the wall vertex.
  size_t unlabeled_vertices = 0;
  size_t default_vertices = 0;
  size_t labeled_vertices = 0;
  for (size_t wall_vertex_idx = 0u;
       wall_vertex_idx < walls_mesh.vertices.size();
       wall_vertex_idx++) {
    CHECK_LT(wall_vertex_idx, walls_mesh.vertices.size());
    const vxb::Point& wall_vertex =
        segmented_walls_mesh->vertices.at(wall_vertex_idx);

    std::map<int, double> room_id_votes;

    // Our query point is the wall vertex
    ColorPoint search_point;
    search_point.x = wall_vertex[0];
    search_point.y = wall_vertex[1];
    search_point.z = wall_vertex[2];
    int nn_size = kdtree.nearestKSearch(
        search_point, K, nn_indices, nn_squared_distances);
    if (nn_size == 0) {
      LOG(ERROR) << "No nearest place to wall vertex found!";
      unlabeled_vertices++;
      continue;
    }

    // Found the nearest neighbor in the skeleton to the wall vertex.
    CHECK_GT(nn_indices.size(), 0u);
    CHECK_GT(nn_squared_distances.size(), 0u);
    CHECK_EQ(nn_indices.size(), nn_squared_distances.size());

    size_t num_far_away = 0u;
    size_t num_outside = 0u;
    size_t num_valid = 0u;
    for (int i = 0u; i < nn_size; i++) {
      // Avoid voters that are very far away
      const double kMaxSquaredDistance = std::pow(5.0, 2);
      const double& nn_squared_distance = nn_squared_distances.at(i);
      if (nn_squared_distance >= kMaxSquaredDistance) {
        num_far_away++;
        continue;
      }

      CHECK_LT(nn_indices.at(i),
               static_cast<int>(pcl_layer.cloud_to_layer_ids.size()));
      auto nearest_node_idx = pcl_layer.cloud_to_layer_ids[nn_indices.at(i)];
      const Node& nearest_node = *(scene_graph.getNode(nearest_node_idx));
      Eigen::Vector3d node_pos = nearest_node.attributes().position;
      vxb::Point vxb_node_pos(node_pos(0), node_pos(1), node_pos(2));
      vxb::Ray direction = vxb_node_pos - wall_vertex;

      if (!nearest_node.hasParent()) {
        continue;
      }

      NodeId room_id = *(nearest_node.getParent());

      // Check normal orientation
      CHECK_LT(wall_vertex_idx, walls_mesh.normals.size());
      vxb::Point normal = segmented_walls_mesh->normals.at(wall_vertex_idx);

      double dot_product_not_normalized =
          (normal.transpose() * direction).sum();
      if (dot_product_not_normalized <= 0.0) {
        num_outside++;
        continue;
      }

      num_valid++;
      // Found valid place "inside" the wall as given by its normal...
      // Get the room id
      if (room_id_votes.find(room_id) == room_id_votes.end()) {
        // Just to make sure we init the value to a 0
        room_id_votes[room_id] = 0u;
      }
      // Weight the vote by the dot product, so just in front places
      // have more weight that the ones around.
      CHECK_GT(nn_squared_distance, 0.0);
      room_id_votes[room_id] +=
          dot_product_not_normalized / nn_squared_distance;
    }

    VLOG(5) << "Wall " << wall_vertex_idx << ": " << num_far_away
            << " (far away) " << num_outside << " (outside) " << num_valid
            << " (valid)";

    // TODO(nathan) fix this so it matches up with the room
    int max_room_id = 0;
    if (room_id_votes.empty()) {
      VLOG_EVERY_N(5, 100) << "No room found for wall vertex (defaulting to 0)";
      default_vertices++;
    } else {
      double max_room_id_votes = 0.0;
      for (const auto& kv : room_id_votes) {
        if (kv.second > max_room_id_votes) {
          max_room_id = kv.first;
          max_room_id_votes = kv.second;
        }
      }
      labeled_vertices++;
    }

    segmented_walls_mesh->colors.at(wall_vertex_idx) =
        getRoomColor(max_room_id);

    // TODO(Toni): Ideally we should update the scene graph at this stage
  }
  VLOG(1) << "Clustered wall vertices: " << labeled_vertices << " (labeled) "
          << unlabeled_vertices << " (unlabeled) " << default_vertices
          << " (default)";
}

}  // namespace kimera
