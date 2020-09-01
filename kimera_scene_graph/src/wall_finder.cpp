#include "kimera_scene_graph/wall_finder.h"

#include <map>
#include <string>

#include <pcl/search/kdtree.h>

#include <voxblox/mesh/mesh.h>

#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/scene_graph_layer.h"
#include "kimera_scene_graph/scene_graph_layer.h"
#include "kimera_scene_graph/scene_graph_node.h"

namespace kimera {

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
  SceneGraphLayer layer = scene_graph.getLayer(LayerId::kPlacesLayerId);
  std::map<int, NodeId> cloud_to_graph_ids;
  ColorPointCloud::Ptr skeleton_graph_cloud =
      layer.convertLayerToPcl(&cloud_to_graph_ids, nullptr);

  pcl::KdTreeFLANN<ColorPoint> kdtree;
  kdtree.setInputCloud(skeleton_graph_cloud);
  static constexpr int K = 30;
  std::vector<int> nn_indices(K);
  std::vector<float> nn_squared_distances(K);

  // For each wall vertex, query the nearest place.
  // The nearest place room id will be assigned to the wall vertex.
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
    if (nn_size > 0) {
      // Found the nearest neighbor in the skeleton to the wall vertex.
      CHECK_GT(nn_indices.size(), 0);
      CHECK_GT(nn_squared_distances.size(), 0);
      CHECK_EQ(nn_indices.size(), nn_squared_distances.size());
      for (size_t i = 0u; i < nn_size; i++) {
        // Avoid voters that are very far away
        static constexpr double kMaxSquaredDistance = std::pow(5.0, 2);
        const double& nn_squared_distance = nn_squared_distances.at(i);
        if (nn_squared_distance < kMaxSquaredDistance) {
          CHECK_LT(nn_indices.at(i), cloud_to_graph_ids.size());
          auto nearest_place_to_wall_index =
              cloud_to_graph_ids[nn_indices.at(i)];
          const SceneGraphNode& nearest_place_to_wall =
              layer.getNode(nearest_place_to_wall_index);
          vxb::Ray direction =
              kimeraPointToVxbPoint(
                  nearest_place_to_wall.attributes_.position_) -
              wall_vertex;

          NodeId room_id = nearest_place_to_wall.parent_edge_.start_node_id_;
          if (room_id == -1) {
            LOG(WARNING) << "Place node with id "
                         << nearest_place_to_wall.node_id_
                         << " does not have room id when finding walls!";
            continue;
          }
          // Check normal orientation
          CHECK_LT(wall_vertex_idx, walls_mesh.normals.size());
          vxb::Point normal = segmented_walls_mesh->normals.at(wall_vertex_idx);

          double dot_product_not_normalized =
              (normal.transpose() * direction).sum();
          if (dot_product_not_normalized > 0) {
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
          } else {
            VLOG(2) << "Not considering invalid place as a valid room id voter";
          }
        } else {
          VLOG(2) << "Not considering far away place as a valid room id voter";
        }
      }
    } else {
      LOG(ERROR) << "No nearest place to wall vertex found!";
    }

    // Take as room id the most voted room id:
    // Find most voted room id
    int max_room_id = 0u;
    size_t max_room_id_votes = 0u;
    for (const std::pair<int, size_t>& kv : room_id_votes) {
      if (kv.second > max_room_id_votes) {
        max_room_id = kv.first;
        max_room_id_votes = kv.second;
      }
    }

    if (max_room_id_votes == 0u) {
      LOG_EVERY_N(ERROR, 100) << "No valid room id found for vertex of wall..."
                                 "Assigning room id: "
                              << max_room_id;
      CHECK_EQ(max_room_id, 0u);
    }

    // Assign to the wall vertex the color corresponding to the most voted
    // room id.
    CHECK_LT(wall_vertex_idx, walls_mesh.colors.size());
    segmented_walls_mesh->colors.at(wall_vertex_idx) =
        getRoomColor(max_room_id);

    // TODO(Toni): Ideally we should update the scene graph at this stage
  }
}
}
