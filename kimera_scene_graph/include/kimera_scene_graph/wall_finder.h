#pragma once

#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

// Region growing pcl
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>

// For Outlier removal
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

// Bounding Box
#include <pcl/features/moment_of_inertia_estimation.h>

#include <voxblox_skeleton/skeleton.h>

#include <kimera_semantics/common.h>

#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/object_finder.h"  // for params definitions

#include "kimera_scene_graph/room_finder.h"  // Only for downsamplePcl()

#include "kimera_scene_graph/utils/voxblox_to_pcl.h"  // for createPclCloud

namespace kimera {

class EnclosingWallFinder {
 public:
  EnclosingWallFinder(const std::string& world_frame)
      : world_frame_(world_frame) {}
  ~EnclosingWallFinder() = default;

 public:
  /**
   * @brief findWalls
   * @param walls_mesh[in/out] Mesh of the walls without room labels [in] and
   * with room labels [out]
   */
  void findWalls(const vxb::Mesh& walls_mesh,
                 const vxb::SparseSkeletonGraph& sparse_skeleton_graph,
                 vxb::Mesh* segmented_walls_mesh) {
    CHECK_NOTNULL(segmented_walls_mesh);
    CHECK(walls_mesh.hasNormals());
    *segmented_walls_mesh = walls_mesh;

    // Create cloud of the skeleton graph so we can do nearest neighbor queries
    // between a wall vertex and the nearest place, make sure indices are 1-to-1
    std::vector<int64_t> vertex_ids;
    std::map<int, int64_t> cloud_to_graph_ids;
    ColorPointCloud::Ptr skeleton_graph_cloud = createPclCloudFromSkeleton(
        sparse_skeleton_graph, &cloud_to_graph_ids, &vertex_ids);

    pcl::KdTreeFLANN<ColorPoint> kdtree;
    kdtree.setInputCloud(skeleton_graph_cloud);
    static constexpr int K = 30;  // find 5 nearest-neighbor
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
            const vxb::SkeletonVertex& nearest_place_to_wall =
                sparse_skeleton_graph.getVertex(nearest_place_to_wall_index);
            vxb::Ray direction = nearest_place_to_wall.point - wall_vertex;

            CHECK_NE(nearest_place_to_wall.room_id, 0u)
                << "Vertex does not have room id when finding walls!";
            // Check normal orientation
            CHECK_LT(wall_vertex_idx, walls_mesh.normals.size());
            vxb::Point normal =
                segmented_walls_mesh->normals.at(wall_vertex_idx);

            double dot_product_not_normalized =
                (normal.transpose() * direction).sum();
            if (dot_product_not_normalized > 0) {
              // Found valid place "inside" the wall as given by its normal...
              // Get the room id
              if (room_id_votes.find(nearest_place_to_wall.room_id) ==
                  room_id_votes.end()) {
                // Just to make sure we init the value to a 0
                room_id_votes[nearest_place_to_wall.room_id] = 0u;
              }
              // Weight the vote by the dot product, so just in front places
              // have more weight that the ones around.
              CHECK_GT(nn_squared_distance, 0.0);
              room_id_votes[nearest_place_to_wall.room_id] +=
                  dot_product_not_normalized / nn_squared_distance;
            } else {
              VLOG(2)
                  << "Not considering invalid place as a valid room id voter";
            }
          } else {
            VLOG(2)
                << "Not considering far away place as a valid room id voter";
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
        LOG_EVERY_N(ERROR, 100)
            << "No valid room id found for vertex of wall..."
               "Assigning room id: "
            << max_room_id;
        CHECK_EQ(max_room_id, 0u);
      }

      // Assign to the wall vertex the color corresponding to the most voted
      // room id.
      CHECK_LT(wall_vertex_idx, walls_mesh.colors.size());
      segmented_walls_mesh->colors.at(wall_vertex_idx) =
          getRoomColor(max_room_id);
    }
  }

 private:
  std::string world_frame_;
};

/** WallFinder class. Attempts to find walls by using a region growing approach
 * This is only useful if we define a wall as a plane-like object.
 * Otherwise, if you want to define a wall as the surrounding planes of a room
 * (without necessarily having a plane-like equation), then use
 * EnclosingWallFinder
 */
template <class T>
class WallFinder : protected ObjectFinder<T> {
 public:
  using PointCloudT = typename ObjectFinder<T>::PointCloudT;
  using ObjectPointClouds = typename ObjectFinder<T>::ObjectPointClouds;
  using BoundingBoxes = typename ObjectFinder<T>::BoundingBoxes;

  WallFinder(const std::string& world_frame)
      : ObjectFinder<T>(world_frame, ObjectFinderType::kRegionGrowing) {}

  virtual ~WallFinder() = default;

  /**
   * @brief findWalls in a given pointcloud by using region growing techniques
   * We assume this pointcloud has already been Semantically segmented (aka
   * it only contains one semantic label)
   * @param pointcloud
   * @return colored pointcloud for cluster visualization...
   */
  ColorPointCloud::Ptr findWalls(const typename PointCloudT::Ptr& pointcloud,
                                 Centroids* centroids,
                                 ObjectPointClouds* object_pcls,
                                 BoundingBoxes* bounding_boxes) {
    CHECK(pointcloud);
    CHECK_NOTNULL(centroids);
    CHECK_NOTNULL(object_pcls);
    CHECK_NOTNULL(bounding_boxes);

    // Downsample
    typename PointCloudT::Ptr cloud_filtered =
        downsamplePcl<T>(pointcloud, 0.15f);

    // Viz downsampled pcl

    // TODO(Toni): we could use the mesh for this, but pcl is optimized I think?
    // Remove floating points (there are many!)
    // pcl::RadiusOutlierRemoval<T> outrem;
    //// Build the filter
    // outrem.setInputCloud(cloud_filtered);
    // outrem.setRadiusSearch(0.2f);
    // outrem.setMinNeighborsInRadius(4);
    //// Apply filter
    // outrem.filter(*cloud_filtered);

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<T> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    // Fine-tune params to find clusters
    ObjectFinder<T>::region_growing_estimator_params_.min_cluster_size_ = 500;
    ObjectFinder<T>::region_growing_estimator_params_.curvature_threshold_ =
        2.0;
    // region_growing_estimator_params_.smoothness_threshold_ = ;

    return ObjectFinder<T>::findObjects(
        cloud_filtered, centroids, object_pcls, bounding_boxes);
  }

 private:
};

}  // namespace kimera
