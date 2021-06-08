#pragma once
#include "kimera_scene_graph/common.h"

#include <actionlib/client/simple_action_client.h>
#include <object_db/ObjectRegistrationAction.h>

#include <kimera_dsg/node_attributes.h>
#include <kimera_dsg/scene_graph.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>

#include <Eigen/StdVector>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace kimera {

using ObjectDBClient =
    actionlib::SimpleActionClient<object_db::ObjectRegistrationAction>;
using ObjectPointClouds = std::vector<ColorPointCloud::Ptr>;
using NodeColor = SemanticNodeAttributes::ColorVector;
using BoundingBoxes =
    std::vector<BoundingBox, Eigen::aligned_allocator<BoundingBox>>;

enum class ObjectFinderType { kRegionGrowing = 0, kEuclidean = 1 };

struct RegionGrowingClusteringParams {
  size_t normal_estimator_neighbour_size = 50;
  size_t min_cluster_size = 250;
  size_t max_cluster_size = 10000000;
  size_t number_of_neighbours = 20;
  double smoothness_threshold = 3.0 / 180.0 * M_PI;
  double curvature_threshold = 1.0;
};

struct EuclideanClusteringParams {
  size_t min_cluster_size = 50;
  size_t max_cluster_size = 1000000;
  double cluster_tolerance = 0.25;
};

class ObjectFinder {
 public:
  ObjectFinder(const std::string& world_frame,
               ObjectFinderType object_finder_type);

  void connectToObjectDb();

  void addObjectsToGraph(const ColorPointCloud::Ptr& semantic_pcl,
                         const NodeColor& label_color,
                         SemanticLabel label,
                         SceneGraph* scene_graph);

  void updateClusterEstimator(ObjectFinderType object_finder_type);

  void updateRegionGrowingParams(
      const RegionGrowingClusteringParams& new_params);

  void updateEuclideanClusterParams(
      const EuclideanClusteringParams& new_params);

  friend std::ostream& operator<<(std::ostream& out,
                                  const ObjectFinder& finder);

 private:
  void setupRegionGrowingClusterEstimator();

  void setupEuclideanClusterEstimator();

  ColorPointCloud::Ptr findObjects(const ColorPointCloud::Ptr& pointcloud,
                                   Centroids* centroids,
                                   ObjectPointClouds* object_pcls,
                                   BoundingBoxes* bounding_boxes);

  ColorPointCloud::Ptr regionGrowingClusterEstimator(
      const ColorPointCloud::Ptr& cloud,
      Centroids* centroids,
      ObjectPointClouds* object_pcls);

  ColorPointCloud::Ptr euclideanClusterEstimator(
      const ColorPointCloud::Ptr& cloud,
      Centroids* centroids,
      ObjectPointClouds* object_pcls);

  ColorPointCloud::Ptr getColoredCloud(
      const ColorPointCloud::Ptr& input,
      const std::vector<pcl::PointIndices>& clusters);

  void getCentroidsGivenClusters(
      const ColorPointCloud::Ptr& cloud,
      const std::vector<pcl::PointIndices>& cluster_indices,
      Centroids* centroids,
      ObjectPointClouds* object_pcls);

  ObjectPointClouds registerObjects(const ObjectPointClouds& object_pcls,
                                    const std::string semantic_label);

  std::string world_frame_;
  ObjectFinderType type_;

  RegionGrowingClusteringParams region_growing_params_;
  EuclideanClusteringParams euclidean_params_;

  pcl::RegionGrowing<ColorPoint, pcl::Normal> region_growing_estimator_;
  pcl::EuclideanClusterExtraction<ColorPoint> euclidean_estimator_;

  std::unique_ptr<ObjectDBClient> object_db_client_;
  NodeSymbol next_object_id_;
};

}  // namespace kimera
