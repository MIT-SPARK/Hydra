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

// Euclidean pcl
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

// Bounding Box
#include <pcl/features/moment_of_inertia_estimation.h>

#include <kimera_semantics/common.h>

#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/object_finder-definitions.h"

namespace kimera {

template <class T>
class ObjectFinder {
 public:
  typedef pcl::PointCloud<T> PointCloudT;
  typedef std::vector<typename PointCloudT::Ptr> ObjectPointClouds;
  typedef std::vector<BoundingBox<T>> BoundingBoxes;

  ObjectFinder(const std::string& world_frame,
               const ObjectFinderType& object_finder_type);
  ~ObjectFinder() = default;

  /**
   * @brief findObjects in a given pointcloud by using region growing techniques
   * We assume this pointcloud has already been Semantically segmented (aka
   * it only contains one semantic label)
   * @param pointcloud
   * @return colored pointcloud for cluster visualization...
   */
  ColorPointCloud::Ptr findObjects(const typename PointCloudT::Ptr& pointcloud,
                                   Centroids* centroids,
                                   ObjectPointClouds* object_pcls,
                                   BoundingBoxes* bounding_boxes);

  BoundingBox<T> findBoundingBox(const typename PointCloudT::Ptr& cloud,
                                 const BoundingBoxType& bb_type);

  void updateClusterEstimator(const ObjectFinderType& object_finder_type);

  void updateRegionGrowingParams(
      const RegionGrowingClusterEstimatorParams& new_params);

  void updateEuclideanClusterParams(
      const EuclideanClusterEstimatorParams& new_params);

  void print() const {
    switch (static_cast<ObjectFinderType>(object_finder_type_)) {
      case ObjectFinderType::kEuclidean: {
        LOG(INFO) << "Object Finder Type: Euclidean.\n"
                  << euclidean_cluster_estimator_params_.print();
        break;
      };
      case ObjectFinderType::kRegionGrowing: {
        LOG(INFO) << "Object Finder Type: Region-Growing. \n"
                  << region_growing_estimator_params_.print();
        break;
      }
      default: {
        LOG(FATAL) << "Unknown object finder type";
        break;
      }
    }
  }

 private:
  ColorPointCloud::Ptr regionGrowingClusterEstimator(
      const typename PointCloudT::Ptr& cloud,
      Centroids* centroids,
      ObjectPointClouds* object_pcls);

  ColorPointCloud::Ptr euclideanClusterEstimator(
      const typename PointCloudT::Ptr& cloud,
      Centroids* centroids,
      ObjectPointClouds* object_pcls);

  ColorPointCloud::Ptr getColoredCloud(
      const typename pcl::PointCloud<T>::Ptr& input,
      const std::vector<pcl::PointIndices>& clusters);

  /**
   * @brief getCentroidsGivenClusters returns the centroids (and pointclouds
   * associated to the centroids) given the clusters and the original pointcloud
   * @param cloud original cloud where the clusters are
   * @param cluster_indices indices of the clusters in the original cloud
   * @param centroids centroids of the clusters
   * @param object_pcls chunks of pointclouds associated to the clusters.
   */
  void getCentroidsGivenClusters(
      const typename pcl::PointCloud<T>::Ptr& cloud,
      const std::vector<pcl::PointIndices>& cluster_indices,
      Centroids* centroids,
      ObjectPointClouds* object_pcls);

  void setupRegionGrowingClusterEstimator();

  void setupEuclideanClusterEstimator();

 protected:
  // Used as well by the wall finder
  RegionGrowingClusterEstimatorParams region_growing_estimator_params_;

 private:
  std::string world_frame_;
  ObjectFinderType object_finder_type_;

  pcl::RegionGrowing<T, pcl::Normal> region_growing_cluster_estimator_;

  EuclideanClusterEstimatorParams euclidean_cluster_estimator_params_;
  pcl::EuclideanClusterExtraction<T> euclidean_cluster_estimator_;
};

}  // namespace kimera

#include "./object_finder-inl.h"
