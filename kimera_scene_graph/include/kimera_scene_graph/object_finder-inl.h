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
#include "kimera_scene_graph/object_finder.h" // just to get autocomplete in qt.

namespace kimera {

template <class T>
ObjectFinder<T>::ObjectFinder(const std::string& world_frame,
                              const ObjectFinderType& object_finder_type)
    : world_frame_(world_frame), object_finder_type_(object_finder_type) {
  setupRegionGrowingClusterEstimator();
  setupEuclideanClusterEstimator();
}

template <class T>
ColorPointCloud::Ptr ObjectFinder<T>::findObjects(
    const typename PointCloudT::Ptr& pointcloud,
    Centroids* centroids,
    ObjectPointClouds* object_pcls,
    BoundingBoxes* bounding_boxes) {
  CHECK_NOTNULL(centroids);
  CHECK_NOTNULL(object_pcls);
  CHECK_NOTNULL(bounding_boxes);
  ColorPointCloud::Ptr clustered_colored_pcl = nullptr;
  switch (object_finder_type_) {
    case ObjectFinderType::kRegionGrowing: {
      LOG(INFO) << "Region Growing object finder.";
      clustered_colored_pcl =
          regionGrowingClusterEstimator(pointcloud, centroids, object_pcls);
      break;
    }
    case ObjectFinderType::kEuclidean: {
      LOG(INFO) << "Euclidean object finder.";
      clustered_colored_pcl =
          euclideanClusterEstimator(pointcloud, centroids, object_pcls);
      break;
    }
    default: { LOG(FATAL) << "Unknown object finder type..."; }
  }
  CHECK(clustered_colored_pcl);

  // Find BB
  bounding_boxes->resize(object_pcls->size());
  for (size_t i = 0; i < object_pcls->size(); ++i) {
    bounding_boxes->at(i) =
        findBoundingBox(object_pcls->at(i), BoundingBoxType::kAABB);
  }

  clustered_colored_pcl->header.frame_id = world_frame_;
  return clustered_colored_pcl;
}

template <class T>
BoundingBox<T> ObjectFinder<T>::findBoundingBox(
    const typename PointCloudT::Ptr& cloud,
    const BoundingBoxType& bb_type) {
  CHECK(cloud);
  pcl::MomentOfInertiaEstimation<T> feature_extractor;
  feature_extractor.setInputCloud(cloud);
  feature_extractor.compute();

  BoundingBox<T> bb;
  bb.type_ = bb_type;

  std::vector<float> moment_of_inertia;
  std::vector<float> eccentricity;
  Eigen::Matrix3f rotational_matrix_OBB;
  float major_value, middle_value, minor_value;
  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  Eigen::Vector3f mass_center;

  feature_extractor.getMomentOfInertia(moment_of_inertia);
  feature_extractor.getEccentricity(eccentricity);

  switch (bb_type) {
    case BoundingBoxType::kOBB: {
      T min_point_OBB;
      T max_point_OBB;
      T position_OBB;
      feature_extractor.getOBB(
          min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
      bb.max_ = max_point_OBB;
      bb.min_ = min_point_OBB;
      bb.position_ = position_OBB;
      bb.orientation_matrix = rotational_matrix_OBB;
      break;
    }
    case BoundingBoxType::kAABB: {
      T min_point_AABB;
      T max_point_AABB;
      feature_extractor.getAABB(min_point_AABB, max_point_AABB);
      bb.max_ = max_point_AABB;
      bb.min_ = min_point_AABB;
      break;
    }
    default: {
      LOG(FATAL) << "Unknown bounding box type.";
      break;
    }
  }

  // Do we want these? Perhaps to display a frame of reference? Perhaps
  // for ICP?
  // feature_extractor.getEigenValues(major_value, middle_value, minor_value);
  // feature_extractor.getEigenVectors(
  //    major_vector, middle_vector, minor_vector);
  // feature_extractor.getMassCenter(mass_center);
  return bb;
}

template <class T>
void ObjectFinder<T>::updateClusterEstimator(
    const ObjectFinderType& object_finder_type) {
  object_finder_type_ = object_finder_type;
}

template <class T>
void ObjectFinder<T>::updateRegionGrowingParams(
    const RegionGrowingClusterEstimatorParams& new_params) {
  region_growing_estimator_params_ = new_params;
  setupRegionGrowingClusterEstimator();
}

template <class T>
void ObjectFinder<T>::updateEuclideanClusterParams(
    const EuclideanClusterEstimatorParams& new_params) {
  euclidean_cluster_estimator_params_ = new_params;
  setupEuclideanClusterEstimator();
}

template <class T>
ColorPointCloud::Ptr ObjectFinder<T>::regionGrowingClusterEstimator(
    const typename PointCloudT::Ptr& cloud,
    Centroids* centroids,
    ObjectPointClouds* object_pcls) {
  typename pcl::search::Search<T>::Ptr tree(new pcl::search::KdTree<T>);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<T, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setKSearch(
      region_growing_estimator_params_.normal_estimator_neighbour_size_);

  normal_estimator.setInputCloud(cloud);
  normal_estimator.compute(*normals);

  // pcl::IndicesPtr indices(new std::vector<int>);
  // pcl::PassThrough<T> pass;
  // pass.setInputCloud(cloud);
  // pass.setFilterFieldName("z");
  // pass.setFilterLimits(0.0, 1.0);
  // pass.filter(*indices);

  region_growing_cluster_estimator_.setSearchMethod(tree);

  region_growing_cluster_estimator_.setInputCloud(cloud);
  region_growing_cluster_estimator_.setInputNormals(normals);

  std::vector<pcl::PointIndices> cluster_indices;
  region_growing_cluster_estimator_.extract(cluster_indices);

  // Get centroids of clusters.
  getCentroidsGivenClusters(cloud, cluster_indices, centroids, object_pcls);

  LOG(INFO) << "Number of clusters found: " << cluster_indices.size();
  return region_growing_cluster_estimator_.getColoredCloud();
}

template <class T>
ColorPointCloud::Ptr ObjectFinder<T>::euclideanClusterEstimator(
    const typename PointCloudT::Ptr& cloud,
    Centroids* centroids,
    ObjectPointClouds* object_pcls) {
  CHECK_NOTNULL(centroids);
  CHECK_NOTNULL(object_pcls);
  // Creating the KdTree object for the search method of the extraction
  typename pcl::search::KdTree<T>::Ptr tree(new pcl::search::KdTree<T>);
  tree->setInputCloud(cloud);

  euclidean_cluster_estimator_.setSearchMethod(tree);
  euclidean_cluster_estimator_.setInputCloud(cloud);

  // Extract clusters
  std::vector<pcl::PointIndices> cluster_indices;
  euclidean_cluster_estimator_.extract(cluster_indices);

  // Get centroids of clusters.
  getCentroidsGivenClusters(cloud, cluster_indices, centroids, object_pcls);

  return getColoredCloud(cloud, cluster_indices);
}

template <class T>
ColorPointCloud::Ptr ObjectFinder<T>::getColoredCloud(
    const typename pcl::PointCloud<T>::Ptr& input,
    const std::vector<pcl::PointIndices>& clusters) {
  ColorPointCloud::Ptr colored_cloud(new ColorPointCloud);

  if (!clusters.empty()) {
    srand(static_cast<unsigned int>(time(nullptr)));
    std::vector<unsigned char> colors;
    for (std::size_t i_segment = 0; i_segment < clusters.size(); i_segment++) {
      colors.push_back(static_cast<unsigned char>(rand() % 256));
      colors.push_back(static_cast<unsigned char>(rand() % 256));
      colors.push_back(static_cast<unsigned char>(rand() % 256));
    }

    colored_cloud->width = input->width;
    colored_cloud->height = input->height;
    colored_cloud->is_dense = input->is_dense;
    for (std::size_t i_point = 0; i_point < input->points.size(); i_point++) {
      ColorPoint point;
      point.x = *(input->points[i_point].data);
      point.y = *(input->points[i_point].data + 1);
      point.z = *(input->points[i_point].data + 2);
      point.r = 255;
      point.g = 0;
      point.b = 0;
      colored_cloud->points.push_back(point);
    }

    int next_color = 0;
    for (auto i_segment = clusters.cbegin(); i_segment != clusters.cend();
         i_segment++) {
      for (auto i_point = i_segment->indices.cbegin();
           i_point != i_segment->indices.cend();
           i_point++) {
        int index;
        index = *i_point;
        colored_cloud->points[index].r = colors[3 * next_color];
        colored_cloud->points[index].g = colors[3 * next_color + 1];
        colored_cloud->points[index].b = colors[3 * next_color + 2];
      }
      next_color++;
    }
  }

  return colored_cloud;
}

template <class T>
void ObjectFinder<T>::getCentroidsGivenClusters(
    const typename pcl::PointCloud<T>::Ptr& cloud,
    const std::vector<pcl::PointIndices>& cluster_indices,
    Centroids* centroids,
    ObjectPointClouds* object_pcls) {
  CHECK_NOTNULL(centroids);
  CHECK_NOTNULL(object_pcls);
  centroids->resize(cluster_indices.size());
  object_pcls->resize(cluster_indices.size());
  for (size_t k = 0; k < cluster_indices.size(); ++k) {
    Centroid& centroid = centroids->at(k);
    typename PointCloudT::Ptr pcl = boost::make_shared<PointCloudT>();
    const auto& indices = cluster_indices.at(k).indices;
    pcl->resize(indices.size());
    for (size_t i = 0; i < indices.size(); ++i) {
      // For centroid of cluster k, add all points belonging to it.
      const T& color_point = cloud->at(indices.at(i));
      centroid.add(Point(color_point.x, color_point.y, color_point.z));
      pcl->at(i) = color_point;
    }
    object_pcls->at(k) = pcl;
  }
}

template <class T>
void ObjectFinder<T>::setupRegionGrowingClusterEstimator() {
  region_growing_cluster_estimator_.setMinClusterSize(
      region_growing_estimator_params_.min_cluster_size_);
  region_growing_cluster_estimator_.setMaxClusterSize(
      region_growing_estimator_params_.max_cluster_size_);
  region_growing_cluster_estimator_.setNumberOfNeighbours(
      region_growing_estimator_params_.number_of_neighbours_);
  region_growing_cluster_estimator_.setSmoothnessThreshold(
      region_growing_estimator_params_.smoothness_threshold_);
  region_growing_cluster_estimator_.setCurvatureThreshold(
      region_growing_estimator_params_.curvature_threshold_);
}

template <class T>
void ObjectFinder<T>::setupEuclideanClusterEstimator() {
  euclidean_cluster_estimator_.setClusterTolerance(
      euclidean_cluster_estimator_params_.cluster_tolerance_);
  euclidean_cluster_estimator_.setMinClusterSize(
      euclidean_cluster_estimator_params_.min_cluster_size_);
  euclidean_cluster_estimator_.setMaxClusterSize(
      euclidean_cluster_estimator_params_.max_cluster_size_);
}

}  // namespace kimera
