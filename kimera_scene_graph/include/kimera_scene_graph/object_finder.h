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
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>

#include <kimera_semantics/common.h>

#include "kimera_scene_graph/common.h"

namespace kimera {

typedef pcl::CentroidPoint<Point> Centroid;
// TODO(Toni): create a structure to hold both Centroids and ObjectPointClouds
// associated to the centroids in the same object...
typedef std::vector<Centroid> Centroids;
typedef std::vector<ColoredPointCloud::Ptr> ObjectPointClouds;

/** TODO(Marcus): this class finds centroids of objects and fills the scene
 * graph layer corresponding to Objects and perhaps Cameras...
 * @brief The ObjectFinder class
 */
enum class ObjectFinderType { kRegionGrowing = 0, kEuclidean = 1 };

struct RegionGrowingClusterEstimatorParams {
  size_t normal_estimator_neighbour_size_ = 50;

  size_t min_cluster_size_ = 50;
  size_t max_cluster_size_ = 1000000;
  size_t number_of_neighbours_ = 30;

  double smoothness_threshold_ = 3.0 / 180.0 * M_PI;
  double curvature_threshold_ = 1.0;

  inline std::string print() const {
    std::stringstream ss;
    // clang-format off
    ss << "================== Region Growing Cluster Config ====================\n";
    ss << " - normal_estimator_neighbour_size:  " << normal_estimator_neighbour_size_ << '\n';
    ss << " - min_cluster_size:                 " << min_cluster_size_ << '\n';
    ss << " - max_cluster_size:                 " << max_cluster_size_ << '\n';
    ss << " - number_of_neighbours:             " << number_of_neighbours_ << '\n';
    ss << " - smoothness_threshold_:            " << smoothness_threshold_ << '\n';
    ss << " - curvature_threshold_:             " << curvature_threshold_ << '\n';
    ss << "==============================================================\n";
    // clang-format on
    return ss.str();
  }
};

struct EuclideanClusterEstimatorParams {
  size_t min_cluster_size_ = 50;
  size_t max_cluster_size_ = 1000000;
  double cluster_tolerance_ = 0.25;

  inline std::string print() const {
    std::stringstream ss;
    // clang-format off
    ss << "================== Euclidean Cluster Config ====================\n";
    ss << " - min_cluster_size:                 " << min_cluster_size_ << '\n';
    ss << " - max_cluster_size:                 " << max_cluster_size_ << '\n';
    ss << " - cluster_tolerance:                " << cluster_tolerance_ << '\n';
    ss << "==============================================================\n";
    // clang-format on
    return ss.str();
  }
};

template <class T>
class ObjectFinder {
 public:
  ObjectFinder(const std::string& world_frame,
               const ObjectFinderType& object_finder_type)
    : world_frame_(world_frame),
      object_finder_type_(object_finder_type) {
    setupRegionGrowingClusterEstimator();
    setupEuclideanClusterEstimator();
  }
  ~ObjectFinder() = default;

  /**
   * @brief findObjects in a given pointcloud by using region growing techniques
   * We assume this pointcloud has already been Semantically segmented (aka
   * it only contains one semantic label)
   * @param pointcloud
   * @return colored pointcloud for cluster visualization...
   */
  ColoredPointCloud::Ptr findObjects(
      const typename pcl::PointCloud<T>::Ptr& pointcloud,
      Centroids* centroids,
      ObjectPointClouds* object_pcls) {
    CHECK_NOTNULL(centroids);
    CHECK_NOTNULL(object_pcls);
    switch (object_finder_type_) {
      case ObjectFinderType::kRegionGrowing: {
        LOG(INFO) << "Region Growing object finder.";
        auto clustered_colored_pointcloud =
            regionGrowingClusterEstimator(pointcloud, centroids, object_pcls);
        CHECK(clustered_colored_pointcloud);
        clustered_colored_pointcloud->header.frame_id = world_frame_;
        return clustered_colored_pointcloud;
        break;
      }
      case ObjectFinderType::kEuclidean: {
        LOG(INFO) << "Euclidean object finder.";
        auto euclidean_cluster_pcl =
            euclideanClusterEstimator(pointcloud, centroids, object_pcls);
        CHECK(euclidean_cluster_pcl);
        euclidean_cluster_pcl->header.frame_id = world_frame_;
        return euclidean_cluster_pcl;
        break;
      }
      default: { LOG(FATAL) << "Unknown object finder type..."; }
    }
  }

  void updateClusterEstimator(const ObjectFinderType& object_finder_type) {
    object_finder_type_ = object_finder_type;
  }

  void updateRegionGrowingParams(const RegionGrowingClusterEstimatorParams& new_params) {
    region_growing_estimator_params_ = new_params;
    setupRegionGrowingClusterEstimator();
  }

  void updateEuclideanClusterParams(const EuclideanClusterEstimatorParams& new_params) {
    euclidean_cluster_estimator_params_ = new_params;
    setupEuclideanClusterEstimator();
  }

 private:
  typename pcl::PointCloud<T>::Ptr regionGrowingClusterEstimator(
      const typename pcl::PointCloud<T>::Ptr& cloud,
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

  typename ColoredPointCloud::Ptr euclideanClusterEstimator(
      const typename pcl::PointCloud<T>::Ptr& cloud,
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

  ColoredPointCloud::Ptr getColoredCloud(
      const typename pcl::PointCloud<T>::Ptr& input,
      const std::vector<pcl::PointIndices>& clusters) {
    ColoredPointCloud::Ptr colored_cloud(
        new ColoredPointCloud);

    if (!clusters.empty()) {
      srand(static_cast<unsigned int>(time(nullptr)));
      std::vector<unsigned char> colors;
      for (std::size_t i_segment = 0; i_segment < clusters.size();
           i_segment++) {
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

  /**
   * @brief getCentroidsGivenClusters returns the centroids (and pointclouds
   * associated to the centroids) given the clusters and the original pointcloud
   * @param cloud original cloud where the clusters are
   * @param cluster_indices indices of the clusters in the original cloud
   * @param centroids centroids of the clusters
   * @param object_pcls chunks of pointclouds associated to the clusters.
   */
  inline void getCentroidsGivenClusters(
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
      ColoredPointCloud::Ptr pcl =
          boost::make_shared<ColoredPointCloud>();
      const auto& indices = cluster_indices.at(k).indices;
      pcl->resize(indices.size());
      for (size_t i = 0; i < indices.size(); ++i) {
        // For centroid of cluster k, add all points belonging to it.
        const ColorPoint& color_point = cloud->at(indices.at(i));
        centroid.add(Point(color_point.x, color_point.y, color_point.z));
        pcl->at(i) = color_point;
      }
      object_pcls->at(k) = pcl;
    }
  }

  void setupRegionGrowingClusterEstimator() {
    region_growing_cluster_estimator_.setMinClusterSize(region_growing_estimator_params_.min_cluster_size_);
    region_growing_cluster_estimator_.setMaxClusterSize(region_growing_estimator_params_.max_cluster_size_);
    region_growing_cluster_estimator_.setNumberOfNeighbours(region_growing_estimator_params_.number_of_neighbours_);
    region_growing_cluster_estimator_.setSmoothnessThreshold(region_growing_estimator_params_.smoothness_threshold_);
    region_growing_cluster_estimator_.setCurvatureThreshold(region_growing_estimator_params_.curvature_threshold_);
  }

  void setupEuclideanClusterEstimator() {
    euclidean_cluster_estimator_.setClusterTolerance(
          euclidean_cluster_estimator_params_.cluster_tolerance_);
    euclidean_cluster_estimator_.setMinClusterSize(euclidean_cluster_estimator_params_.min_cluster_size_);
    euclidean_cluster_estimator_.setMaxClusterSize(euclidean_cluster_estimator_params_.max_cluster_size_);
  }

 private:
  std::string world_frame_;
  ObjectFinderType object_finder_type_;

  RegionGrowingClusterEstimatorParams region_growing_estimator_params_;
  pcl::RegionGrowing<T, pcl::Normal> region_growing_cluster_estimator_;

  EuclideanClusterEstimatorParams euclidean_cluster_estimator_params_;
  pcl::EuclideanClusterExtraction<T> euclidean_cluster_estimator_;
};

}
