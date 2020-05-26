#pragma once

#include <string>

namespace kimera {

/** TODO(Marcus): this class finds centroids of objects and fills the scene
 * graph layer corresponding to Objects and perhaps Cameras...
 * @brief The ObjectFinder class
 */
enum class ObjectFinderType { kRegionGrowing = 0, kEuclidean = 1 };

struct RegionGrowingClusterEstimatorParams {
  size_t normal_estimator_neighbour_size_ = 50;

  size_t min_cluster_size_ = 250;
  size_t max_cluster_size_ = 10000000;
  size_t number_of_neighbours_ = 20;

  double smoothness_threshold_ = 3.0 / 180.0 * M_PI;
  double curvature_threshold_ = 1.0;

  inline std::string print() const {
    std::stringstream ss;
    // clang-format off
    ss << "\n================== Region Growing Cluster Config ====================\n";
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
    ss << "\n================== Euclidean Cluster Config ====================\n";
    ss << " - min_cluster_size:                 " << min_cluster_size_ << '\n';
    ss << " - max_cluster_size:                 " << max_cluster_size_ << '\n';
    ss << " - cluster_tolerance:                " << cluster_tolerance_ << '\n';
    ss << "==============================================================\n";
    // clang-format on
    return ss.str();
  }
};

}  // namespace kimera
