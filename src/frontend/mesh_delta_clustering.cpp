#include "hydra/frontend/mesh_delta_clustering.h"

#include <config_utilities/config.h>
#include <glog/logging.h>
#include <kimera_pgmo/mesh_delta.h>
#include <pcl/point_cloud.h>

#define PCL_NO_PRECOMPILE
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#undef PCL_NO_PRECOMPILE

namespace hydra::clustering {

void declare_config(ClusteringConfig& config) {
  using namespace config;
  name("ClusteringConfig");
  field(config.cluster_tolerance, "cluster_tolerance");
  field(config.min_cluster_size, "min_cluster_size");
  field(config.max_cluster_size, "max_cluster_size");
  check(config.cluster_tolerance, GT, 0.0, "clustering_tolerance");
  check(config.min_cluster_size, GT, 0, "min_cluster_size");
  check(config.max_cluster_size, GT, config.min_cluster_size, "max > min");
}

std::string printLabels(const std::set<uint32_t>& labels) {
  std::stringstream ss;
  ss << "[";
  auto iter = labels.begin();
  while (iter != labels.end()) {
    ss << static_cast<uint64_t>(*iter);
    ++iter;
    if (iter != labels.end()) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}

LabelIndices getLabelIndices(const std::set<uint32_t>& desired_labels,
                             const kimera_pgmo::MeshDelta& delta,
                             const std::unordered_set<size_t>* invalid) {
  LabelIndices label_indices;
  if (!kimera_pgmo::traits::get_vertex_properties(delta).has_label) {
    LOG(WARNING) << "[Delta Clustering] Mesh missing labels!";
    return label_indices;
  }

  std::set<uint32_t> seen_labels;
  for (size_t i = delta.getNumArchivedVertices(); i < delta.getNumVertices(); ++i) {
    if (invalid && invalid->count(i)) {
      continue;
    }

    const auto& v = delta.getVertex(i);
    const auto label = v.traits.label;
    seen_labels.insert(label);
    if (!desired_labels.count(label)) {
      continue;
    }

    auto iter = label_indices.find(label);
    if (iter == label_indices.end()) {
      iter = label_indices.emplace(label, std::vector<size_t>()).first;
    }

    iter->second.push_back(i);
  }

  VLOG(2) << "[Delta Clustering] Seen labels: " << printLabels(seen_labels);
  return label_indices;
}

Clusters findClusters(const ClusteringConfig& config,
                      const kimera_pgmo::MeshDelta& delta,
                      const std::vector<size_t>& indices) {
  using pcl::PointXYZ;
  pcl::PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<PointXYZ>());
  for (const auto idx : indices) {
    const auto& pos = delta.getVertex(idx).pos;
    auto& p = cloud->emplace_back();
    p.x = pos.x();
    p.y = pos.y();
    p.z = pos.z();
  }

  pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>());
  tree->setInputCloud(cloud);

  pcl::EuclideanClusterExtraction<PointXYZ> estimator;
  estimator.setClusterTolerance(config.cluster_tolerance);
  estimator.setMinClusterSize(config.min_cluster_size);
  estimator.setMaxClusterSize(config.max_cluster_size);
  estimator.setSearchMethod(tree);
  estimator.setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  estimator.extract(cluster_indices);

  Clusters clusters;
  clusters.resize(cluster_indices.size());
  for (size_t k = 0; k < clusters.size(); ++k) {
    auto& cluster = clusters.at(k);
    const auto& indices = cluster_indices.at(k).indices;
    cluster.insert(cluster.end(), indices.begin(), indices.end());
  }

  return clusters;
}

}  // namespace hydra::clustering
