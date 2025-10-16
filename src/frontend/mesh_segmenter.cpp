/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra/frontend/mesh_segmenter.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#define PCL_NO_PRECOMPILE
#include <pcl/segmentation/extract_clusters.h>
#undef PCL_NO_PRECOMPILE

namespace hydra {

using spatial_hash::LongIndex;
using ClusterMap = std::map<uint32_t, MeshSegmenter::Clusters>;

namespace {

std::string printLabels(const std::set<uint32_t>& labels) {
  std::stringstream ss;
  ss << "[";
  auto iter = labels.begin();
  while (iter != labels.end()) {
    ss << *iter;
    ++iter;
    if (iter != labels.end()) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}

struct HashedCloud {
  explicit HashedCloud(float grid_size);
  void addPoint(const Eigen::Vector3f& pos);

  const spatial_hash::Grid<LongIndex> grid;
  spatial_hash::LongIndexSet occupied;
  pcl::PointCloud<pcl::PointXYZ>::Ptr vertices;
};

HashedCloud::HashedCloud(float grid_size)
    : grid(grid_size), vertices(new pcl::PointCloud<pcl::PointXYZ>()) {}

void HashedCloud::addPoint(const Eigen::Vector3f& pos) {
  const auto index = grid.toIndex(pos);
  if (occupied.count(index)) {
    return;
  }

  occupied.insert(index);
  vertices->push_back(pcl::PointXYZ(pos.x(), pos.y(), pos.z()));
}

void findAndFillClusters(const MeshSegmenter::Config& config,
                         const pcl::PointCloud<pcl::PointXYZ>::Ptr& vertices,
                         MeshSegmenter::Clusters& clusters) {
  using KdTreeT = pcl::search::KdTree<pcl::PointXYZ>;
  KdTreeT::Ptr tree(new KdTreeT());
  tree->setInputCloud(vertices);

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> estimator;
  estimator.setClusterTolerance(config.cluster_tolerance);
  estimator.setMinClusterSize(config.min_cluster_size);
  estimator.setMaxClusterSize(config.max_cluster_size);
  estimator.setSearchMethod(tree);
  estimator.setInputCloud(vertices);

  std::vector<pcl::PointIndices> found;
  estimator.extract(found);

  clusters.reserve(found.size());
  for (const auto& curr : found) {
    if (curr.indices.empty()) {
      continue;
    }

    auto& cluster = clusters.emplace_back();
    cluster.points.reserve(curr.indices.size());
    for (const auto idx : curr.indices) {
      const auto& p = vertices->at(idx);
      const Eigen::Vector3f pos(p.x, p.y, p.z);
      cluster.centroid += pos;
      cluster.points.push_back(pos);
    }

    cluster.centroid /= cluster.points.size();
  }
}

}  // namespace

void declare_config(MeshSegmenter::Config& config) {
  using namespace config;
  name("MeshSegmenter::Config");
  field(config.labels, "labels");
  field(config.grid_size, "grid_size");
  check(config.grid_size, GT, 0.0f, "grid_size");
}

MeshSegmenter::Config MeshSegmenter::Config::with_labels(
    const std::set<uint32_t>& labels) const {
  auto to_return = *this;
  to_return.labels = labels;
  return to_return;
}

MeshSegmenter::MeshSegmenter(const Config& config)
    : config(config::checkValid(config)) {
  VLOG(2) << "[Mesh Segmenter] using labels: " << printLabels(config.labels);
}

ClusterMap MeshSegmenter::segment(const MeshLayer& mesh) const {
  std::set<uint32_t> seen_labels;
  std::map<uint32_t, HashedCloud> clouds;
  for (const auto& block : mesh) {
    if (!block.has_labels || block.labels.size() != block.points.size()) {
      LOG(WARNING) << "[Mesh Segmenter] Block has no labels!";
      continue;
    }

    for (size_t i = 0; i < block.labels.size(); ++i) {
      const auto label = block.labels[i];
      seen_labels.insert(label);
      if (!config.labels.count(label)) {
        continue;
      }

      auto iter = clouds.find(label);
      if (iter == clouds.end()) {
        iter = clouds.emplace(label, HashedCloud(config.grid_size)).first;
      }

      iter->second.addPoint(block.points[i]);
    }
  }

  VLOG(2) << "[Mesh Segmenter] Seen labels: " << printLabels(seen_labels);

  std::map<uint32_t, Clusters> clusters_by_label;
  for (const auto& [label, cloud] : clouds) {
    if (cloud.vertices->size() < config.min_cluster_size) {
      continue;
    }

    auto iter = clusters_by_label.emplace(label, Clusters{}).first;
    findAndFillClusters(config, cloud.vertices, iter->second);
    VLOG(2) << "[Mesh Segmenter] Found " << iter->second.size()
            << " cluster(s) of label " << label;
  }

  return clusters_by_label;
}

}  // namespace hydra
