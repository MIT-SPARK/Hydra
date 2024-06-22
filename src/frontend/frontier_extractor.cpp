#include <config_utilities/config_utilities.h>
#include <spark_dsg/dynamic_scene_graph.h>
#include <spark_dsg/dynamic_scene_graph_layer.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/scene_graph_types.h>
#include <spatial_hash/neighbor_utils.h>

#include <algorithm>

#define PCL_NO_PRECOMPILE
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#undef PCL_NO_PRECOMPILE

#include <queue>

#include "hydra/common/config_utilities.h"
#include "hydra/frontend/frontier_extractor.h"
#include "hydra/reconstruction/voxel_types.h"
#include "hydra/utils/nearest_neighbor_utilities.h"

namespace hydra {

using spatial_hash::IndexSet;

template <typename T, size_t N>
std::pair<T, Eigen::Matrix<T, N, 1>> getMaxEigenvector(Eigen::Matrix<T, N, N> cov) {
  Eigen::EigenSolver<Eigen::Matrix<T, N, N>> es(cov, true);
  auto evals_complex = es.eigenvalues();
  Eigen::Matrix<T, N, 1> evals = evals_complex.real();
  Eigen::Index max_idx;
  evals.maxCoeff(&max_idx);
  auto max_vec_complex = es.eigenvectors().col(max_idx);
  Eigen::Matrix<T, N, 1> max_vec = max_vec_complex.real();
  return std::pair(evals(max_idx), max_vec);
}

Eigen::Vector3d frontiersToCenters(const std::vector<Eigen::Vector3f>& positions) {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (auto p : positions) {
    center += p.cast<double>();
  }
  return center / positions.size();
}

void centroidAndCovFromPoints(const std::vector<Eigen::Vector3f>& positions,
                              Eigen::Vector3f& centroid,
                              Eigen::Matrix3f& cov) {
  centroid = frontiersToCenters(positions).cast<float>();

  cov.setZero();
  for (auto p : positions) {
    auto u = p - centroid;
    cov += u * u.transpose();
  }
  cov /= positions.size();
}

bool splitFrontier(const std::vector<Eigen::Vector3f>& positions,
                   const double std_threshold,
                   std::vector<Eigen::Vector3f>& child1,
                   std::vector<Eigen::Vector3f>& child2) {
  Eigen::Vector3f centroid;
  Eigen::Matrix3f cov;
  centroidAndCovFromPoints(positions, centroid, cov);

  std::pair<float, Eigen::Vector3f> val_vec = getMaxEigenvector<float, 3>(cov);
  if (val_vec.first < std_threshold) {
    return false;
  }
  for (auto p : positions) {
    auto u = p - centroid;
    auto prod = u.dot(val_vec.second);
    if (prod > 0) {
      child1.push_back(p);
    } else {
      child2.push_back(p);
    }
  }
  return true;
}

void splitAllFrontiers(std::vector<std::vector<Eigen::Vector3f>>& frontiers,
                       const double std_threshold,
                       const size_t point_threshold,
                       std::vector<std::vector<Eigen::Vector3f>>& finished_frontiers) {
  while (frontiers.size() > 0) {
    std::vector<Eigen::Vector3f> frontier = frontiers.back();
    frontiers.pop_back();
    std::vector<Eigen::Vector3f> child1;
    std::vector<Eigen::Vector3f> child2;
    if (splitFrontier(frontier, std_threshold, child1, child2)) {
      if (child1.size() < point_threshold) {
        finished_frontiers.push_back(child1);
      } else {
        frontiers.push_back(child1);
      }
      if (child2.size() < point_threshold) {
        finished_frontiers.push_back(child2);
      } else {
        frontiers.push_back(child2);
      }
    } else {
      finished_frontiers.push_back(frontier);
    }
  }
}

FrontierExtractor::FrontierExtractor(const Config& config)
    : config(config), next_node_id_(config.prefix, 0) {}

void clusterFrontiers(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                      const double cluster_tolerance,
                      const size_t min_cluster_size,
                      const size_t max_cluster_size,
                      std::vector<std::vector<Eigen::Vector3f>>& frontiers_to_split) {
  pcl::IndicesPtr cloud_indices(new pcl::Indices);
  cloud_indices->resize(cloud->points.size());
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    cloud_indices->at(i) = i;
  }
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  tree->setInputCloud(cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> estimator;
  estimator.setClusterTolerance(cluster_tolerance);
  estimator.setMinClusterSize(min_cluster_size);
  estimator.setMaxClusterSize(max_cluster_size);
  estimator.setSearchMethod(tree);
  estimator.setInputCloud(cloud);
  estimator.setIndices(cloud_indices);

  std::vector<pcl::PointIndices> cluster_indices;
  estimator.extract(cluster_indices);

  std::vector<std::pair<Eigen::Vector3d, int>> frontier_clusters;

  frontiers_to_split.clear();
  frontier_clusters.resize(cluster_indices.size());
  for (size_t k = 0; k < cluster_indices.size(); ++k) {
    frontiers_to_split.push_back(std::vector<Eigen::Vector3f>());
    pcl::CentroidPoint<pcl::PointXYZ> centroid;
    for (const auto& ind : cluster_indices.at(k).indices) {
      pcl::PointXYZ pt = cloud->points.at(ind);
      centroid.add(pt);
      frontiers_to_split.back().push_back({pt.x, pt.y, pt.z});
    }
    pcl::PointXYZ c;
    centroid.get(c);
  }
}

std::vector<std::pair<Eigen::Vector3d, double>> getPlacesForBlock(
    const DynamicSceneGraph& graph,
    const Eigen::Vector3f& block_center,
    NearestNodeFinder& finder,
    const double block_size,
    const double max_place_radius) {
  std::vector<std::pair<Eigen::Vector3d, double>> center_dists;
  finder.findRadius(block_center.cast<double>(),
                    block_size * 1.414 + max_place_radius,
                    false,
                    [&](NodeId pid, size_t, double) {
                      const auto& pattr =
                          graph.getNode(pid).attributes<PlaceNodeAttributes>();
                      center_dists.push_back({pattr.position, pattr.distance});
                    });
  return center_dists;
}

void computeVoxelsInPlace(
    const Eigen::Vector3f block_origin,
    const std::vector<std::pair<Eigen::Vector3d, double>>& center_dists,
    const std::vector<std::pair<Eigen::Vector3d, double>>& archived_center_dists,
    const size_t voxels_per_block,
    const size_t voxels_per_side,
    const double voxel_size,
    std::vector<bool>& inside_place,
    std::vector<bool>& inside_archived_place) {
  for (size_t v = 0; v < voxels_per_block; ++v) {
    VoxelIndex ind = spatial_hash::voxelIndexFromLinearIndex(v, voxels_per_side);
    const Point center =
        block_origin + spatial_hash::centerPointFromIndex(ind, voxel_size);
    inside_place[v] = false;
    for (const auto& cd : center_dists) {
      if ((center - cd.first.cast<float>()).norm() <= cd.second) {
        inside_place[v] = true;
        break;
      }
    }
    inside_archived_place[v] = false;
    if (inside_place[v]) {
      continue;
    }
    for (const auto& cd : archived_center_dists) {
      if ((center - cd.first.cast<float>()).norm() <= cd.second) {
        inside_archived_place[v] = true;
        break;
      }
    }
  }
}

void checkFreeNeighbors(const std::vector<bool>& inside_place,
                        const std::vector<bool>& inside_archived_place,
                        const int vps,
                        const VoxelIndex& index,
                        bool& neighbor_free,
                        bool& neighbor_archived_free) {
  const spatial_hash::NeighborSearch search(26);
  for (const auto& neighbor : search.neighborIndices(index)) {
    if (neighbor.array().minCoeff() < 0 || neighbor.array().maxCoeff() >= vps) {
      continue;
    }
    const size_t vn = spatial_hash::linearIndexFromVoxelIndex(neighbor, vps);
    neighbor_free = inside_place[vn];
    neighbor_archived_free |= inside_archived_place[vn];
    if (neighbor_free) {
      return;
    }
  }
}

void processBlock(NearestNodeFinder& finder,
                  const BlockIndex& block_index,
                  const DynamicSceneGraph& graph,
                  const ReconstructionOutput& input,
                  const std::vector<NodeId>& archived_places,
                  const double max_place_radius,
                  const double min_frontier_z,
                  const double max_frontier_z,
                  const bool skip_adding_frontiers,
                  IndexSet& skip_add_blocks,
                  std::queue<BlockIndex>& extra_blocks,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr archived_cloud) {
  const auto& tsdf = input.map().getTsdfLayer();
  // Get all active places near block
  const Eigen::Vector3f block_center =
      spatial_hash::centerPointFromIndex(block_index, tsdf.blockSize());
  const Eigen::Vector3f block_origin =
      spatial_hash::originPointFromIndex(block_index, tsdf.blockSize());
  auto center_dists = getPlacesForBlock(
      graph, block_center, finder, tsdf.blockSize(), max_place_radius);

  // TODO(aaron): We should probably check to see if the whole block is outside of the
  // z band of voxels we are about. Can greatly reduce amount of work.

  // Get all recently-archived places near block
  std::vector<std::pair<Eigen::Vector3d, double>> archived_center_dists;
  for (auto pid : archived_places) {
    const auto node = graph.findNode(pid);
    if (!node) {
      continue;
    }

    const auto& pattr = node->attributes<PlaceNodeAttributes>();
    archived_center_dists.push_back({pattr.position, pattr.distance});
  }

  // find all voxels that are inside a place
  const size_t voxels_per_block = std::pow(tsdf.voxels_per_side, 3);
  std::vector<bool> inside_place(voxels_per_block);
  std::vector<bool> inside_archived_place(voxels_per_block);

  computeVoxelsInPlace(block_origin,
                       center_dists,
                       archived_center_dists,
                       voxels_per_block,
                       tsdf.voxels_per_side,
                       tsdf.voxel_size,
                       inside_place,
                       inside_archived_place);

  // find voxels that are on boundary of unobserved space and space inside a place
  for (size_t v = 0; v < voxels_per_block; ++v) {
    // Check the voxel is not observed in the TSDF
    const auto tsdf_block = tsdf.getBlockPtr(block_index);
    if (tsdf_block && tsdf_block->getVoxel(v).weight >= 1e-6) {
      continue;
    }

    const VoxelIndex voxel_index =
        spatial_hash::voxelIndexFromLinearIndex(v, tsdf.voxels_per_side);

    // Skip voxels outside of the window that we care about
    VoxelKey key(block_index, voxel_index);
    auto center = tsdf.getVoxelPosition(key);
    if (center.z() < min_frontier_z or center.z() > max_frontier_z) {
      continue;
    }

    // If voxel is on the "border" of the block, and it is inside a place, and the
    // neighboring block is not allocated, need to add neighbor to queue of "extra"
    // blocks
    if (inside_place[v] || inside_archived_place[v]) {
      const spatial_hash::VoxelNeighborSearch search(tsdf, 6);
      for (const auto& neighbor_key : search.neighborKeys({block_index, voxel_index})) {
        if (!tsdf.hasBlock(neighbor_key.first) &&
            !skip_add_blocks.count(neighbor_key.first)) {
          extra_blocks.push(neighbor_key.first);
          skip_add_blocks.insert(neighbor_key.first);
        }
      }
      continue;
    }

    if (skip_adding_frontiers) {
      return;
    }
    bool neighbor_free = false;
    bool neighbor_archived_free = false;
    checkFreeNeighbors(inside_place,
                       inside_archived_place,
                       tsdf.voxels_per_side,
                       voxel_index,
                       neighbor_free,
                       neighbor_archived_free);

    if (!neighbor_free && !neighbor_archived_free) {
      continue;
    }

    bool archived = std::find(input.archived_blocks.begin(),
                              input.archived_blocks.end(),
                              block_index) != input.archived_blocks.end();

    // A frontier is archived if its block is deallocated, or if its only neighbor
    // that's inside a place is in an archived place
    archived = archived || (!neighbor_free && neighbor_archived_free);
    if (archived) {
      archived_cloud->points.push_back({center.x(), center.y(), center.z()});
    } else {
      cloud->points.push_back({center.x(), center.y(), center.z()});
    }
  }
}

void FrontierExtractor::populateDenseFrontiers(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr archived_cloud,
    const double voxel_scale,
    const TsdfLayer& layer) {
  for (auto p : cloud->points) {
    Eigen::Vector3d center = {p.x, p.y, p.z};
    BlockIndex bix = layer.getBlockIndex(center.cast<float>());
    frontiers_.push_back(
        {center, {voxel_scale, voxel_scale, voxel_scale}, {1, 0, 0, 0}, 1, bix});
  }
  for (auto p : archived_cloud->points) {
    Eigen::Vector3d center = {p.x, p.y, p.z};
    BlockIndex bix = layer.getBlockIndex(center.cast<float>());
    archived_frontiers_.push_back({{p.x, p.y, p.z},
                                   {voxel_scale, voxel_scale, voxel_scale},
                                   {1, 0, 0, 0},
                                   1,
                                   bix});
  }
}

void FrontierExtractor::computeSparseFrontiers(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    const bool compute_frontier_shape,
    const TsdfLayer& layer,
    std::vector<Frontier>& frontiers) const {
  if (cloud->points.size() <= 0) {
    return;
  }
  std::vector<std::vector<Eigen::Vector3f>> frontiers_to_split;
  clusterFrontiers(cloud,
                   config.cluster_tolerance,
                   config.min_cluster_size,
                   config.max_cluster_size,
                   frontiers_to_split);

  std::vector<std::vector<Eigen::Vector3f>> finished_frontiers;
  splitAllFrontiers(frontiers_to_split,
                    config.frontier_splitting_threshold,
                    config.point_threshold,
                    finished_frontiers);

  for (auto f : finished_frontiers) {
    if (f.size() < config.culling_point_threshold) {
      continue;
    }

    Eigen::Vector3f centroid;
    Eigen::Matrix3f cov;
    centroidAndCovFromPoints(f, centroid, cov);

    BlockIndex bix = layer.getBlockIndex(centroid);

    if (compute_frontier_shape) {
      Eigen::EigenSolver<Eigen::Matrix3f> es(cov, true);
      auto evals_complex = es.eigenvalues();
      Eigen::Vector3f evals = evals_complex.real();
      Eigen::Vector3f scale = Eigen::sqrt(evals.array()) * 5;
      Eigen::Quaterniond quat(es.eigenvectors().real().cast<double>());

      frontiers.push_back(
          {centroid.cast<double>(), scale.cast<double>(), quat, f.size(), bix});
    } else {
      frontiers.push_back({centroid.cast<double>(), f.size(), bix});
    }
  }
}

void FrontierExtractor::detectFrontiers(const ReconstructionOutput& input,
                                        DynamicSceneGraph& graph,
                                        NearestNodeFinder& finder) {
  frontiers_.clear();
  archived_frontiers_.clear();

  for (auto b : input.archived_blocks) {
    recently_archived_blocks_.push_back(b);
  }

  const auto& tsdf = input.map().getTsdfLayer();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr archived_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  double min_frontier_z = input.world_t_body.z() + config.minimum_relative_z;
  double max_frontier_z = input.world_t_body.z() + config.maximum_relative_z;

  std::queue<BlockIndex> extra_blocks;
  IndexSet processed_extra_blocks;
  for (const auto& idx : tsdf.allocatedBlockIndices()) {
    processBlock(finder,
                 idx,
                 graph,
                 input,
                 archived_places_,
                 config.max_place_radius,
                 min_frontier_z,
                 max_frontier_z,
                 false,
                 processed_extra_blocks,
                 extra_blocks,
                 cloud,
                 archived_cloud);
  }

  while (!extra_blocks.empty()) {
    BlockIndex bix = extra_blocks.front();
    extra_blocks.pop();
    bool skip_adding_frontiers = false;
    if (std::find(recently_archived_blocks_.begin(),
                  recently_archived_blocks_.end(),
                  bix) != recently_archived_blocks_.end()) {
      skip_adding_frontiers = true;
    }

    processBlock(finder,
                 bix,
                 graph,
                 input,
                 archived_places_,
                 config.max_place_radius,
                 min_frontier_z,
                 max_frontier_z,
                 skip_adding_frontiers,
                 processed_extra_blocks,
                 extra_blocks,
                 cloud,
                 archived_cloud);
  }

  if (config.dense_frontiers) {
    populateDenseFrontiers(cloud, archived_cloud, tsdf.voxel_size, tsdf);
  } else {
    computeSparseFrontiers(cloud, config.compute_frontier_shape, tsdf, frontiers_);
    computeSparseFrontiers(
        archived_cloud, config.compute_frontier_shape, tsdf, archived_frontiers_);
  }

  archived_places_.clear();
}

void FrontierExtractor::addFrontiers(uint64_t timestamp_ns,
                                     DynamicSceneGraph& graph,
                                     NearestNodeFinder& finder) {
  for (auto nid_bix : nodes_to_remove_) {
    graph.removeNode(nid_bix.first);
  }
  nodes_to_remove_.clear();

  // Add non-archived frontiers and save their node ids for removing
  for (auto& frontier : frontiers_) {
    finder.find(frontier.center, 1, false, [&](NodeId place_id, size_t, double) {
      PlaceNodeAttributes::Ptr attrs(new PlaceNodeAttributes(1, 0));
      attrs->position = frontier.center;
      attrs->frontier_scale = frontier.scale;
      attrs->orientation = frontier.orientation;
      attrs->num_frontier_voxels = frontier.num_frontier_voxels;
      attrs->real_place = false;
      attrs->need_cleanup = true;
      attrs->last_update_time_ns = timestamp_ns;
      attrs->is_active = false;
      attrs->active_frontier = true;
      graph.emplaceNode(DsgLayers::PLACES, next_node_id_, std::move(attrs));
      graph.insertEdge(place_id, next_node_id_);
    });

    nodes_to_remove_.push_back({next_node_id_, frontier.block_index});
    ++next_node_id_;
  }

  // Add archived frontiers
  for (auto& frontier : archived_frontiers_) {
    finder.find(frontier.center, 1, false, [&](NodeId place_id, size_t, double) {
      PlaceNodeAttributes::Ptr attrs(new PlaceNodeAttributes(1, 0));
      attrs->position = frontier.center;
      attrs->frontier_scale = frontier.scale;
      attrs->orientation = frontier.orientation;
      attrs->num_frontier_voxels = frontier.num_frontier_voxels;
      attrs->real_place = false;
      attrs->need_cleanup = true;
      attrs->last_update_time_ns = timestamp_ns;
      attrs->is_active = false;
      attrs->active_frontier = false;
      graph.emplaceNode(DsgLayers::PLACES, next_node_id_, std::move(attrs));
      graph.insertEdge(place_id, next_node_id_);
    });
    ++next_node_id_;
  }
}

void FrontierExtractor::updateRecentBlocks(Eigen::Vector3d current_position,
                                           double block_size) {
  BlockIndices updated_archived_blocks;
  for (BlockIndex bix : recently_archived_blocks_) {
    Eigen::Vector3f block_origin = spatial_hash::originPointFromIndex(bix, block_size);
    if ((block_origin - current_position.cast<float>()).norm() <
        config.recent_block_distance) {
      updated_archived_blocks.push_back(bix);
    }
  }
  recently_archived_blocks_ = updated_archived_blocks;
}

void declare_config(FrontierExtractor::Config& config) {
  using namespace config;
  name("FrontierExtractor::Config");
  field<CharConversion>(config.prefix, "prefix");
  field(config.cluster_tolerance, "cluster_tolerance");
  field(config.min_cluster_size, "min_cluster_size");
  field(config.max_cluster_size, "max_cluster_size");
  field(config.max_place_radius, "max_place_radius");
  field(config.dense_frontiers, "dense_frontiers");
  field(config.frontier_splitting_threshold, "frontier_splitting_threshold");
  field(config.point_threshold, "point_threshold");
  field(config.culling_point_threshold, "culling_point_threshold");
  field(config.recent_block_distance, "recent_block_distance");
  field(config.minimum_relative_z, "minimum_relative_z");
  field(config.maximum_relative_z, "maximum_relative_z");
  field(config.compute_frontier_shape, "compute_frontier_shape");
}

}  // namespace hydra
