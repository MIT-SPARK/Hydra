#include <config_utilities/config_utilities.h>
#include <spark_dsg/dynamic_scene_graph_layer.h>
#include <voxblox/core/block.h>
#include <voxblox/core/common.h>
#include <voxblox/core/voxel.h>

#include "hydra/common/config_utilities.h"
// #include "hydra/frontend/frontier_extractor_config.h"
#define PCL_NO_PRECOMPILE
#include <hydra/utils/nearest_neighbor_utilities.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <spark_dsg/dynamic_scene_graph.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/scene_graph_types.h>

#include <queue>

#include "hydra/frontend/frontier_extractor.h"

namespace hydra {

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

void frontiersToCenters(const std::vector<Eigen::Vector3f>& positions,
                        Eigen::Vector3d& center) {
  center.setZero();
  for (auto p : positions) {
    center += p.cast<double>();
  }
  center /= positions.size();
}

void centroidAndCovFromPoints(const std::vector<Eigen::Vector3f>& positions,
                              Eigen::Vector3f& centroid,
                              Eigen::Matrix3f& cov) {
  centroid.setZero();
  for (auto p : positions) {
    centroid += p;
  }
  centroid /= positions.size();

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
    : config_(config), next_node_id_(config.prefix, 0) {}

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

template <typename T>
void checkNeighborAndAddBlock(const int i,
                              const int j,
                              const int k,
                              const voxblox::BlockIndex& block_index,
                              const typename voxblox::Layer<T>::Ptr& layer,
                              std::vector<voxblox::BlockIndex>& blocks_to_skip,
                              std::queue<voxblox::BlockIndex>& extra_blocks) {
  voxblox::GlobalIndex gvi = voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
      block_index, {i, j, k}, layer->voxels_per_side());
  voxblox::BlockIndex bix =
      voxblox::getBlockIndexFromGlobalVoxelIndex(gvi, layer->voxels_per_side_inv());
  if (!layer->hasBlock(bix)) {
    auto it = find(blocks_to_skip.begin(), blocks_to_skip.end(), bix);
    if (it == blocks_to_skip.end()) {
      extra_blocks.push(bix);
      blocks_to_skip.push_back(bix);
    }
  }
}

voxblox::VoxelIndex computeVoxelIndexFromLinearIndex(size_t linear_index,
                                                     int voxels_per_side) {
  int rem = linear_index;
  voxblox::VoxelIndex result;
  std::div_t div_temp = std::div(rem, voxels_per_side * voxels_per_side);
  rem = div_temp.rem;
  result.z() = div_temp.quot;
  div_temp = std::div(rem, voxels_per_side);
  result.y() = div_temp.quot;
  result.x() = div_temp.rem;
  return result;
}

size_t computeLinearIndexFromVoxelIndex(const voxblox::VoxelIndex& index,
                                        const size_t voxels_per_side) {
  size_t linear_index = static_cast<size_t>(
      index.x() + voxels_per_side * (index.y() + index.z() * voxels_per_side));
  return linear_index;
}

std::vector<std::pair<Eigen::Vector3d, double>> getPlacesForBlock(
    const DynamicSceneGraph& graph,
    const Eigen::Vector3f& block_origin,
    NearestNodeFinder& finder,
    const double block_size,
    const float voxel_size,
    const double max_place_radius) {
  std::vector<std::pair<Eigen::Vector3d, double>> center_dists;
  Eigen::Vector3f block_relative_center =
      voxblox::getCenterPointFromGridIndex<Eigen::Vector3f>(
          {voxel_size / 2.0f, voxel_size / 2.0f, voxel_size / 2.0f}, voxel_size);
  Eigen::Vector3f global_block_center = block_origin + block_relative_center;
  finder.findRadius(global_block_center.cast<double>(),
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
    voxblox::VoxelIndex ind = computeVoxelIndexFromLinearIndex(v, voxels_per_side);
    voxblox::Point center =
        block_origin + voxblox::getCenterPointFromGridIndex(ind, voxel_size);
    inside_place[v] = false;
    for (auto cd : center_dists) {
      if ((center - cd.first.cast<float>()).norm() <= cd.second) {
        inside_place[v] = true;
        break;
      }
    }
    inside_archived_place[v] = false;
    if (inside_place[v]) {
      continue;
    }
    for (auto cd : archived_center_dists) {
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
                        const int i,
                        const int j,
                        const int k,
                        bool& neighbor_free,
                        bool& neighbor_archived_free) {
  int offsets[3] = {-1, 0, 1};
  for (int di : offsets) {
    for (int dj : offsets) {
      for (int dk : offsets) {
        if (i + di < 0 || i + di >= vps || j + dj < 0 || j + dj >= vps || k + dk < 0 ||
            k + dk >= vps) {
          continue;
        }

        size_t vn = computeLinearIndexFromVoxelIndex({i + di, j + dj, k + dk}, vps);
        neighbor_free = inside_place[vn];
        neighbor_archived_free |= inside_archived_place[vn];
        if (neighbor_free) {
          break;
        }
      }
      if (neighbor_free) {
        break;
      }
    }
    if (neighbor_free) {
      break;
    }
  }
}

template <typename T>
void processBlock(NearestNodeFinder& finder,
                  const voxblox::BlockIndex& block_index,
                  const DynamicSceneGraph& graph,
                  const ReconstructionOutput& input,
                  const std::vector<NodeId> archived_places,
                  const double max_place_radius,
                  std::vector<voxblox::BlockIndex>& skip_add_blocks,
                  std::queue<voxblox::BlockIndex>& extra_blocks,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr archived_cloud) {
  double voxel_size = input.tsdf->voxel_size();
  double block_size = input.tsdf->block_size();
  // Get all active places near block
  Eigen::Vector3f block_origin =
      voxblox::getOriginPointFromGridIndex(block_index, block_size);
  auto center_dists = getPlacesForBlock(
      graph, block_origin, finder, block_size, voxel_size, max_place_radius);

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
  int vps = input.tsdf->voxels_per_side();
  size_t voxels_per_block = std::pow(vps, 3);
  std::vector<bool> inside_place;
  inside_place.resize(voxels_per_block);
  std::vector<bool> inside_archived_place;
  inside_archived_place.resize(voxels_per_block);

  computeVoxelsInPlace(block_origin,
                       center_dists,
                       archived_center_dists,
                       voxels_per_block,
                       vps,
                       voxel_size,
                       inside_place,
                       inside_archived_place);

  // find voxels that are on boundary of unobserved space and space inside a place
  for (size_t v = 0; v < voxels_per_block; ++v) {
    // We only need to check if the voxel has been observed if the block is allocated
    if (input.tsdf->hasBlock(block_index)) {
      voxblox::Block<voxblox::TsdfVoxel>::Ptr block =
          input.tsdf->getBlockPtrByIndex(block_index);
      const voxblox::TsdfVoxel& voxel = block->getVoxelByLinearIndex(v);

      if (voxel.weight >= 1e-6) {
        continue;
      }
    }

    voxblox::VoxelIndex ind = computeVoxelIndexFromLinearIndex(v, vps);
    int i = ind.x();
    int j = ind.y();
    int k = ind.z();

    // If voxel is on the "border" of the block, and it is inside a place, and the
    // neighboring block is not allocated, need to add neighbor to queue of "extra"
    // blocks
    if (inside_place[v] || inside_archived_place[v]) {
      checkNeighborAndAddBlock<voxblox::TsdfVoxel>(
          i + 1, j, k, block_index, input.tsdf, skip_add_blocks, extra_blocks);
      checkNeighborAndAddBlock<voxblox::TsdfVoxel>(
          i - 1, j, k, block_index, input.tsdf, skip_add_blocks, extra_blocks);
      checkNeighborAndAddBlock<voxblox::TsdfVoxel>(
          i, j + 1, k, block_index, input.tsdf, skip_add_blocks, extra_blocks);
      checkNeighborAndAddBlock<voxblox::TsdfVoxel>(
          i, j - 1, k, block_index, input.tsdf, skip_add_blocks, extra_blocks);
      checkNeighborAndAddBlock<voxblox::TsdfVoxel>(
          i, j, k + 1, block_index, input.tsdf, skip_add_blocks, extra_blocks);
      checkNeighborAndAddBlock<voxblox::TsdfVoxel>(
          i, j, k - 1, block_index, input.tsdf, skip_add_blocks, extra_blocks);

      continue;
    }

    bool neighbor_free = false;
    bool neighbor_archived_free = false;
    checkFreeNeighbors(inside_place,
                       inside_archived_place,
                       vps,
                       i,
                       j,
                       k,
                       neighbor_free,
                       neighbor_archived_free);

    if (neighbor_free || neighbor_archived_free) {
      auto it = std::find(
          input.archived_blocks.begin(), input.archived_blocks.end(), block_index);

      bool archived = false;
      if (it != input.archived_blocks.end()) {
        archived = true;
      }
      // A frontier is archived if its block is deallocated, or if its only neighbor
      // that's inside a place is in an archived place
      archived = archived || (!neighbor_free && neighbor_archived_free);
      Eigen::Vector3f relative_center =
          voxblox::getCenterPointFromGridIndex<Eigen::Vector3i>({i, j, k}, voxel_size);
      Eigen::Vector3f center = block_origin + relative_center;

      if (archived) {
        archived_cloud->points.push_back({center.x(), center.y(), center.z()});

      } else {
        cloud->points.push_back({center.x(), center.y(), center.z()});
      }
    }
  }
}

void FrontierExtractor::populateDenseFrontiers(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr archived_cloud,
    const double voxel_scale) {
  for (auto p : cloud->points) {
    frontiers_.push_back(
        {{p.x, p.y, p.z}, {voxel_scale, voxel_scale, voxel_scale}, {1, 0, 0, 0}, 1});
  }
  for (auto p : archived_cloud->points) {
    archived_frontiers_.push_back(
        {{p.x, p.y, p.z}, {voxel_scale, voxel_scale, voxel_scale}, {1, 0, 0, 0}, 1});
  }
}

void computeSparseFrontiers(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                            const FrontierExtractor::Config config,
                            std::vector<Frontier>& frontiers) {
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
    Eigen::Vector3f centroid;
    Eigen::Matrix3f cov;
    centroidAndCovFromPoints(f, centroid, cov);

    Eigen::EigenSolver<Eigen::Matrix3f> es(cov, true);
    auto evals_complex = es.eigenvalues();
    Eigen::Vector3f evals = evals_complex.real();
    Eigen::Vector3f scale = Eigen::sqrt(evals.array()) * 5;
    Eigen::Quaterniond quat(es.eigenvectors().real().cast<double>());

    frontiers.push_back(
        {centroid.cast<double>(), scale.cast<double>(), quat, f.size()});
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

  voxblox::BlockIndexList allocated_blocks;
  input.tsdf->getAllAllocatedBlocks(&allocated_blocks);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr archived_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<NodeId> block_is_archived;

  std::queue<voxblox::BlockIndex> extra_blocks;
  std::vector<voxblox::BlockIndex> processed_extra_blocks;

  for (const auto& idx : allocated_blocks) {
    voxblox::Block<voxblox::TsdfVoxel>::Ptr block = input.tsdf->getBlockPtrByIndex(idx);

    processBlock<voxblox::TsdfVoxel>(finder,
                                     block->block_index(),
                                     graph,
                                     input,
                                     archived_places_,
                                     config_.max_place_radius,
                                     processed_extra_blocks,
                                     extra_blocks,
                                     cloud,
                                     archived_cloud);
  }

  while (!extra_blocks.empty()) {
    voxblox::BlockIndex bix = extra_blocks.front();
    extra_blocks.pop();
    if (std::find(recently_archived_blocks_.begin(),
                  recently_archived_blocks_.end(),
                  bix) != recently_archived_blocks_.end()) {
      continue;
    }

    processBlock<voxblox::TsdfVoxel>(finder,
                                     bix,
                                     graph,
                                     input,
                                     archived_places_,
                                     config_.max_place_radius,
                                     processed_extra_blocks,
                                     extra_blocks,
                                     cloud,
                                     archived_cloud);
  }

  if (config_.dense_frontiers) {
    populateDenseFrontiers(cloud, archived_cloud, input.tsdf->voxel_size());
  } else {
    computeSparseFrontiers(cloud, config_, frontiers_);
    computeSparseFrontiers(archived_cloud, config_, archived_frontiers_);
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
  for (size_t ix = 0; ix < frontiers_.size(); ++ix) {
    finder.find(
        std::get<0>(frontiers_.at(ix)), 1, false, [&](NodeId place_id, size_t, double) {
          PlaceNodeAttributes::Ptr attrs(new PlaceNodeAttributes(1, 0));
          attrs->position = std::get<0>(frontiers_.at(ix));
          attrs->frontier_scale = std::get<1>(frontiers_.at(ix));
          attrs->orientation = std::get<2>(frontiers_.at(ix));
          attrs->num_frontier_voxels = std::get<3>(frontiers_.at(ix));
          attrs->real_place = false;
          attrs->need_cleanup = true;
          attrs->last_update_time_ns = timestamp_ns;
          attrs->is_active = false;
          attrs->active_frontier = true;
          graph.emplaceNode(DsgLayers::PLACES, next_node_id_, std::move(attrs));
          graph.insertEdge(place_id, next_node_id_);
        });

    nodes_to_remove_.push_back({next_node_id_, voxblox::BlockIndex()});
    ++next_node_id_;
  }

  // Add archived frontiers
  for (size_t ix = 0; ix < archived_frontiers_.size(); ++ix) {
    finder.find(std::get<0>(archived_frontiers_.at(ix)),
                1,
                false,
                [&](NodeId place_id, size_t, double) {
                  PlaceNodeAttributes::Ptr attrs(new PlaceNodeAttributes(1, 0));
                  attrs->position = std::get<0>(archived_frontiers_.at(ix));
                  attrs->frontier_scale = std::get<1>(archived_frontiers_.at(ix));
                  attrs->orientation = std::get<2>(archived_frontiers_.at(ix));
                  attrs->num_frontier_voxels = std::get<3>(archived_frontiers_.at(ix));
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
  std::vector<voxblox::BlockIndex> updated_archived_blocks;
  for (voxblox::BlockIndex bix : recently_archived_blocks_) {
    Eigen::Vector3f block_origin =
        voxblox::getOriginPointFromGridIndex(bix, block_size);
    if ((block_origin - current_position.cast<float>()).norm() <
        config_.recent_block_distance) {
      updated_archived_blocks.push_back(bix);
    }
  }
  recently_archived_blocks_ = updated_archived_blocks;
}

void declare_config(FrontierExtractor::Config& config) {
  using namespace config;
  name("FrontierExtractorConfig");
  field<CharConversion>(config.prefix, "prefix");
  field(config.cluster_tolerance, "cluster_tolerance");
  field(config.min_cluster_size, "min_cluster_size");
  field(config.max_cluster_size, "max_cluster_size");
  field(config.max_place_radius, "max_place_radius");
  field(config.dense_frontiers, "dense_frontiers");
  field(config.frontier_splitting_threshold, "frontier_splitting_threshold");
  field(config.point_threshold, "point_threshold");
  field(config.recent_block_distance, "recent_block_distance");
}

}  // namespace hydra
