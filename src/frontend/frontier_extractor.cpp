#define PCL_NO_PRECOMPILE
#include "hydra/frontend/frontier_extractor.h"

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

//#include <pcl/impl/point_types.hpp>

namespace hydra {

FrontierExtractor::FrontierExtractor() : next_node_id_('f', 0) {}

void FrontierExtractor::detectFrontiers(const ReconstructionOutput& input,
                                        DynamicSceneGraph& graph,
                                        NearestNodeFinder& finder) {
  frontier_positions_.clear();
  frontier_distances_.clear();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& idx : input.archived_blocks) {
    voxblox::Block<voxblox::TsdfVoxel>::Ptr block = input.tsdf->getBlockPtrByIndex(idx);

    // find all active places that are close to the block
    std::vector<std::pair<Eigen::Vector3d, double>> center_dists;
    finder.findRadius(
        block->origin().cast<double>(),
        block->block_size() * 1.414 + 5,
        false,
        [&](NodeId pid, size_t, double) {
          PlaceNodeAttributes pattr =
              graph.getNode(pid).value().get().attributes<PlaceNodeAttributes>();
          center_dists.push_back({pattr.position, pattr.distance});
        });

    // find all voxels that are inside a place
    bool inside_place[16][16][16];
    for (size_t v = 0; v < block->num_voxels(); ++v) {
      voxblox::VoxelIndex ind = block->computeVoxelIndexFromLinearIndex(v);
      voxblox::Point center = block->computeCoordinatesFromLinearIndex(v);
      inside_place[ind.x()][ind.y()][ind.z()] = false;
      for (auto cd : center_dists) {
        if ((center - cd.first.cast<float>()).norm() <= cd.second) {
          inside_place[ind.x()][ind.y()][ind.z()] = true;
          break;
        }
      }
    }

    // find voxels that are on boundary of unobserved space and space inside a place
    for (size_t v = 0; v < block->num_voxels(); ++v) {
      const voxblox::TsdfVoxel& voxel = block->getVoxelByLinearIndex(v);

      if (voxel.weight >= 1e-6) {
        continue;
      }

      voxblox::VoxelIndex ind = block->computeVoxelIndexFromLinearIndex(v);
      int i = ind.x();
      int j = ind.y();
      int k = ind.z();

      if (inside_place[i][j][k]) {
        continue;
      }
      int offsets[3] = {-1, 0, 1};
      bool neighbor_free = false;
      for (int di : offsets) {
        for (int dj : offsets) {
          for (int dk : offsets) {
            if (!block->isValidVoxelIndex({i + di, j + dj, k + dk})) {
              continue;
            }
            neighbor_free = inside_place[i + di][j + dj][k + dk];
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
      if (neighbor_free) {
        voxblox::Point center = block->computeCoordinatesFromLinearIndex(v);
        cloud->points.push_back({center.x(), center.y(), center.z()});
        // frontier_positions_.push_back({center.x(), center.y(), center.z()});
        // frontier_distances_.push_back(voxel.distance);
      }
    }
  }

  if (cloud->points.size() == 0) {
    return;
  } else {
    LOG(WARNING) << "cloud size: " << cloud->points.size();
  }
  pcl::IndicesPtr cloud_indices(new pcl::Indices);
  cloud_indices->resize(cloud->points.size());
  for (int i = 0; i < cloud->points.size(); ++i) {
    cloud_indices->at(i) = i;
  }
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  tree->setInputCloud(cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> estimator;
  estimator.setClusterTolerance(.3);
  estimator.setMinClusterSize(15);
  estimator.setMaxClusterSize(300);
  estimator.setSearchMethod(tree);
  estimator.setInputCloud(cloud);
  estimator.setIndices(cloud_indices);

  std::vector<pcl::PointIndices> cluster_indices;
  estimator.extract(cluster_indices);

  std::vector<std::pair<Eigen::Vector3d, int>> frontier_clusters;

  frontier_clusters.resize(cluster_indices.size());
  LOG(WARNING) << "N clusters: " << cluster_indices.size();
  for (size_t k = 0; k < frontier_clusters.size(); ++k) {
    pcl::CentroidPoint<pcl::PointXYZ> centroid;
    for (const auto& ind : cluster_indices.at(k).indices) {
      centroid.add(cloud->points.at(ind));
    }
    pcl::PointXYZ c;
    centroid.get(c);
    frontier_positions_.push_back({c.x, c.y, c.z});
    frontier_distances_.push_back(cluster_indices.at(k).indices.size());
  }
}

// void FrontierExtractor::addFrontiers(uint64_t timestamp_ns,
//                                     DynamicSceneGraph& graph,
//                                     NearestNodeFinder& finder) {
//  for (size_t ix = 0; ix < frontier_positions_.size(); ++ix) {
//    finder.find(
//        frontier_positions_.at(ix), 1, false, [&](NodeId place_id, size_t, double) {
//          Eigen::Vector3d x_frontier = frontier_positions_.at(ix);
//          PlaceNodeAttributes place_attrs =
//              graph.getNode(place_id).value().get().attributes<PlaceNodeAttributes>();
//          Eigen::Vector3d x_place = place_attrs.position;
//          Eigen::Vector3d r_place_frontier = x_frontier - x_place;
//          if (r_place_frontier.norm() < place_attrs.distance) {
//            Eigen::Vector3d u = r_place_frontier / (r_place_frontier).norm();
//            x_frontier = x_place + u * place_attrs.distance * 1.2;
//          }
//
//          //
//          std::vector<size_t> conflicting_nodes;
//          finder.findRadius(x_frontier, 8, false, [&](NodeId pid, size_t, double) {
//            PlaceNodeAttributes pattr =
//                graph.getNode(pid).value().get().attributes<PlaceNodeAttributes>();
//            if ((pattr.position - x_frontier).norm() < pattr.distance) {
//              conflicting_nodes.push_back(pid);
//            }
//          });
//          if (conflicting_nodes.size() > 0) {
//            return;
//          }
//          //
//
//          double d = frontier_distances_.at(ix);
//          PlaceNodeAttributes::Ptr attrs(new PlaceNodeAttributes(d, 0));
//          attrs->position = x_frontier;
//          attrs->real_place = false;
//          attrs->need_cleanup = true;
//          attrs->last_update_time_ns = timestamp_ns;
//          graph.emplaceNode(DsgLayers::PLACES, next_node_id_, std::move(attrs));
//          graph.insertEdge(place_id, next_node_id_);
//        });
//
//    ++next_node_id_;
//
//    // Note that the frontier in the frontend scene graph might represent a
//    // location that we have observed previously, but not during the same active
//    // window. The check to ensure we don't add a frontier that corresponds to an
//    // area we have already visited happens in the backend.
//  }
//}

void FrontierExtractor::addFrontiers(uint64_t timestamp_ns,
                                     DynamicSceneGraph& graph,
                                     NearestNodeFinder& finder) {
  for (size_t ix = 0; ix < frontier_positions_.size(); ++ix) {
    finder.find(
        frontier_positions_.at(ix), 1, false, [&](NodeId place_id, size_t, double) {
          double d = frontier_distances_.at(ix);
          PlaceNodeAttributes::Ptr attrs(new PlaceNodeAttributes(d, 0));
          attrs->position = frontier_positions_.at(ix);
          attrs->real_place = false;
          attrs->need_cleanup = true;
          attrs->last_update_time_ns = timestamp_ns;
          attrs->is_active = false;
          graph.emplaceNode(DsgLayers::PLACES, next_node_id_, std::move(attrs));
          graph.insertEdge(place_id, next_node_id_);
        });

    ++next_node_id_;
  }
}

}  // namespace hydra
