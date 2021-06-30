#pragma once
#include "kimera_topology/gvd_voxel.h"
#include "kimera_topology/voxblox_types.h"
#include "kimera_topology/voxel_aware_mesh_integrator.h"

#include <utility>

namespace kimera {
namespace topology {

struct GvdIntegratorConfig {
  FloatingPoint max_distance_m = 2.0;
  FloatingPoint min_distance_m = 0.2;
  FloatingPoint min_diff_m = 1.0e-3;
  FloatingPoint min_weight = 1.0e-6;
  double voronoi_min_distance_m = 0.2;
  double voronoi_neighbor_l1_separation = 2.0;
  int num_buckets = 20;
  bool multi_queue = false;
  bool positive_distance_only = true;
  bool parent_derived_distance = true;
  voxblox::MeshIntegratorConfig mesh_integrator_config;
};

/**
 * @brief Tracking statistics for what the integrator did
 */
struct UpdateStatistics {
  size_t number_lowered_voxels;
  size_t number_raised_voxels;
  size_t number_new_voxels;
  size_t number_raise_updates;
  size_t number_voronoi_found;
  size_t number_lower_skipped;
  size_t number_lower_updated;

  void clear();
};

/**
 * An ESDF and GVD integrator based on https://arxiv.org/abs/1611.03631
 */
class GvdIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GvdIntegrator(const GvdIntegratorConfig& config,
                const Layer<TsdfVoxel>::Ptr& tsdf_layer,
                const Layer<GvdVoxel>::Ptr& gvd_layer,
                const MeshLayer::Ptr& mesh_layer);

  virtual ~GvdIntegrator() = default;

  void updateFromTsdfLayer(bool clear_updated_flag);

 protected:
  void processTsdfBlock(const Block<TsdfVoxel>& block, const BlockIndex& index);

  void processRaiseSet();

  void processLowerSet();

  void updateFromTsdfBlocks(const BlockIndexList& tsdf_blocks);

  bool updateVoxelFromNeighbors(const GlobalIndex& index, GvdVoxel& voxel);

  bool processNeighbor(GvdVoxel& voxel,
                       const GlobalIndex& voxel_idx,
                       FloatingPoint neighbor_distance,
                       const GlobalIndex& neighbor_idx,
                       GvdVoxel& neighbor);

  void updateVoronoiQueue(GvdVoxel& curr_voxel,
                          const GlobalIndex& curr_pos,
                          GvdVoxel& neighbor_voxel,
                          const GlobalIndex& neighbor_pos);

  void updateUnobservedGvdVoxel(const TsdfVoxel& tsdf_voxel,
                                const GlobalIndex& index,
                                GvdVoxel& gvd_voxel);

  void updateObservedGvdVoxel(const TsdfVoxel& tsdf_voxel,
                              const GlobalIndex& index,
                              GvdVoxel& gvd_voxel);

  void setFixedParent(const Neighborhood<>::IndexMatrix& neighbor_indices,
                      GvdVoxel& voxel);

 protected:
  std::unique_ptr<VoxelAwareMeshIntegrator> mesh_integrator_;

  enum class PushType {
    NEW,
    LOWER,
    RAISE,
    BOTH,
  };

  UpdateStatistics update_stats_;

  GvdIntegratorConfig config_;
  Layer<TsdfVoxel>::Ptr tsdf_layer_;
  Layer<GvdVoxel>::Ptr gvd_layer_;
  MeshLayer::Ptr mesh_layer_;

  BucketQueue<GlobalIndex> lower_;

  AlignedQueue<GlobalIndex> raise_;

  FloatingPoint voxel_size_;

 protected:
  inline void pushToQueue(const GlobalIndex& index, GvdVoxel& voxel, PushType action) {
    switch (action) {
      case PushType::NEW:
        voxel.in_queue = true;
        lower_.push(index, voxel.distance);
        update_stats_.number_new_voxels++;
        break;
      case PushType::LOWER:
        voxel.in_queue = true;
        lower_.push(index, voxel.distance);
        update_stats_.number_lowered_voxels++;
        break;
      case PushType::RAISE:
        raise_.push(index);
        update_stats_.number_raised_voxels++;
        break;
      case PushType::BOTH:
        voxel.in_queue = true;
        lower_.push(index, voxel.distance);
        raise_.push(index);
        update_stats_.number_raised_voxels++;
        break;
      default:
        LOG(FATAL) << "Invalid push type!";
        break;
    }
  }

  inline GlobalIndex popFromLower() {
    GlobalIndex index = lower_.front();
    lower_.pop();
    return index;
  }

  inline GlobalIndex popFromRaise() {
    GlobalIndex index = raise_.front();
    raise_.pop();
    return index;
  }

  inline void setDefaultDistance(GvdVoxel& voxel, double signed_distance) {
    voxel.distance = std::copysign(config_.max_distance_m, signed_distance);
  }

  inline bool isTsdfFixed(const TsdfVoxel& voxel) {
    return voxel.distance < config_.min_distance_m;
  }

  inline bool voxelHasDistance(const GvdVoxel& voxel) {
    if (!voxel.observed) {
      return false;
    }

    if (voxel.distance >= config_.max_distance_m) {
      return false;
    }

    if (config_.positive_distance_only && voxel.distance < 0.0) {
      return false;
    }

    if (voxel.distance <= -config_.max_distance_m) {
      return false;
    }

    return true;
  }

  inline voxblox::Point getVoxelCoordinates(const GlobalIndex& index) {
    BlockIndex block_idx;
    VoxelIndex voxel_idx;
    voxblox::getBlockAndVoxelIndexFromGlobalVoxelIndex(
        index, gvd_layer_->voxels_per_side(), &block_idx, &voxel_idx);

    return gvd_layer_->getBlockByIndex(block_idx).computeCoordinatesFromVoxelIndex(
        voxel_idx);
  }
};

}  // namespace topology
}  // namespace kimera
