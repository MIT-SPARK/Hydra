#pragma once
#include "kimera_topology/voxblox_types.h"

#include <utility>

namespace kimera {
namespace topology {

struct GvdIntegratorConfig {
  double sma_angle = M_PI / 2.0;
  double sma_truncation_multiplier = 1.5;
  bool full_euclidean_distance = false;
  FloatingPoint max_distance_m = 2.0;
  FloatingPoint min_distance_m = 0.2;
  FloatingPoint min_diff_m = 1.0e-3;
  FloatingPoint min_weight = 1.0e-6;
  int num_buckets = 20;
  bool multi_queue = false;
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
                const Layer<GvdVoxel>::Ptr& gvd_layer);

  virtual ~GvdIntegrator() = default;

  void updateFromTsdfLayer(bool clear_updated_flag);

 protected:
  void processTsdfBlock(const Block<TsdfVoxel>& block, const BlockIndex& index);

  void processRaiseSet();

  void processOpenSet();

  void updateFromTsdfBlocks(const BlockIndexList& tsdf_blocks);

  bool updateVoxelFromNeighbors(const GlobalIndex& index, GvdVoxel& voxel);

  // TODO(nathan) this is getting hairy
  bool processNeighbor(GvdVoxel& voxel,
                       const GlobalIndex& voxel_idx,
                       unsigned int neighbor_linear_index,
                       const GlobalIndex& neighbor_idx,
                       GvdVoxel& neighbor);

  void updateMedialAxisQueue(GvdVoxel& curr_voxel,
                             const GlobalIndex& curr_idx,
                             GvdVoxel& neighbor_voxel,
                             const GlobalIndex& neighbor_idx,
                             const SignedIndex& direction);

  void updateUnobservedGvdVoxel(const TsdfVoxel& tsdf_voxel,
                                const GlobalIndex& index,
                                GvdVoxel& gvd_voxel);

  void updateObservedGvdVoxel(const TsdfVoxel& tsdf_voxel,
                              const GlobalIndex& index,
                              GvdVoxel& gvd_voxel);

 protected:
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

  BucketQueue<GlobalIndex> open_;

  AlignedQueue<GlobalIndex> raise_;

  double sma_threshold_;
  double sma_min_distance_;
  FloatingPoint voxel_size_;

 protected:
  inline void pushToQueue(const GlobalIndex& index, GvdVoxel& voxel, PushType action) {
    switch (action) {
      case PushType::NEW:
        voxel.in_queue = true;
        open_.push(index, voxel.distance);
        update_stats_.number_new_voxels++;
        break;
      case PushType::LOWER:
        voxel.in_queue = true;
        open_.push(index, voxel.distance);
        update_stats_.number_lowered_voxels++;
        break;
      case PushType::RAISE:
        raise_.push(index);
        update_stats_.number_raised_voxels++;
        break;
      case PushType::BOTH:
        voxel.in_queue = true;
        open_.push(index, voxel.distance);
        raise_.push(index);
        update_stats_.number_raised_voxels++;
        break;
      default:
        LOG(FATAL) << "Invalid push type!";
        break;
    }
  }

  inline GlobalIndex popFromOpen() {
    GlobalIndex index = open_.front();
    open_.pop();
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

  inline bool voxelHasDistance(const GvdVoxel& voxel) {
    if (!voxel.observed) {
      return false;
    }

    if (voxel.distance >= config_.max_distance_m) {
      return false;
    }

    // TODO(nathan) asymmetric max distance
    if (voxel.distance <= -config_.max_distance_m) {
      return false;
    }

    return true;
  }
};

}  // namespace topology
}  // namespace kimera
