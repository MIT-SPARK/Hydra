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
#pragma once
#include "hydra_topology/graph_extractor.h"
#include "hydra_topology/gvd_utilities.h"
#include "hydra_topology/gvd_voxel.h"
#include "hydra_topology/voxblox_types.h"
#include "hydra_topology/voxel_aware_mesh_integrator.h"

#include <utility>

namespace hydra {
namespace topology {

struct GvdIntegratorConfig {
  FloatingPoint max_distance_m = 2.0;
  FloatingPoint min_distance_m = 0.2;
  FloatingPoint min_diff_m = 1.0e-3;
  FloatingPoint min_weight = 1.0e-6;
  int num_buckets = 20;
  bool multi_queue = false;
  bool positive_distance_only = true;
  bool parent_derived_distance = true;
  uint8_t min_basis_for_extraction = 3;
  VoronoiCheckConfig voronoi_config;
  voxblox::MeshIntegratorConfig mesh_integrator_config;
  GraphExtractorConfig graph_extractor_config;
  bool extract_graph = true;
  bool mesh_only = false;
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
  size_t number_fixed_no_parent;
  size_t number_force_lowered;

  void clear();
};

/**
 * An ESDF and GVD integrator based on https://arxiv.org/abs/1611.03631
 */
class GvdIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GvdIntegrator(const GvdIntegratorConfig& config,
                Layer<TsdfVoxel>* tsdf_layer,
                const Layer<GvdVoxel>::Ptr& gvd_layer,
                const MeshLayer::Ptr& mesh_layer);

  virtual ~GvdIntegrator() = default;

  void updateFromTsdfLayer(bool clear_updated_flag,
                           bool clear_surface_flag = true,
                           bool use_all_blocks = false);

  inline const SceneGraphLayer& getGraph() const {
    return graph_extractor_->getGraph();
  }

  inline GraphExtractor& getGraphExtractor() const { return *graph_extractor_; }

  BlockIndexList removeDistantBlocks(const voxblox::Point& center, double max_distance);

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

  void setFixedParent(const GvdNeighborhood::IndexMatrix& neighbor_indices,
                      GvdVoxel& voxel);

  void raiseVoxel(GvdVoxel& voxel, const GlobalIndex& voxel_index);

  uint8_t updateGvdParentMap(const GlobalIndex& voxel_index, const GvdVoxel& neighbor);

  void updateNearestParent(const GvdVoxel& voxel,
                           const GlobalIndex& voxel_index,
                           const GvdVoxel& neighbor,
                           const GlobalIndex& parent);

  void removeVoronoiFromGvdParentMap(const GlobalIndex& voxel_index);

  void updateGvdVoxel(const GlobalIndex& voxel_index, GvdVoxel& voxel, GvdVoxel& other);

  void clearGvdVoxel(const GlobalIndex& index, GvdVoxel& voxel);

  void updateVertexMapping();

  void markNewGvdParent(const GlobalIndex& parent);

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
  Layer<TsdfVoxel>* tsdf_layer_;
  Layer<GvdVoxel>::Ptr gvd_layer_;
  MeshLayer::Ptr mesh_layer_;

  GvdParentMap gvd_parents_;
  GvdVertexMap gvd_parent_vertices_;

  GraphExtractor::Ptr graph_extractor_;

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
    return std::abs(voxel.distance) < config_.min_distance_m;
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
};

}  // namespace topology
}  // namespace hydra
