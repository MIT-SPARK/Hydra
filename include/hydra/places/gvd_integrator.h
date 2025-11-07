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

#include <spatial_hash/neighbor_utils.h>

#include <utility>

#include "hydra/places/graph_extractor.h"
#include "hydra/places/gvd_integrator_config.h"
#include "hydra/places/gvd_parent_tracker.h"
#include "hydra/places/gvd_utilities.h"
#include "hydra/places/gvd_voxel.h"
#include "hydra/places/update_statistics.h"
#include "hydra/reconstruction/voxel_types.h"
#include "hydra/utils/bucket_queue.h"
#include "hydra/utils/logging.h"

namespace hydra::places {
namespace gvd {

bool setFixedParent(const GvdLayer& layer,
                    const GlobalIndices& neighbor_indices,
                    const GlobalIndex& voxel_index,
                    GvdVoxel& voxel);

}

struct OpenQueueEntry {
  GlobalIndex index;
  GvdVoxel* voxel;
};

/**
 * An ESDF and GVD integrator based on https://arxiv.org/abs/1611.03631
 */
class GvdIntegrator {
 public:
  struct Config : VerbosityConfig {
    //! Maximum distance to integrate ESDF to
    float max_distance_m = 2.0f;
    //! Minimum distance to start integration at
    float min_distance_m = 0.2f;
    //! Minimum difference in TSDF value to trigger voxel update
    float min_diff_m = 1.0e-3f;
    //! Minimum observation weight
    float min_weight = 1.0e-6f;
    //! Only integrate ESDF for positive values
    bool positive_distance_only = true;
    //! Use TSDF values to detect surfaces instead of mesh vertices
    bool use_tsdf_for_surface = false;
    //! Amount to inflate SDF zero-crossing threshold
    float surface_threshold_inflation = 1.0;
    //! Use TSDF gradient to refine basis point positions
    bool refine_surface_voxel_pos = false;
    //! Criteria for GVD membership
    VoronoiCheckConfig voronoi_config;

    Config() : VerbosityConfig("[gvd] ") {}
  } const config;

  GvdIntegrator(const Config& config, const GvdLayer::Ptr& gvd_layer);

  virtual ~GvdIntegrator() = default;

  void updateFromTsdf(uint64_t timestamp_ns,
                      const TsdfLayer& tsdf,
                      bool clear_updated_flag,
                      const MeshLayer* mesh = nullptr,
                      bool use_all_blocks = false);

  void updateGvd(uint64_t timestamp_ns, GraphExtractor* graph_extractor = nullptr);

  void archiveBlocks(const BlockIndices& blocks,
                     GraphExtractor* graph_extractor = nullptr);

  const double default_distance;

 protected:
  // GVD membership
  void updateGvdVoxel(const GlobalIndex& voxel_index,
                      GvdVoxel& voxel,
                      GvdVoxel& other,
                      GraphExtractor* graph_extractor);

  void clearGvdVoxel(const GlobalIndex& index,
                     GvdVoxel& voxel,
                     GraphExtractor* graph_extractor);

  void updateVoronoiQueue(GvdVoxel& curr_voxel,
                          const GlobalIndex& curr_pos,
                          GvdVoxel& neighbor_voxel,
                          const GlobalIndex& neighbor_pos,
                          GraphExtractor* extractor);

  // TSDF propagation
  void propagateSurface(const TsdfLayer& tsdf, const BlockIndices& blocks);
  void propagateSurface(const MeshLayer& mesh);

  void processTsdfBlock(const TsdfBlock& block, const BlockIndex& index);

  void updateUnobservedVoxel(const TsdfVoxel& tsdf_voxel,
                             const GlobalIndex& index,
                             GvdVoxel& gvd_voxel);

  void updateObservedVoxel(const TsdfVoxel& tsdf_voxel,
                           const GlobalIndex& index,
                           GvdVoxel& gvd_voxel);

  // ESDF integration
  void pushToQueue(const GlobalIndex& index, GvdVoxel& voxel);

  void raiseVoxel(const GlobalIndex& index, GvdVoxel& voxel);

  void lowerVoxel(const GlobalIndex& index, GvdVoxel& voxel, GraphExtractor* extractor);

  void processOpenQueue(GraphExtractor* extractor);

  // Helpers
  bool isTsdfFixed(const TsdfVoxel& voxel);

  Point getParentPosition(const GlobalIndex& index, const GvdVoxel& voxel) const;

  std::optional<Point> computeGradient(const TsdfLayer& tsdf,
                                       const GlobalIndex& index) const;

 protected:
  UpdateStatistics update_stats_;

  GvdLayer::Ptr gvd_layer_;

  GvdParentTracker parent_tracker_;
  const spatial_hash::NeighborSearch neighbor_search_;

  BucketQueue<OpenQueueEntry> open_;

  float voxel_size_;
  float min_integration_distance_m_;
};

void declare_config(GvdIntegrator::Config& config);

}  // namespace hydra::places
