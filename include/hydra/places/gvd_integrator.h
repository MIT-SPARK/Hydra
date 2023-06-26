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
#include <utility>

#include "hydra/places/graph_extractor_interface.h"
#include "hydra/places/gvd_integrator_config.h"
#include "hydra/places/gvd_parent_tracker.h"
#include "hydra/places/gvd_utilities.h"
#include "hydra/places/gvd_voxel.h"
#include "hydra/places/update_statistics.h"
#include "hydra/places/voxblox_types.h"
#include "hydra/places/vertex_voxel.h"

namespace hydra {
namespace places {

struct OpenQueueEntry {
  GlobalIndex index;
  GvdVoxel* voxel;
};

/**
 * An ESDF and GVD integrator based on https://arxiv.org/abs/1611.03631
 */
class GvdIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GvdIntegrator(const GvdIntegratorConfig& config,
                const Layer<GvdVoxel>::Ptr& gvd_layer);

  virtual ~GvdIntegrator() = default;

  const SceneGraphLayer& getGraph() const;

  const GvdGraph& getGvdGraph() const;

  GraphExtractorInterface& getGraphExtractor() const;

  void updateFromTsdf(uint64_t timestamp_ns,
                      Layer<TsdfVoxel>& tsdf,
                      const Layer<VertexVoxel>& vertices,
                      const MeshLayer& mesh,
                      bool clear_updated_flag,
                      bool use_all_blocks = false);

  void updateGvd(uint64_t timestamp_ns);

  void archiveBlocks(const BlockIndexList& blocks);

  // TODO(nathan) test this
  static bool setFixedParent(const Layer<GvdVoxel>& layer,
                             const GvdNeighborhood::IndexMatrix& neighbor_indices,
                             GvdVoxel& voxel);

 protected:
  // GVD membership
  void updateGvdVoxel(const GlobalIndex& voxel_index, GvdVoxel& voxel, GvdVoxel& other);

  void clearGvdVoxel(const GlobalIndex& index, GvdVoxel& voxel);

  void updateVoronoiQueue(GvdVoxel& curr_voxel,
                          const GlobalIndex& curr_pos,
                          GvdVoxel& neighbor_voxel,
                          const GlobalIndex& neighbor_pos);

  // TSDF propagation
  void propagateSurface(const BlockIndex& block_index,
                        const MeshLayer& mesh,
                        const Layer<VertexVoxel>& vertices);

  void processTsdfBlock(const Block<TsdfVoxel>& block, const BlockIndex& index);

  void updateUnobservedVoxel(const TsdfVoxel& tsdf_voxel,
                             const GlobalIndex& index,
                             GvdVoxel& gvd_voxel);

  void updateObservedVoxel(const TsdfVoxel& tsdf_voxel,
                           const GlobalIndex& index,
                           GvdVoxel& gvd_voxel);

  // ESDF integration
  void pushToQueue(const GlobalIndex& index, GvdVoxel& voxel);

  void raiseVoxel(const GlobalIndex& index, GvdVoxel& voxel);

  void lowerVoxel(const GlobalIndex& index, GvdVoxel& voxel);

  void processOpenQueue();

  // Helpers
  bool isTsdfFixed(const TsdfVoxel& voxel);

  voxblox::Point getParentPosition(const GlobalIndex& index,
                                   const GvdVoxel& voxel) const;

  voxblox::Point getPosition(const GlobalIndex& index) const;

 protected:
  UpdateStatistics update_stats_;

  const double default_distance_;
  GvdIntegratorConfig config_;
  Layer<GvdVoxel>::Ptr gvd_layer_;

  GraphExtractorInterface::Ptr graph_extractor_;
  GvdParentTracker parent_tracker_;
  GvdNeighborhood::IndexMatrix neighbor_indices_;

  BucketQueue<OpenQueueEntry> open_;

  FloatingPoint voxel_size_;
  FloatingPoint min_integration_distance_m_;
};

}  // namespace places
}  // namespace hydra
