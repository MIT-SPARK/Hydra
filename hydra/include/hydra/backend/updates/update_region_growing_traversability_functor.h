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

#include <spark_dsg/node_attributes.h>

#include "hydra/backend/deformation_interpolator.h"
#include "hydra/backend/update_functions.h"
#include "hydra/utils/logging.h"

namespace hydra {

/**
 * @brief Functor to update traversability places in the DSG. This functor should be
 * called with exhaustive merging enabled.
 */
struct UpdateRegionGrowingTraversabilityFunctor : public UpdateFunctor {
  struct Config : public VerbosityConfig {
    //! Layer to update traversability in
    std::string layer = spark_dsg::DsgLayers::TRAVERSABILITY;

    DeformationInterpolator::Config deformation;
  } const config;

  using EdgeSet = std::set<spark_dsg::EdgeKey>;
  using NodeSet = std::set<spark_dsg::NodeId>;
  using State = spark_dsg::TraversabilityState;

  explicit UpdateRegionGrowingTraversabilityFunctor(const Config& config);

  Hooks hooks() const override;

  void call(const spark_dsg::SceneGraph& unmerged,
            SharedDsgInfo& dsg,
            const UpdateInfo::ConstPtr& info) const override;

 protected:
  // Hook callbacks.
  MergeList findNodeMerges(const spark_dsg::SceneGraph& dsg,
                           const UpdateInfo::ConstPtr& info) const;

  spark_dsg::NodeAttributes::Ptr mergeNodes(
      const spark_dsg::SceneGraph& dsg,
      const std::vector<spark_dsg::NodeId>& merge_ids) const;

  void cleanup(const UpdateInfo::ConstPtr& /* info */, SharedDsgInfo* /* dsg */) const;

  // Processing Steps.
  /**
   * @brief Update the positions of all traversability nodes in the DSG. Propagates to
   * the complete DSG in case of new loop closures.
   */
  void updateDeformation(const spark_dsg::SceneGraph& unmerged,
                         SharedDsgInfo& dsg,
                         const UpdateInfo::ConstPtr& info) const;

  /**
   * @brief Remove all active window and inactive edges from the graph.
   */
  void resetAddedEdges(spark_dsg::SceneGraph& dsg) const;

  /**
   * @brief Compute edges between overlapping inactive nodes globally.
   */
  void findInactiveEdges(spark_dsg::SceneGraph& dsg) const;

  /**
   * @brief Find and add edges from active to inactive nodes.
   */
  void findActiveWindowEdges(spark_dsg::SceneGraph& dsg) const;

  /**
   * @brief Remove active window edges that no longer have active overlap and move
   * designate archived ones as inactive edges.
   */
  void pruneActiveWindowEdges(spark_dsg::SceneGraph& dsg) const;

  // Helper functions.

  /**
   * @brief Find places that are inactive and spatially but not temporally overlap with
   * the given node.
   */
  std::vector<spark_dsg::NodeId> findConnections(
      const spark_dsg::SceneGraph& dsg,
      const spark_dsg::TravNodeAttributes& from_attrs) const;

  /**
   * @brief Check if two traversability nodes have active window (temporal) overlap.
   */
  static bool hasActiveOverlap(const spark_dsg::TravNodeAttributes& attrs1,
                               const spark_dsg::TravNodeAttributes& attrs2);

  /**
   * @brief View on all active nodes in a layer.
   */
  static spark_dsg::LayerView activeNodes(const spark_dsg::SceneGraphLayer& layer);

 protected:
  // Members.
  const DeformationInterpolator deformation_interpolator_;

  // State.
  mutable EdgeSet active_edges_;      // Active window edges in the current update.
  mutable EdgeSet merge_candidates_;  // List of nodes that have inactive edges and
                                      // could thus be merged this iteration.
};

void declare_config(UpdateRegionGrowingTraversabilityFunctor::Config& config);

}  // namespace hydra
