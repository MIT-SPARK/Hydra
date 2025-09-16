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
#include <config_utilities/factory.h>
#include <spark_dsg/traversability_boundary.h>

#include "hydra/backend/deformation_interpolator.h"
#include "hydra/backend/update_functions.h"
#include "hydra/openset/embedding_distances.h"
#include "hydra/utils/active_window_tracker.h"
#include "hydra/utils/nearest_neighbor_utilities.h"

namespace hydra {

/**
 * @brief Functor to update traversability places in the DSG. This functor should be
 * called with exhaustive merging enabled.
 */
struct UpdateTraversabilityFunctor : public UpdateFunctor {
  struct Config {
    //! Layer to update traversability in
    std::string layer = DsgLayers::TRAVERSABILITY;

    //! Minimum side length of a traversability area to be considered [m].
    double min_place_size = 0.5;

    //! Maximum width of a place created in the active window [m].
    double max_place_size = 1.0;

    //! Extra tolerance for connectivity distance and overlap checks for merging in
    //! meters.
    double tolerance = 0.1;

    //! If false, compute the topoligcal distance to occupied space. If true, use metric
    //! distance.
    bool use_metric_distance = false;

    DeformationInterpolator::Config deformation;

    // TMP(lschmid): Cognition labels.
    bool use_cognition_labels = true;
  } const config;

  using EdgeSet = std::set<EdgeKey>;
  using NodeSet = std::set<NodeId>;
  using State = spark_dsg::TraversabilityState;

  explicit UpdateTraversabilityFunctor(const Config& config);

  Hooks hooks() const override;

  void call(const DynamicSceneGraph& unmerged,
            SharedDsgInfo& dsg,
            const UpdateInfo::ConstPtr& info) const override;

 protected:
  // Hook callbacks.
  MergeList findNodeMerges(const DynamicSceneGraph& dsg,
                           const UpdateInfo::ConstPtr& info) const;

  NodeAttributes::Ptr mergeNodes(const DynamicSceneGraph& dsg,
                                 const std::vector<NodeId>& merge_ids) const;

  void cleanup(const UpdateInfo::ConstPtr& /* info */, SharedDsgInfo* dsg) const;

  // Processing Steps.
  /**
   * @brief Update the positions of all traversability nodes in the DSG. Propagates to
   * the complete DSG in case of new loop closures.
   */
  void updateDeformation(const DynamicSceneGraph& unmerged,
                         SharedDsgInfo& dsg,
                         const UpdateInfo::ConstPtr& info) const;

  EdgeSet findActiveWindowEdges(DynamicSceneGraph& dsg) const;

  void pruneActiveWindowEdges(DynamicSceneGraph& dsg,
                              const EdgeSet& active_edges) const;

  void updateDistances(const SceneGraphLayer& layer) const;

  // Helper functions.

  /**
   * @brief Find connections of places that should be considered for merging or
   * connection in the active window.
   */
  std::vector<NodeId> findConnections(
      const DynamicSceneGraph& dsg,
      const TraversabilityNodeAttributes& from_attrs) const;

  /**
   * @brief Check whether the two traversability areas overlap sufficiently to be
   * considered traversable in-place.
   */
  bool hasTraversableOverlap(const TraversabilityNodeAttributes& from,
                             const TraversabilityNodeAttributes& to) const;

  /**
   * @brief Check whether the from boundary is contained in the in boundary.
   */
  bool isContained(const Boundary& from, const Boundary& in) const;

  /**
   * @brief Recursively compute the metrc distance to the nearest intraversable
   * obstacle for the query place.
   */
  double computeMetricDistance(const SceneGraphLayer& layer,
                               const Eigen::Vector2d& point,
                               const NodeSet& to_visit,
                               NodeSet& visited) const;

  double distanceToIntraversable(const TraversabilityNodeAttributes& attrs,
                                 const Eigen::Vector2d& point) const;

  void computeTopologicalDistances(const SceneGraphLayer& layer) const;

  void resetNeighborFinder(const DynamicSceneGraph& dsg) const;

  void computeCognitionDistances(const DynamicSceneGraph& dsg) const;

 protected:
  // Cached constants.
  const double radius_;
  const double min_connectivity_;

  // State.
  mutable EdgeSet previous_active_edges_;
  mutable std::set<EdgeKey> overlapping_nodes_to_cleanup_;

  // Members.
  mutable ActiveWindowTracker active_tracker_;
  const DeformationInterpolator deformation_interpolator_;
  mutable NearestNodeFinder::Ptr nn_;

  // TMP(lschmid): Track cognition labels.
  mutable std::unordered_map<NodeId, int> previous_cognition_labels_;
};

void declare_config(UpdateTraversabilityFunctor::Config& config);

}  // namespace hydra
