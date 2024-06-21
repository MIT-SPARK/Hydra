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
#include "hydra/backend/update_functions.h"
#include "hydra/utils/active_window_tracker.h"
#include "hydra/utils/nearest_neighbor_utilities.h"

namespace hydra {

struct Update2dPlacesFunctor : public UpdateFunctor {
  struct Config {
    bool allow_places_merge = true;
    //! If two places differ by at least this much in z, they won't be merged
    double merge_max_delta_z = 0.5;
    //! Minimum number of points to allow splitting place
    size_t min_points = 10;
    //! Minimum size of place for splitting
    double min_size = 2;
    //! Amount of overlap between places necessary to add edge
    double connection_overlap_threshold = 0;
    //! Maximum difference in z between neighboring places
    double connection_max_delta_z = 0.5;
    //! How much to inflate place ellipsoid relative to bounding box
    double connection_ellipse_scale_factor = 1.0;
  };

  Update2dPlacesFunctor(const Config& config);
  Hooks hooks() const override;
  MergeList call(const DynamicSceneGraph& unmerged,
                 SharedDsgInfo& dsg,
                 const UpdateInfo::ConstPtr& info) const override;

  void updateNode(const spark_dsg::Mesh::Ptr& mesh,
                  NodeId node,
                  Place2dNodeAttributes& attrs) const;

  std::optional<NodeId> proposeMerge(const SceneGraphLayer& layer,
                                     const SceneGraphNode& node) const;

  bool shouldMerge(const Place2dNodeAttributes& from_attrs,
                   const Place2dNodeAttributes& to_attrs) const;

  void cleanup(SharedDsgInfo& dsg) const;

  size_t num_merges_to_consider = 1;
  mutable SemanticNodeFinders node_finders;
  mutable ActiveWindowTracker active_tracker;

 private:
  Config config_;
  mutable NodeSymbol next_node_id_ = NodeSymbol('S', 0);
  const LayerId layer_id_ = DsgLayers::MESH_PLACES;
};

void declare_config(Update2dPlacesFunctor::Config& conf);

}  // namespace hydra
