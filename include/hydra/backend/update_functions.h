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
#include <gtsam/nonlinear/Values.h>

#include "hydra/common/common.h"
#include "hydra/common/shared_dsg_info.h"
#include "hydra/utils/nearest_neighbor_utilities.h"

namespace hydra {

class RoomFinder;
struct RoomFinderConfig;

struct UpdateInfo {
  const gtsam::Values* places_values = nullptr;
  const gtsam::Values* pgmo_values = nullptr;
  bool loop_closure_detected = false;
  uint64_t timestamp_ns = 0;
  bool allow_node_merging = false;
  const gtsam::Values* complete_agent_values = nullptr;
};

using MergeMap = std::map<NodeId, NodeId>;
using MeshVertices = DynamicSceneGraph::MeshVertices;
using LayerUpdateFunc = std::function<MergeMap(SharedDsgInfo&, const UpdateInfo&)>;
using LayerCleanupFunc = std::function<void(const UpdateInfo&, SharedDsgInfo*)>;
using NodeMergeFunc = std::function<void(const DynamicSceneGraph&, NodeId, NodeId)>;
using MergeValidFunc =
    std::function<bool(const NodeAttributes*, const NodeAttributes*)>;
using NodeUpdateFunc = std::function<void(
    const UpdateInfo&, const MeshVertices::Ptr, NodeId, NodeAttributes*)>;

namespace dsg_updates {

struct UpdateFunctor {
  using Ptr = std::shared_ptr<UpdateFunctor>;

  struct Hooks {
    LayerUpdateFunc update;
    LayerCleanupFunc cleanup;
    MergeValidFunc should_merge;
    NodeMergeFunc merge;
    NodeUpdateFunc node_update;
  };

  virtual MergeMap call(SharedDsgInfo& dsg, const UpdateInfo& info) const = 0;

  virtual Hooks hooks() const {
    Hooks my_hooks;
    my_hooks.update = [this](SharedDsgInfo& dsg, const UpdateInfo& info) {
      return call(dsg, info);
    };
    return my_hooks;
  }
};

struct UpdateObjectsFunctor : public UpdateFunctor {
  UpdateObjectsFunctor();

  explicit UpdateObjectsFunctor(float angle_step);

  Hooks hooks() const override;

  MergeMap call(SharedDsgInfo& dsg, const UpdateInfo& info) const override;

  size_t makeNodeFinders(const SceneGraphLayer& layer) const;

  void updateObject(const MeshVertices::Ptr& mesh,
                    NodeId node,
                    ObjectNodeAttributes& attrs) const;

  std::optional<NodeId> proposeObjectMerge(const SceneGraphLayer& layer,
                                           const ObjectNodeAttributes& attrs,
                                           bool skip_first) const;

  void mergeAttributes(const DynamicSceneGraph& layer, NodeId from, NodeId to) const;

  bool shouldMerge(const ObjectNodeAttributes& from_attrs,
                   const ObjectNodeAttributes& to_attrs) const;

  float angle_step;
  size_t num_merges_to_consider = 1;
  bool use_active_flag = true;
  bool allow_connection_merging = false;
  std::shared_ptr<std::set<size_t>> invalid_indices;
  mutable std::map<SemanticLabel, std::unique_ptr<NearestNodeFinder>> node_finders;
};

struct UpdatePlacesFunctor : public UpdateFunctor {
  UpdatePlacesFunctor(double pos_threshold, double distance_tolerance)
      : pos_threshold_m(pos_threshold), distance_tolerance_m(distance_tolerance) {}

  Hooks hooks() const override;

  MergeMap call(SharedDsgInfo& dsg, const UpdateInfo& info) const override;

  size_t makeNodeFinder(const SceneGraphLayer& layer) const;

  void updatePlace(const gtsam::Values& values,
                   NodeId node,
                   NodeAttributes& attrs) const;

  std::optional<NodeId> proposePlaceMerge(const SceneGraphLayer& layer,
                                          NodeId node_id,
                                          const PlaceNodeAttributes& attrs,
                                          bool skip_first) const;

  bool shouldMerge(const PlaceNodeAttributes& from_attrs,
                   const PlaceNodeAttributes& to_attrs) const;

  void filterMissing(DynamicSceneGraph& graph,
                     const std::list<NodeId> missing_nodes) const;

  size_t num_merges_to_consider = 1;
  bool use_active_flag = true;
  double pos_threshold_m;
  double distance_tolerance_m;
  mutable std::unique_ptr<NearestNodeFinder> node_finder;
};

struct UpdateRoomsFunctor : public UpdateFunctor {
  UpdateRoomsFunctor(const RoomFinderConfig& config);

  ~UpdateRoomsFunctor();

  MergeMap call(SharedDsgInfo& dsg, const UpdateInfo& info) const override;

  void rewriteRooms(const SceneGraphLayer* new_rooms, DynamicSceneGraph& graph) const;

  std::unique_ptr<RoomFinder> room_finder;
};

struct UpdateBuildingsFunctor : public UpdateFunctor {
  UpdateBuildingsFunctor(const SemanticNodeAttributes::ColorVector& color,
                         SemanticNodeAttributes::Label label);

  MergeMap call(SharedDsgInfo& dsg, const UpdateInfo& info) const override;

  SemanticNodeAttributes::ColorVector building_color;
  SemanticNodeAttributes::Label building_semantic_label;
};

MergeMap updateAgents(SharedDsgInfo& graph, const UpdateInfo& info);

}  // namespace dsg_updates
}  // namespace hydra
