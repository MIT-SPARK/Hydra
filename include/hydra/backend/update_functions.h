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
  using Ptr = std::shared_ptr<UpdateInfo>;
  using ConstPtr = std::shared_ptr<const UpdateInfo>;
  const gtsam::Values* places_values = nullptr;
  const gtsam::Values* pgmo_values = nullptr;
  bool loop_closure_detected = false;
  uint64_t timestamp_ns = 0;
  bool allow_node_merging = false;
  const gtsam::Values* complete_agent_values = nullptr;
};

struct Places2dConfig {
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

void declare_config(Places2dConfig& conf);

using MergeMap = std::map<NodeId, NodeId>;
using LayerUpdateFunc =
    std::function<MergeMap(SharedDsgInfo&, const UpdateInfo::ConstPtr&)>;
using LayerCleanupFunc =
    std::function<void(const UpdateInfo::ConstPtr&, SharedDsgInfo*)>;
using NodeMergeFunc = std::function<void(const DynamicSceneGraph&, NodeId, NodeId)>;
using MergeValidFunc =
    std::function<bool(const NodeAttributes*, const NodeAttributes*)>;
using NodeUpdateFunc = std::function<void(
    const UpdateInfo::ConstPtr&, const spark_dsg::Mesh::Ptr, NodeId, NodeAttributes*)>;

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

  virtual ~UpdateFunctor() = default;

  virtual MergeMap call(SharedDsgInfo& dsg, const UpdateInfo::ConstPtr& info) const = 0;

  virtual Hooks hooks() const {
    Hooks my_hooks;
    my_hooks.update = [this](SharedDsgInfo& dsg, const UpdateInfo::ConstPtr& info) {
      return call(dsg, info);
    };
    return my_hooks;
  }
};

struct UpdateObjectsFunctor : public UpdateFunctor {
  UpdateObjectsFunctor();

  Hooks hooks() const override;

  MergeMap call(SharedDsgInfo& dsg, const UpdateInfo::ConstPtr& info) const override;

  size_t makeNodeFinders(const SceneGraphLayer& layer) const;

  void updateObject(const spark_dsg::Mesh::Ptr& mesh,
                    NodeId node,
                    ObjectNodeAttributes& attrs) const;

  std::optional<NodeId> proposeObjectMerge(const SceneGraphLayer& layer,
                                           const ObjectNodeAttributes& attrs,
                                           bool skip_first) const;

  void mergeAttributes(const DynamicSceneGraph& layer, NodeId from, NodeId to) const;

  bool shouldMerge(const ObjectNodeAttributes& from_attrs,
                   const ObjectNodeAttributes& to_attrs) const;

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

  MergeMap call(SharedDsgInfo& dsg, const UpdateInfo::ConstPtr& info) const override;

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

struct UpdateFrontiersFunctor : public UpdateFunctor {
  UpdateFrontiersFunctor() {}

  Hooks hooks() const override;

  MergeMap call(SharedDsgInfo& dsg, const UpdateInfo::ConstPtr& info) const override;

  size_t makeNodeFinder(const SceneGraphLayer& layer) const;

  void updateFrontier(const gtsam::Values& values,
                      NodeId node,
                      NodeAttributes& attrs) const;

  std::optional<NodeId> proposeFrontierMerge(const SceneGraphLayer& layer,
                                             NodeId node_id,
                                             const FrontierNodeAttributes& attrs,
                                             bool skip_first) const;

  bool shouldMerge(const FrontierNodeAttributes& from_attrs,
                   const FrontierNodeAttributes& to_attrs) const;

  void filterMissing(DynamicSceneGraph& graph,
                     const std::list<NodeId> missing_nodes) const;

  size_t num_merges_to_consider = 1;
  bool use_active_flag = true;  // currently this disables any frontier merges I believe
  double pos_threshold_m;
  double distance_tolerance_m;
  mutable std::unique_ptr<NearestNodeFinder> node_finder;
};

struct UpdateRoomsFunctor : public UpdateFunctor {
  UpdateRoomsFunctor(const RoomFinderConfig& config);

  ~UpdateRoomsFunctor();

  MergeMap call(SharedDsgInfo& dsg, const UpdateInfo::ConstPtr& info) const override;

  void rewriteRooms(const SceneGraphLayer* new_rooms, DynamicSceneGraph& graph) const;

  std::unique_ptr<RoomFinder> room_finder;
};

struct UpdateBuildingsFunctor : public UpdateFunctor {
  UpdateBuildingsFunctor(const SemanticNodeAttributes::ColorVector& color,
                         SemanticNodeAttributes::Label label);

  MergeMap call(SharedDsgInfo& dsg, const UpdateInfo::ConstPtr& info) const override;

  SemanticNodeAttributes::ColorVector building_color;
  SemanticNodeAttributes::Label building_semantic_label;
};

MergeMap updateAgents(SharedDsgInfo& graph, const UpdateInfo::ConstPtr& info);

struct Update2dPlacesFunctor : public UpdateFunctor {
  Hooks hooks() const override;

  Update2dPlacesFunctor(const Places2dConfig& config);

  ~Update2dPlacesFunctor();

  MergeMap call(SharedDsgInfo& dsg, const UpdateInfo::ConstPtr& info) const override;

  size_t makeNodeFinders(const SceneGraphLayer& layer) const;

  void mergeAttributes(const DynamicSceneGraph& layer, NodeId from, NodeId to) const;

  void updateNode(const spark_dsg::Mesh::Ptr& mesh,
                  NodeId node,
                  Place2dNodeAttributes& attrs) const;

  std::optional<NodeId> proposeMerge(const SceneGraphLayer& layer,
                                     const NodeId from_node,
                                     const Place2dNodeAttributes& attrs,
                                     bool skip_first) const;

  bool shouldMerge(const Place2dNodeAttributes& from_attrs,
                   const Place2dNodeAttributes& to_attrs) const;

  size_t num_merges_to_consider = 1;
  bool use_active_flag = true;
  mutable std::map<SemanticLabel, std::unique_ptr<NearestNodeFinder>> node_finders;

 private:
  Places2dConfig config_;
  mutable NodeSymbol next_node_id_ = NodeSymbol('S', 0);
  const LayerId layer_id_ = DsgLayers::MESH_PLACES;
};

template <typename Derived, typename Func>
bool dispatchMergeCheck(const NodeAttributes* lhs,
                        const NodeAttributes* rhs,
                        const Func& func) {
  const auto d_lhs = dynamic_cast<const Derived*>(lhs);
  const auto d_rhs = dynamic_cast<const Derived*>(rhs);
  if (!d_lhs || !d_rhs) {
    return false;
  }

  return func(*d_lhs, *d_rhs);
}

}  // namespace dsg_updates
}  // namespace hydra
