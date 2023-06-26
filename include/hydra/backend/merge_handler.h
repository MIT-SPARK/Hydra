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

namespace hydra {

struct NodeInfo {
  using Ptr = std::unique_ptr<NodeInfo>;

  LayerId layer;
  NodeAttributes::Ptr attrs;
  bool is_active;
  bool need_backend_update = false;
  std::vector<NodeId> neighbors;
};

class MergeHandler {
 public:
  using ObjectUpdater = dsg_updates::UpdateObjectsFunctor;
  using PlaceUpdater = dsg_updates::UpdatePlacesFunctor;
  using MeshVertices = DynamicSceneGraph::MeshVertices;

  MergeHandler(const std::shared_ptr<ObjectUpdater>& object_updater,
               const std::shared_ptr<PlaceUpdater> place_updater,
               bool undo_allowed);

  void updateFromUnmergedGraph(const DynamicSceneGraph& graph);

  size_t checkAndUndo(DynamicSceneGraph& graph, const UpdateInfo& info);

  void updateMerges(const std::map<NodeId, NodeId>& new_merges,
                    DynamicSceneGraph& graph);

  void reset();

  inline const std::map<NodeId, NodeId>& mergedNodes() const { return merged_nodes_; }

  inline const std::map<NodeId, std::set<NodeId>>& mergeParentNodes() const {
    return merged_nodes_parents_;
  }

 protected:
  void clearRemovedNodes(const DynamicSceneGraph& graph);

  void updateNodeEntry(const SceneGraphNode& node, NodeInfo& entry);

  void addNodeToCache(const SceneGraphNode& node,
                      std::map<NodeId, NodeInfo::Ptr>& cache,
                      bool check_if_present);

  void updateCacheEntryFromInfo(const MeshVertices::Ptr& mesh,
                                const UpdateInfo& info,
                                NodeId node,
                                NodeInfo& entry);

  void updateInfoCaches(const DynamicSceneGraph& graph, const UpdateInfo& info);

  bool shouldUndo(const NodeInfo& from_info, const NodeInfo& to_info) const;

  void undoMerge(DynamicSceneGraph& graph,
                 const UpdateInfo& info,
                 NodeId from_node,
                 NodeId to_node);

  std::map<NodeId, NodeInfo::Ptr> merged_nodes_cache_;
  std::map<NodeId, NodeInfo::Ptr> parent_nodes_cache_;
  std::map<NodeId, NodeId> merged_nodes_;
  std::map<NodeId, std::set<NodeId>> merged_nodes_parents_;

  std::shared_ptr<ObjectUpdater> object_updater_;
  std::shared_ptr<PlaceUpdater> place_updater_;
  bool undo_allowed_;
};

}  // namespace hydra
