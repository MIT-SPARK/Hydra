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
#include <memory>

#include "hydra/common/dsg_types.h"
#include "hydra/frontend/mesh_segmenter_config.h"

namespace kimera_pgmo {
class MeshDelta;
}  // namespace kimera_pgmo

namespace hydra {

struct Cluster {
  Eigen::Vector3d centroid;
  std::vector<size_t> indices;
};

using LabelIndices = std::map<uint32_t, std::vector<size_t>>;

class MeshSegmenter {
 public:
  using Clusters = std::vector<Cluster>;
  using LabelClusters = std::map<uint32_t, Clusters>;
  using CallbackFunc = std::function<void(const kimera_pgmo::MeshDelta& cloud,
                                          const std::vector<size_t>& indices,
                                          const LabelIndices& label_indices)>;

  explicit MeshSegmenter(const MeshSegmenterConfig& config);

  LabelClusters detect(uint64_t timestamp_ns,
                       const kimera_pgmo::MeshDelta& active,
                       const std::optional<Eigen::Vector3d>& pos);

  void updateGraph(uint64_t timestamp,
                   const LabelClusters& clusters,
                   size_t num_archived_vertices,
                   DynamicSceneGraph& graph);

  std::unordered_set<NodeId> getActiveNodes() const;

  inline void addVisualizationCallback(const CallbackFunc& func) {
    callback_funcs_.push_back(func);
  }

 private:
  void archiveOldNodes(const DynamicSceneGraph& graph, size_t num_archived_vertices);

  void addNodeToGraph(DynamicSceneGraph& graph,
                      const Cluster& cluster,
                      uint32_t label,
                      uint64_t timestamp);

  void updateNodeInGraph(DynamicSceneGraph& graph,
                         const Cluster& cluster,
                         const SceneGraphNode& node,
                         uint64_t timestamp);

  void mergeActiveNodes(DynamicSceneGraph& graph, uint32_t label);

 private:
  MeshSegmenterConfig config_;
  NodeSymbol next_node_id_;

  std::map<uint32_t, std::set<NodeId>> active_nodes_;
  std::vector<CallbackFunc> callback_funcs_;
};

}  // namespace hydra
