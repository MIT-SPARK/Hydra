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
#include <queue>

#include "hydra/common/dsg_types.h"
#include "hydra/places/graph_extractor_config.h"
#include "hydra/places/gvd_graph.h"
#include "hydra/places/gvd_voxel.h"
#include "hydra/reconstruction/voxel_types.h"

namespace hydra::places {

struct GvdParentTracker;

class GraphExtractorInterface {
 public:
  using Ptr = std::shared_ptr<GraphExtractorInterface>;
  using NodeIndexMap = std::unordered_map<NodeId, GlobalIndex>;

  explicit GraphExtractorInterface(const GraphExtractorConfig& config);

  virtual ~GraphExtractorInterface();

  virtual void pushGvdIndex(const GlobalIndex& index);

  virtual void clearGvdIndex(const GlobalIndex& index) = 0;

  virtual void removeDistantIndex(const GlobalIndex& index) = 0;

  virtual void extract(const GvdLayer& layer, uint64_t timestamp_ns) = 0;

  void fillParentInfo(const GvdLayer& gvd, const GvdParentTracker& parents);

  const SceneGraphLayer& getGraph() const;

  const GvdGraph& getGvdGraph() const;

  std::unordered_set<NodeId> getActiveNodes() const;

  const std::unordered_set<NodeId>& getDeletedNodes() const;

  const std::vector<NodeId>& getDeletedEdges() const;

  void clearDeleted();

  const NodeIndexMap& getIndexMap() const { return node_index_map_; }

 protected:
  NodeId addPlaceToGraph(const GvdLayer& layer,
                         const GvdVoxel& voxel,
                         const GlobalIndex& index);

  EdgeAttributes::Ptr makeEdgeInfo(const GvdLayer& layer,
                                   NodeId source_id,
                                   NodeId target_id) const;

  GlobalIndex popFromModifiedQueue();

  void removeGraphNode(NodeId node);

  void updateGraphEdge(NodeId source,
                       NodeId target,
                       EdgeAttributes::Ptr&& attrs,
                       bool is_heursitic = false);

  void addGraphNode(NodeId node, PlaceNodeAttributes::Ptr&& attrs = nullptr);

  void removeGraphEdge(NodeId source, NodeId target);

  void mergeGraphNodes(NodeId from, NodeId to);

  void updateHeuristicEdges(const GvdLayer& layer);

 protected:
  const GraphExtractorConfig config_;
  NodeSymbol next_node_id_;
  GvdGraph::Ptr gvd_;

  NodeIndexMap node_index_map_;
  std::queue<GlobalIndex> modified_voxel_queue_;

  std::map<EdgeKey, bool> heuristic_edges_;
  std::unordered_set<NodeId> deleted_nodes_;
  std::vector<NodeId> deleted_edges_;

 private:
  // just to make book-keeping easier
  IsolatedSceneGraphLayer::Ptr graph_;
};

}  // namespace hydra::places
