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

#include <queue>

#include "hydra/common/dsg_types.h"
#include "hydra/places/graph_extractor_interface.h"
#include "hydra/places/gvd_voxel.h"
#include "hydra/places/voxel_templates.h"
#include "hydra/reconstruction/voxel_types.h"

namespace hydra::places {

struct VoxelGraphInfo {
  // TODO(nathan) consider copy constructor-eqsue cleanup of extract edges
  VoxelGraphInfo();
  VoxelGraphInfo(NodeId id, bool is_from_split);

  NodeId id;
  bool is_node;
  bool is_split_node;
  size_t edge_id;
};

struct EdgeInfo {
  EdgeInfo() = default;

  EdgeInfo(size_t id, NodeId source);

  size_t id;
  NodeId source;
  GlobalIndexSet indices;
  std::set<NodeId> node_connections;
  std::set<size_t> connections;
};

struct EdgeSplitSeed {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSplitSeed(const GlobalIndex& index, double distance_to_edge, size_t edge_id);

  GlobalIndex index;
  double distance_to_edge;
  size_t edge_id;
};

bool operator<(const EdgeSplitSeed& lhs, const EdgeSplitSeed& rhs);

struct PseudoEdgeInfo {
  std::vector<NodeId> nodes;
  GlobalIndices indices;
};

std::ostream& operator<<(std::ostream& out, const VoxelGraphInfo& info);

std::ostream& operator<<(std::ostream& out, const EdgeInfo& info);

class FloodfillGraphExtractor : public GraphExtractorInterface {
 public:
  using Ptr = std::unique_ptr<FloodfillGraphExtractor>;

  using NodeChildMap = std::unordered_map<NodeId, GlobalIndexSet>;
  using IndexGraphInfoMap = GlobalIndexMap<VoxelGraphInfo>;
  using EdgeInfoMap = std::unordered_map<size_t, EdgeInfo>;
  using EdgeSplitQueue = std::priority_queue<EdgeSplitSeed, std::vector<EdgeSplitSeed>>;

  explicit FloodfillGraphExtractor(const FloodfillExtractorConfig& config);

  virtual ~FloodfillGraphExtractor();

  void clearGvdIndex(const GlobalIndex& index) override;

  void removeDistantIndex(const GlobalIndex& index) override;

  void extract(const GvdLayer& layer, uint64_t timestamp_ns) override;

 protected:
  void addNewPlaceNode(const GvdLayer& layer,
                       const GvdVoxel& voxel,
                       const GlobalIndex& index,
                       bool is_from_split = false);

  void clearNodeInfo(NodeId node_id);

  void clearEdgeInfo(size_t edge_id, bool clear_indices = true);

  void removeNodeIndex(NodeId node_id);

  void removeEdgeIndices(size_t edge_id, bool clear_indices);

  void addNeighborToFrontier(const VoxelGraphInfo& info,
                             const GlobalIndex& neighbor_index);

  bool updateEdgeMaps(const VoxelGraphInfo& info, const VoxelGraphInfo& neighbor_info);

  void addEdgeToGraph(const GvdLayer& layer,
                      const VoxelGraphInfo& curr_info,
                      const VoxelGraphInfo& neighbor_info);

  void findBadEdgeIndices(const EdgeInfo& info);

  void findNewVertices(const GvdLayer& layer);

  bool attemptNodeMerge(const GvdLayer& layer,
                        const VoxelGraphInfo& curr_info,
                        const VoxelGraphInfo& neighbor_info);

  void extractEdges(const GvdLayer& layer, bool allow_merging = false);

  void filterRemovedConnections();

  void splitEdges(const GvdLayer& layer);

  GlobalIndex popFromFloodfillFrontier();

  void clearNewConnections(bool clear_modified_voxels);

  bool isVertex(const GvdLayer& layer, const GvdVoxel& voxel, const GlobalIndex& index);

 protected:
  FloodfillExtractorConfig config_;
  CornerFinder corner_finder_;

  std::queue<GlobalIndex> floodfill_frontier_;

  IndexGraphInfoMap index_graph_info_map_;
  NodeChildMap node_child_map_;

  size_t next_edge_id_;
  EdgeInfoMap edge_info_map_;
  std::map<NodeId, std::set<size_t>> node_edge_id_map_;
  std::map<NodeId, std::set<size_t>> node_edge_connections_;

  EdgeSplitQueue edge_split_queue_;

  // TODO(nathan) rename these
  std::map<size_t, std::set<size_t>> checked_edges_;
  std::set<size_t> connected_edges_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<GraphExtractorInterface,
                                     FloodfillGraphExtractor,
                                     FloodfillExtractorConfig>(
          "FloodfillGraphExtractor");
};

}  // namespace hydra::places
