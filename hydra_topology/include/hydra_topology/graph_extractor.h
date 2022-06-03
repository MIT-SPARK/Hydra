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
#include "hydra_topology/graph_extraction_utilities.h"
#include "hydra_topology/graph_extractor_types.h"
#include "hydra_topology/gvd_voxel.h"
#include "hydra_topology/voxblox_types.h"

#include <queue>

namespace hydra {
namespace topology {

/**
 * @brief Configuration structure for graph extraction
 * @note We use vertex to refer to a voxel in the GVD that either acts as a connection
 * point between voxels that are topologically relevant (typically those with 3 or more
 * basis points) or encode curvature in topologically relevant voxels. We use node to
 * refer to a member of the places graph. This distinction becomes especially meaningful
 * when talking about constructing connections between disconnected components: these
 * connections can be comprised of nodes that are not backed by any specific vertex in
 * the GVD that still encode meaningful information about the curvature of the GVD.
 */
struct GraphExtractorConfig {
  //! Number of basis points for a voxel to be consider for extraction
  uint8_t min_extra_basis = 2;
  //! Number of basis points for a voxel to be automatically labeled a vertex
  uint8_t min_vertex_basis = 3;
  //! Whether or not to merge nodes close together during initial edge extraction
  bool merge_new_nodes = true;
  //! Maximum distance between two nodes to consider a merge
  double node_merge_distance_m = 0.2;
  /** @brief Whether or not to merge nodes close together when splitting edges
   *  @warning There is no guarantee that edge splitting will terminate in a fixed
   *           number of iterations with this option active.
   */
  bool edge_splitting_merge_nodes = true;
  //! Number of maximum iterations to run edges splitting (set to 0 to disable)
  size_t max_edge_split_iterations = 5;
  //! Maximum squared voxel distance an edge can be from supporting voxels at any point
  int64_t max_edge_deviation = 4;
  //! Add edges between nodes that have overlapping free-space regions
  bool add_freespace_edges = true;
  //! Maximum number of hops used to compute the candidate nodes for free-space edges
  size_t freespace_active_neighborhood_hops = 2;
  //! Number of nearest neighbors to check for each free-space edge candidate node
  size_t freespace_edge_num_neighbors = 3;
  //! Minimum clearance at edge intersection
  double freespace_edge_min_clearance_m = 0.2;
  //! Add edges between disconnected components to attempt to improve graph connectivity
  bool add_component_connection_edges = true;
  //! Number of iterations of visited nodes to keep
  size_t connected_component_window = 5;
  //! Maximum number of hops used to compute the candidate connected components
  size_t connected_component_hops = 2;
  //! Number of candidate nodes in each connected component to consider adding edges to
  size_t component_nodes_to_check = 5;
  //! Number of nearest neighbors to check for each connected component candidate node
  size_t component_nearest_neighbors = 3;
  //! Maximum permissible length for an edge between disconnected component nodes
  double component_max_edge_length_m = 5.0;
  //! Minimum distance from obstacle for a straight-line edge
  double component_min_clearance_m = 0.2;
  //! Remove nodes with no edges
  bool remove_isolated_nodes = true;
};

class GraphExtractor {
 public:
  using Ptr = std::unique_ptr<GraphExtractor>;
  using GvdLayer = Layer<GvdVoxel>;

  // TODO(nathan) may need aligned allocator
  using NodeIdRootMap = std::unordered_map<NodeId, GlobalIndex>;
  using NodeIdIndexMap = std::unordered_map<NodeId, voxblox::LongIndexSet>;
  using IndexGraphInfoMap = voxblox::LongIndexHashMapType<VoxelGraphInfo>::type;
  using EdgeInfoMap = std::unordered_map<size_t, EdgeInfo>;
  using EdgeSplitQueue =
      std::priority_queue<EdgeSplitSeed, voxblox::AlignedVector<EdgeSplitSeed>>;
  using PseudoEdgeInfoMap = std::map<size_t, PseudoEdgeInfo>;
  using PseudoEdgeMap = voxblox::LongIndexHashMapType<std::unordered_set<size_t>>::type;
  using Components = std::vector<std::vector<NodeId>>;

  explicit GraphExtractor(const GraphExtractorConfig& config);

  inline void pushGvdIndex(const GlobalIndex& index) {
    modified_voxel_queue_.push(index);
  }

  void clearGvdIndex(const GlobalIndex& index);

  void removeDistantIndex(const GlobalIndex& index);

  void extract(const GvdLayer& layer);

  void assignMeshVertices(const GvdLayer& gvd,
                          const GvdParentMap& parents,
                          const GvdVertexMap& parent_vertices);

  std::unordered_set<NodeId> getActiveNodes() const;

  std::unordered_set<NodeId> getDeletedNodes() const;

  void clearDeletedNodes();

  inline const SceneGraphLayer& getGraph() const { return *graph_; }

  inline const EdgeInfoMap& getGvdEdgeInfo() const { return edge_info_map_; }

  inline const NodeIdRootMap& getNodeRootMap() const { return node_id_root_map_; }

 protected:
  void clearNodeInfo(NodeId node_id);

  void clearEdgeInfo(size_t edge_id, bool clear_indices = true);

  void removeNodeIndex(NodeId node_id);

  void removeEdgeIndices(size_t edge_id, bool clear_indices);

  void clearPseudoEdgeInfo(const GlobalIndex& index);

  void addNeighborToFrontier(const VoxelGraphInfo& info,
                             const GlobalIndex& neighbor_index);

  void addPlaceToGraph(const GvdLayer& layer,
                       const GvdVoxel& voxel,
                       const GlobalIndex& index,
                       bool is_from_split = false);

  bool updateEdgeMaps(const VoxelGraphInfo& info, const VoxelGraphInfo& neighbor_info);

  EdgeAttributes::Ptr makeEdgeInfo(const GvdLayer& layer,
                                   NodeId source_id,
                                   NodeId target_id) const;

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

  void findFreespaceEdges();

  Components filterComponents(const Components& to_filter) const;

  bool addPseudoEdge(const GvdLayer& layer, NodeId source, NodeId target);

  void findComponentConnections(const GvdLayer& layer);

  void filterIsolatedNodes();

 protected:
  GraphExtractorConfig config_;

  CornerFinder corner_finder_;

  AlignedQueue<GlobalIndex> modified_voxel_queue_;
  AlignedQueue<GlobalIndex> floodfill_frontier_;

  NodeSymbol next_node_id_;
  IndexGraphInfoMap index_graph_info_map_;
  NodeIdIndexMap node_id_index_map_;
  NodeIdRootMap node_id_root_map_;

  size_t next_edge_id_;
  EdgeInfoMap edge_info_map_;
  std::map<NodeId, std::set<size_t>> node_edge_id_map_;
  std::map<NodeId, std::set<size_t>> node_edge_connections_;

  EdgeSplitQueue edge_split_queue_;

  // TODO(nathan) rename these
  std::map<size_t, std::set<size_t>> checked_edges_;
  std::set<size_t> connected_edges_;
  std::unordered_set<NodeId> visited_nodes_;
  std::unordered_set<NodeId> deleted_nodes_;

  size_t next_pseudo_edge_id_;
  PseudoEdgeInfoMap pseudo_edge_info_;
  PseudoEdgeMap pseudo_edge_map_;
  std::list<std::unordered_set<NodeId>> pseudo_edge_window_;
  std::list<std::pair<NodeId, NodeId>> removed_pseudo_edges_;

  IsolatedSceneGraphLayer::Ptr graph_;

 protected:
  inline GlobalIndex popFromModifiedGvd() {
    GlobalIndex index = modified_voxel_queue_.front();
    modified_voxel_queue_.pop();
    return index;
  }

  inline GlobalIndex popFromFloodfillFrontier() {
    GlobalIndex index = floodfill_frontier_.front();
    floodfill_frontier_.pop();
    return index;
  }

  inline void clearNewConnections(bool clear_modified_voxels) {
    checked_edges_.clear();
    connected_edges_.clear();
    if (clear_modified_voxels) {
      modified_voxel_queue_ = AlignedQueue<GlobalIndex>();
    }
  }

  inline bool isVertex(const GvdLayer& layer,
                       const GvdVoxel& voxel,
                       const GlobalIndex& index) {
    if (voxel.num_extra_basis >= config_.min_vertex_basis) {
      return true;
    }

    std::bitset<27> gvd_flags =
        extractNeighborhoodFlags(layer, index, config_.min_extra_basis);
    if (corner_finder_.match(gvd_flags)) {
      return true;
    }

    return false;
  }
};

}  // namespace topology
}  // namespace hydra
