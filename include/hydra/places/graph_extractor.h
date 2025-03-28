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
#include "hydra/places/graph_extractor_utilities.h"
#include "hydra/places/gvd_graph.h"
#include "hydra/places/gvd_merge_policies.h"
#include "hydra/places/gvd_voxel.h"
#include "hydra/reconstruction/voxel_types.h"

namespace hydra::places {

struct GvdParentTracker;

struct IndexVoxelPair {
  GlobalIndex index;
  const GvdVoxel* voxel;
};

struct CompressedNode {
  using CompressedNodeMap = std::unordered_map<uint64_t, CompressedNode>;
  uint64_t node_id;
  std::set<uint64_t> siblings;
  std::set<uint64_t> active_refs;
  std::set<uint64_t> archived_refs;
  std::unordered_map<uint64_t, std::map<uint64_t, uint64_t>> sibling_support;
  std::map<uint64_t, uint64_t> sibling_ref_counts;
  uint64_t best_gvd_id;
  bool in_graph = false;

  explicit CompressedNode(uint64_t node_id);

  void addEdgeObservation(uint64_t gvd_id,
                          uint64_t neighbor_gvd_id,
                          uint64_t sibling_id);

  bool removeEdgeObservation(uint64_t gvd_id, uint64_t neighbor_gvd_id);

  std::list<uint64_t> removeEdgeObservations(uint64_t gvd_id, CompressedNodeMap& nodes);

  void mergeObservations(uint64_t original_id, uint64_t new_id);

  void merge(CompressedNode& other, CompressedNodeMap& nodes);
};

using IndexVoxelQueue = std::list<IndexVoxelPair>;

class GraphExtractor {
 public:
  using Ptr = std::shared_ptr<GraphExtractor>;
  using NodeIndexMap = std::unordered_map<NodeId, GlobalIndex>;

  struct Config {
    //! Average resolution of sparse graph
    double compression_distance_m = 0.5;
    //! Validate graph properties
    bool validate_graph = false;
    //! Minimum distance for a GVD node to be considered for compression
    double min_node_distance_m = 0.3;
    //! Minimum distance for a edge to be valid
    double min_edge_distance_m = 0.4;
    //! Whether to merge nearby neighbors
    bool merge_nearby_nodes = true;
    //! Distance to consider a merge at
    double node_merge_distance_m = 0.3;
    //! Merge policy
    std::string merge_policy = "basis_points";
    //! Whether or not to add heuristic edges
    bool add_heuristic_edges = false;
    //! Add edges between nodes that have overlapping free-space regions
    bool add_overlap_edges = true;
    //! Configuration for overlap-based edges
    OverlapEdgeConfig overlap_edges;
    //! Add edges between disconnected components
    bool add_freespace_edges = true;
    //! Configuration for freespace edges
    FreespaceEdgeConfig freespace_edges;
  } const config;

  explicit GraphExtractor(const Config& config);

  virtual ~GraphExtractor();

  void pushGvdIndex(const GlobalIndex& index);

  void clearGvdIndex(const GlobalIndex& index);

  void removeDistantIndex(const GlobalIndex& index);

  void extract(const GvdLayer& layer, uint64_t timestamp_ns);

  void fillParentInfo(const GvdLayer& gvd, const GvdParentTracker& parents);

  const SceneGraphLayer& getGraph() const;

  const GvdGraph& getGvdGraph() const;

  std::unordered_set<NodeId> getActiveNodes() const;

  const std::unordered_set<NodeId>& getDeletedNodes() const;

  const std::vector<NodeId>& getDeletedEdges() const;

  void clearDeleted();

  const NodeIndexMap& getIndexMap() const { return node_index_map_; }

  const std::unordered_map<uint64_t, CompressedNode>& getCompressedNodeInfo() const {
    return compressed_info_map_;
  }

  const std::unordered_map<uint64_t, uint64_t>& getCompressedRemapping() const {
    return compressed_remapping_;
  }

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

  void fillSeenVoxels(const GvdLayer& layer,
                      uint64_t timestamp_ns,
                      IndexVoxelQueue& seen_voxels);

  void updateNodeInfoMap(const GvdLayer& layer,
                         const IndexVoxelQueue& update_info,
                         uint64_t timestamp_ns);

  void updateGvdGraph(const IndexVoxelQueue& update_info, uint64_t timestamp_ns);

  void clearCompressionId(uint64_t node_id, bool is_delete);

  GlobalIndex getCompressedIndex(const Eigen::Vector3d& point) const;

  uint64_t getNextId();

  NodeId getPlaceId(uint64_t gvd_id) const;

  void compressNode(uint64_t node_id, const GvdMemberInfo& node);

  void assignCompressedNodeAttributes();

  void mergeNearbyNodes();

  void updateCompressedEdges(const GvdLayer& layer);

  void deleteChildren(const CompressedNode& info);

  void clearArchived();

  void updateNode(const GlobalIndex& index,
                  const Eigen::Vector3d& position,
                  double distance,
                  uint8_t num_basis_points);

  void validate(const GvdLayer& layer) const;

  std::optional<uint64_t> findCluster(uint64_t node_id,
                                      const GvdMemberInfo& node,
                                      const std::set<uint64_t>& clusters);

  void mergeCompressedNodes(uint64_t curr_node_id,
                            CompressedNode& curr_node,
                            uint64_t neighbor_node_id);

 protected:
  NodeSymbol next_node_id_;
  GvdGraph::Ptr gvd_;
  // just to make book-keeping easier
  SceneGraphLayer::Ptr graph_;

  NodeIndexMap node_index_map_;
  std::queue<GlobalIndex> modified_voxel_queue_;

  std::map<EdgeKey, bool> heuristic_edges_;
  std::unordered_set<NodeId> deleted_nodes_;
  std::vector<NodeId> deleted_edges_;

  const double compression_factor_;
  GlobalIndexMap<uint64_t> index_id_map_;
  std::unique_ptr<MergePolicy> merge_policy_;

  uint64_t next_id_;
  std::list<uint64_t> id_queue_;

  std::unordered_set<uint64_t> updated_nodes_;
  std::unordered_set<uint64_t> to_archive_;

  std::unordered_map<uint64_t, CompressedNode> compressed_info_map_;
  GlobalIndexMap<std::set<uint64_t>> compressed_index_map_;
  std::unordered_map<uint64_t, GlobalIndex> compressed_id_map_;
  std::unordered_map<uint64_t, uint64_t> compressed_remapping_;

  std::unordered_set<NodeId> archived_node_ids_;
};

void declare_config(GraphExtractor::Config& config);

}  // namespace hydra::places
