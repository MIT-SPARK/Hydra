#pragma once
#include "kimera_topology/graph_extraction_utilities.h"
#include "kimera_topology/gvd_voxel.h"
#include "kimera_topology/voxblox_types.h"

#include <kimera_dsg/scene_graph_layer.h>

#include <queue>

namespace kimera {
namespace topology {

inline Eigen::Vector3d getVoxelPosition(const Layer<GvdVoxel>& layer,
                                        const GlobalIndex& index) {
  BlockIndex block_idx;
  VoxelIndex voxel_idx;
  voxblox::getBlockAndVoxelIndexFromGlobalVoxelIndex(
      index, layer.voxels_per_side(), &block_idx, &voxel_idx);

  return layer.getBlockByIndex(block_idx)
      .computeCoordinatesFromVoxelIndex(voxel_idx)
      .cast<double>();
}

struct GraphExtractorConfig {
  uint8_t min_extra_basis = 2;
  uint8_t min_vertex_basis = 3;
  int64_t max_edge_deviation = 4;
  size_t max_edge_split_iterations = 5;
  bool add_cleared_indices_to_wavefront = true;
};

struct VoxelGraphInfo {
  VoxelGraphInfo() : is_node(false), is_split_node(false) {}

  explicit VoxelGraphInfo(NodeId id, bool is_from_split)
      : id(id), is_node(true), is_split_node(is_from_split) {}

  NodeId id;
  bool is_node;
  bool is_split_node;
  size_t edge_id;
};

struct EdgeInfo {
  EdgeInfo() = default;

  EdgeInfo(size_t id, NodeId source) : id(id), source(source) {}

  size_t id;
  NodeId source;
  voxblox::LongIndexSet indices;
  std::set<NodeId> node_connections;
  std::set<size_t> connections;
};

struct EdgeDeviation {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeDeviation(const GlobalIndex& index, double distance_to_edge, size_t edge_id)
      : index(index), distance_to_edge(distance_to_edge), edge_id(edge_id) {}

  GlobalIndex index;
  double distance_to_edge;
  size_t edge_id;
};

inline bool operator<(const EdgeDeviation& lhs, const EdgeDeviation& rhs) {
  return lhs.distance_to_edge < rhs.distance_to_edge;
}

// TODO(nathan) consider namespacing in class
// TODO(nathan) may need aligned allocator
using IdIndexMap = std::unordered_map<NodeId, GlobalIndex>;
using IdIndiceMap = std::unordered_map<NodeId, voxblox::LongIndexSet>;
using IndexInfoMap = voxblox::LongIndexHashMapType<VoxelGraphInfo>::type;
using EdgeInfoMap = std::unordered_map<size_t, EdgeInfo>;
using EdgeDeviationQueue =
    std::priority_queue<EdgeDeviation, voxblox::AlignedVector<EdgeDeviation>>;

class GraphExtractor {
 public:
  using Ptr = std::unique_ptr<GraphExtractor>;
  using GvdLayer = Layer<GvdVoxel>;

  explicit GraphExtractor(const GraphExtractorConfig& config);

  inline void pushGvdIndex(const GlobalIndex& index) {
    modified_voxel_queue_.push(index);
  }

  void clearGvdIndex(const GlobalIndex& index);

  void extract(const GvdLayer& layer);

  inline const SceneGraphLayer& getGraph() const { return *graph_; }

  inline const EdgeInfoMap& getGvdEdgeInfo() const { return edge_info_map_; }

  inline const IdIndexMap& getNodeIndexMap() const { return id_root_index_map_; }

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

  void removeNodeInfo(NodeId node_id);

  void removeEdgeInfo(size_t edge_id, bool clear_indices = true);

  void addNeighborToFrontier(const VoxelGraphInfo& info,
                             const GlobalIndex& neighbor_index);

  void addPlaceToGraph(const GvdLayer& layer,
                       const GvdVoxel& voxel,
                       const GlobalIndex& index,
                       bool is_from_split = false);

  void addEdgeToGraph(const VoxelGraphInfo& curr_info,
                      const VoxelGraphInfo& neighbor_info);

  void findBadEdgeIndices(const EdgeInfo& info);

  void findNewVertices(const GvdLayer& layer);

  void extractEdges(const GvdLayer& layer);

  void splitEdges(const GvdLayer& layer);

  bool updateEdgeMaps(const VoxelGraphInfo& info, const VoxelGraphInfo& neighbor_info);

  GraphExtractorConfig config_;

  AlignedQueue<GlobalIndex> modified_voxel_queue_;
  AlignedQueue<GlobalIndex> floodfill_frontier_;
  CornerFinder corner_finder_;

  NodeSymbol next_node_id_;
  size_t next_edge_id_;
  IndexInfoMap index_info_map_;
  IdIndiceMap id_index_map_;
  IdIndexMap id_root_index_map_;

  EdgeInfoMap edge_info_map_;
  std::map<NodeId, std::set<size_t>> node_edge_id_map_;
  std::map<NodeId, std::set<size_t>> node_edge_connections_;

  // TODO(nathan) rename all of these
  EdgeDeviationQueue edge_deviation_queue_;
  std::map<size_t, std::set<size_t>> checked_edges_;
  std::set<size_t> connected_edges_;

  IsolatedSceneGraphLayer::Ptr graph_;
};

}  // namespace topology
}  // namespace kimera
