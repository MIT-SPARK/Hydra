#pragma once
#include "hydra_topology/voxblox_types.h"
#include "hydra_topology/dsg_types.h"

namespace hydra {
namespace topology {

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
  voxblox::LongIndexSet indices;
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
  voxblox::AlignedVector<GlobalIndex> indices;
};

}  // namespace topology
}  // namespace hydra
