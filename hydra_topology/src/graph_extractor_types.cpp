#include "hydra_topology/graph_extractor_types.h"

namespace hydra {
namespace topology {

VoxelGraphInfo::VoxelGraphInfo() : is_node(false), is_split_node(false) {}

VoxelGraphInfo::VoxelGraphInfo(NodeId id, bool is_from_split)
    : id(id), is_node(true), is_split_node(is_from_split) {}

EdgeInfo::EdgeInfo(size_t id, NodeId source) : id(id), source(source) {}

EdgeSplitSeed::EdgeSplitSeed(const GlobalIndex& index,
                             double distance_to_edge,
                             size_t edge_id)
    : index(index), distance_to_edge(distance_to_edge), edge_id(edge_id) {}

bool operator<(const EdgeSplitSeed& lhs, const EdgeSplitSeed& rhs) {
  return lhs.distance_to_edge < rhs.distance_to_edge;
}

}  // namespace topology
}  // namespace hydra
