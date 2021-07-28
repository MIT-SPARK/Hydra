#include "kimera_topology/graph_extractor.h"

#include <kimera_dsg/node_attributes.h>
#include <nanoflann.hpp>

namespace kimera {
namespace topology {

using GvdLayer = GraphExtractor::GvdLayer;

std::ostream& operator<<(std::ostream& out, const VoxelGraphInfo& info) {
  if (info.is_node) {
    out << "node " << NodeSymbol(info.id).getLabel();
  } else {
    out << "edge " << info.edge_id;
  }

  return out;
}

GraphExtractor::GraphExtractor(const GraphExtractorConfig& config)
    : config_(config),
      next_node_id_('p', 0),
      next_edge_id_(0),
      graph_(new IsolatedSceneGraphLayer(to_underlying(KimeraDsgLayers::PLACES))) {}

void GraphExtractor::addPlaceToGraph(const GvdLayer& layer,
                                     const GvdVoxel& voxel,
                                     const GlobalIndex& index,
                                     bool is_from_split) {
  if (index_info_map_.count(index)) {
    LOG(WARNING) << "[Graph Extractor] attempted to add duplicate node @ "
                 << index.transpose() << " with previous entry "
                 << index_info_map_.at(index);
    return;
  }

  NodeAttributes::Ptr attributes(
      new PlaceNodeAttributes(voxel.distance, voxel.num_extra_basis + 1));

  attributes->position = getVoxelPosition(layer, index);

  index_info_map_.emplace(index, VoxelGraphInfo(next_node_id_, is_from_split));
  id_index_map_[next_node_id_] = voxblox::LongIndexSet();
  id_root_index_map_[next_node_id_] = index;
  node_edge_id_map_[next_node_id_] = std::set<size_t>();
  node_edge_connections_[next_node_id_] = std::set<size_t>();

  graph_->emplaceNode(next_node_id_, std::move(attributes));
  next_node_id_++;
}

bool GraphExtractor::updateEdgeMaps(const VoxelGraphInfo& info,
                                    const VoxelGraphInfo& neighbor_info) {
  if (info.is_node) {
    // precondition is that both can't be nodes, so neighbor_info.edge_id is safe
    return node_edge_connections_[info.id].insert(neighbor_info.edge_id).second;
  }

  if (neighbor_info.is_node) {
    // precondition is both can't be nodes, so info.edge_id is safe
    return edge_info_map_.at(info.edge_id)
        .node_connections.insert(neighbor_info.id)
        .second;
  }

  return edge_info_map_.at(info.edge_id)
      .connections.insert(neighbor_info.edge_id)
      .second;
}

void GraphExtractor::addEdgeToGraph(const VoxelGraphInfo& curr_info,
                                    const VoxelGraphInfo& neighbor_info) {
  if (curr_info.is_node && neighbor_info.is_node) {
    // this case can happen (potentially frequently) if the basis count is noisy
    // ideally adjacent vertices get clustered downstream and pruned
    graph_->insertEdge(curr_info.id, neighbor_info.id);
    return;
  }

  if (!updateEdgeMaps(curr_info, neighbor_info)) {
    return;
  }

  // TODO(nathan) check if we really need symmetric lookup
  updateEdgeMaps(neighbor_info, curr_info);

  graph_->insertEdge(curr_info.id, neighbor_info.id);

  if (!curr_info.is_node) {
    connected_edges_.insert(curr_info.edge_id);
  }

  if (!neighbor_info.is_node) {
    connected_edges_.insert(neighbor_info.edge_id);
  }
}

using NfMatrix = Eigen::Matrix<int64_t, Eigen::Dynamic, 3>;

using NfKdTree = nanoflann::KDTreeEigenMatrixAdaptor<NfMatrix, 3>;

inline NfMatrix getIndexMatrixForEdge(const EdgeInfo& info) {
  NfMatrix matrix(info.indices.size(), 3);

  int insertion_id = 0;
  for (const auto& index : info.indices) {
    matrix.block<1, 3>(insertion_id, 0) = index.transpose();
    insertion_id++;
  }

  return matrix;
}

inline NfMatrix extendEdgeIndexMatrix(const NfMatrix& original_matrix,
                                      const EdgeInfo& other_info) {
  int num_nodes = original_matrix.rows() + other_info.indices.size();

  NfMatrix matrix(num_nodes, 3);
  matrix.topRows(original_matrix.rows()) = original_matrix;

  int insertion_id = original_matrix.rows();
  for (const auto& index : other_info.indices) {
    matrix.block<1, 3>(insertion_id, 0) = index.transpose();
    insertion_id++;
  }

  return matrix;
}

struct NearestIndexResult {
  int64_t distance = 0;
  int64_t index = 0;
};

NearestIndexResult findFurthestIndex(const NfKdTree& kdtree,
                                     const GlobalIndex start,
                                     const GlobalIndex& end) {
  voxblox::AlignedVector<GlobalIndex> line = makeBresenhamLine(start, end);

  NearestIndexResult result;

  // TODO(nathan) can also consider a size based check
  if (line.empty()) {
    return result;
  }

  for (const auto& index : line) {
    int64_t nn_index = 0;
    int64_t distance = 0;
    kdtree.index->knnSearch(index.data(), 1, &nn_index, &distance);

    if (distance >= result.distance) {
      result.distance = distance;
      result.index = nn_index;
    }
  }

  return result;
}

void GraphExtractor::findBadEdgeIndices(const EdgeInfo& info) {
  const GlobalIndex start = id_root_index_map_.at(info.source);
  NfMatrix edge_index_matrix = getIndexMatrixForEdge(info);

  checked_edges_[info.id] = info.connections;
  for (auto other_edge : info.connections) {
    if (checked_edges_.count(other_edge) &&
        checked_edges_.at(other_edge).count(info.id)) {
      continue;  // we've seen this before from the other direction
    }

    NodeId other_node = edge_info_map_.at(other_edge).source;
    NfMatrix index_matrix =
        extendEdgeIndexMatrix(edge_index_matrix, edge_info_map_.at(other_edge));
    NfKdTree kdtree(3, index_matrix);
    kdtree.index->buildIndex();

    const GlobalIndex end = id_root_index_map_.at(other_node);

    NearestIndexResult result = findFurthestIndex(kdtree, start, end);
    GlobalIndex furthest_index = index_matrix.block<1, 3>(result.index, 0).transpose();

    if (result.distance > config_.max_edge_deviation && result.distance > 0) {
      const size_t result_edge_id =
          static_cast<size_t>(result.index) < info.indices.size() ? info.id
                                                                  : other_edge;
      edge_deviation_queue_.emplace(furthest_index, result.distance, result_edge_id);
    }
  }

  // handle direct connections to nodes
  NfKdTree kdtree(3, edge_index_matrix);
  kdtree.index->buildIndex();
  for (auto other_node : info.node_connections) {
    const GlobalIndex end = id_root_index_map_.at(other_node);

    NearestIndexResult result = findFurthestIndex(kdtree, start, end);
    GlobalIndex furthest_index =
        edge_index_matrix.block<1, 3>(result.index, 0).transpose();

    if (result.distance > config_.max_edge_deviation && result.distance > 0) {
      edge_deviation_queue_.emplace(furthest_index, result.distance, info.id);
    }
  }
}

void GraphExtractor::findNewVertices(const GvdLayer& layer) {
  voxblox::LongIndexSet seen_nodes;

  while (!modified_voxel_queue_.empty()) {
    const GlobalIndex index = popFromModifiedGvd();

    if (seen_nodes.count(index)) {
      continue;
    }

    seen_nodes.insert(index);

    const auto& info_iter = index_info_map_.find(index);
    if (info_iter != index_info_map_.end() && info_iter->second.is_node &&
        info_iter->second.is_split_node) {
      // we don't check the vertex condition for nodes that were from edge
      // splits to reduce thrashing
      floodfill_frontier_.push(index);
      continue;
    }

    // TODO(nathan) slightly duplicated with neighbor flag extraction
    const GvdVoxel& voxel = *CHECK_NOTNULL(layer.getVoxelPtrByGlobalIndex(index));
    if (!isVertex(layer, voxel, index)) {
      if (info_iter != index_info_map_.end() && info_iter->second.is_node) {
        // node no longer matches criteria
        removeNodeInfo(info_iter->second.id);
      }
      continue;
    }

    if (info_iter != index_info_map_.end()) {
      if (info_iter->second.is_node) {
        continue;  // don't try to do anything with a valid previous vertex
      }
      // we just found a vertex, but we had an edge previously, so throw out the
      // previous edge
      removeEdgeInfo(info_iter->second.edge_id);
    }

    floodfill_frontier_.push(index);
    addPlaceToGraph(layer, voxel, index);
  }
}

void GraphExtractor::addNeighborToFrontier(const VoxelGraphInfo& info,
                                           const GlobalIndex& neighbor_index) {
  floodfill_frontier_.push(neighbor_index);
  VoxelGraphInfo neighbor_info(info);
  neighbor_info.is_node = false;

  if (info.is_node) {
    neighbor_info.edge_id = next_edge_id_;
    node_edge_id_map_[info.id].insert(next_edge_id_);
    edge_info_map_[next_edge_id_] = EdgeInfo(next_edge_id_, info.id);
    next_edge_id_++;
  }

  edge_info_map_[neighbor_info.edge_id].indices.insert(neighbor_index);

  index_info_map_[neighbor_index] = neighbor_info;
  id_index_map_[info.id].insert(neighbor_index);
}

void GraphExtractor::removeNodeInfo(NodeId node_id) {
  graph_->removeNode(node_id);

  if (config_.add_cleared_indices_to_wavefront) {
    modified_voxel_queue_.push(id_root_index_map_.at(node_id));
  }

  index_info_map_.erase(id_root_index_map_.at(node_id));
  id_root_index_map_.erase(node_id);

  // remove all GVD voxels that used to be flood-filled by this node
  for (const auto& index : id_index_map_.at(node_id)) {
    index_info_map_.erase(index);
    if (config_.add_cleared_indices_to_wavefront) {
      modified_voxel_queue_.push(index);
    }
  }
  id_index_map_.erase(node_id);

  // remove all edge book-keeping for the node
  for (size_t edge_id : node_edge_id_map_.at(node_id)) {
    removeEdgeInfo(edge_id, false);
  }
  node_edge_id_map_.erase(node_id);

  for (size_t edge_id : node_edge_connections_.at(node_id)) {
    edge_info_map_.at(edge_id).node_connections.erase(node_id);
  }
  node_edge_connections_.erase(node_id);
}

void GraphExtractor::removeEdgeInfo(size_t edge_id, bool clear_indices) {
  const auto edge_iter = edge_info_map_.find(edge_id);
  if (edge_iter == edge_info_map_.end()) {
    // TODO(nathan) think about warning
    return;
  }

  {  // start info reference lifetime
    const EdgeInfo& info = edge_iter->second;
    if (config_.add_cleared_indices_to_wavefront) {
      const auto& node_iter = id_root_index_map_.find(info.source);
      if (node_iter != id_root_index_map_.end()) {
        modified_voxel_queue_.push(node_iter->second);
      }
    }

    for (size_t other_edge_id : info.connections) {
      graph_->removeEdge(info.source, edge_info_map_.at(other_edge_id).source);
      edge_info_map_.at(other_edge_id).connections.erase(edge_id);
    }

    for (NodeId node_id : info.node_connections) {
      graph_->removeEdge(info.source, node_id);
      node_edge_connections_.at(node_id).erase(edge_id);
    }

    if (clear_indices) {
      // invalidate all indices for the specific edge
      for (const auto& index : info.indices) {
        index_info_map_.erase(index);
        if (config_.add_cleared_indices_to_wavefront) {
          modified_voxel_queue_.push(index);
        }

        id_index_map_.at(info.source).erase(index);
      }
    }
  }  // end info reference lifetime

  edge_info_map_.erase(edge_iter);
}

void GraphExtractor::clearGvdIndex(const GlobalIndex& index) {
  const auto info_iter = index_info_map_.find(index);
  if (info_iter == index_info_map_.end()) {
    return;
  }

  if (info_iter->second.is_node) {
    removeNodeInfo(info_iter->second.id);
  } else {
    removeEdgeInfo(info_iter->second.edge_id);
  }
}

void GraphExtractor::extractEdges(const GvdLayer& layer) {
  GvdNeighborhood::IndexMatrix neighbor_indices;

  while (!floodfill_frontier_.empty()) {
    const GlobalIndex index = popFromFloodfillFrontier();
    GvdNeighborhood::getFromGlobalIndex(index, &neighbor_indices);
    VoxelGraphInfo curr_info = index_info_map_.at(index);

    for (unsigned int n = 0u; n < neighbor_indices.cols(); ++n) {
      const GlobalIndex& neighbor_index = neighbor_indices.col(n);
      const GvdVoxel* neighbor = layer.getVoxelPtrByGlobalIndex(neighbor_index);
      if (!neighbor) {
        continue;
      }

      if (neighbor->num_extra_basis < config_.min_extra_basis) {
        continue;
      }

      const auto& neighbor_info_iter = index_info_map_.find(neighbor_index);
      if (neighbor_info_iter == index_info_map_.end()) {
        addNeighborToFrontier(curr_info, neighbor_index);
        continue;
      }

      if (neighbor_info_iter->second.id == curr_info.id) {
        continue;
      }

      addEdgeToGraph(curr_info, neighbor_info_iter->second);
    }
  }
}

void GraphExtractor::splitEdges(const GvdLayer& layer) {
  bool reached_max_iters = true;
  for (size_t iter = 0; iter < config_.max_edge_split_iterations; ++iter) {
    // identify best edge split candidates
    for (size_t edge_id : connected_edges_) {
      if (!edge_info_map_.count(edge_id)) {
        LOG(WARNING) << "[Graph Extractor] edge " << edge_id
                     << " does not exist any more";
        continue;
      }
      findBadEdgeIndices(edge_info_map_.at(edge_id));
    }

    checked_edges_.clear();
    connected_edges_.clear();

    if (edge_deviation_queue_.empty()) {
      reached_max_iters = false;
      break;  // no more work to do
    }

    while (!edge_deviation_queue_.empty()) {
      EdgeDeviation curr_voxel = edge_deviation_queue_.top();
      edge_deviation_queue_.pop();

      const auto& voxel_info_iter = index_info_map_.find(curr_voxel.index);
      if (voxel_info_iter == index_info_map_.end()) {
        continue;  // a split has cleared this index already
      }

      if (voxel_info_iter->second.is_node) {
        continue;  // a split has chosen this index as a node already
      }

      // add original node to floodfill frontier (to redo old edge)
      floodfill_frontier_.push(id_root_index_map_.at(voxel_info_iter->second.id));
      removeEdgeInfo(curr_voxel.edge_id);

      // add a new node and also push to floodfill frontier
      const GvdVoxel& voxel =
          *CHECK_NOTNULL(layer.getVoxelPtrByGlobalIndex(curr_voxel.index));
      addPlaceToGraph(layer, voxel, curr_voxel.index, true);
      floodfill_frontier_.push(curr_voxel.index);
    }

    // run flood-fill seeded from new nodes and split nodes
    extractEdges(layer);
  }

  // this gets filled because of edge removal (but we don't want it filled for the next
  // extraction pass) so we recreate it
  modified_voxel_queue_ = AlignedQueue<GlobalIndex>();
  checked_edges_.clear();
  connected_edges_.clear();

  if (reached_max_iters && config_.max_edge_split_iterations > 0) {
    LOG_FIRST_N(WARNING, 1)
        << "[Graph Extractor] Splitting edges during graph-extraction reached "
           "the maximum number of "
           "iterations. Extracted graph edges may not match GVD edges. Consider "
           "increasing maximum edge splitting iterations";
  }
}

void GraphExtractor::extract(const GvdLayer& layer) {
  // identify any new vertices (corners and voxels with enough basis)
  findNewVertices(layer);

  // extract straight-line edges between new vertices via flood-fill
  extractEdges(layer);

  // insert new voxels for any parts of the flood-filled edges that deviate too much
  // from the straight-line edge
  splitEdges(layer);
}

}  // namespace topology
}  // namespace kimera
