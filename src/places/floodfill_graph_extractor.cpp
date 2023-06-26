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
#include "hydra/places/floodfill_graph_extractor.h"

#include "hydra/places/graph_extractor_utilities.h"
#include "hydra/places/nearest_voxel_utilities.h"

namespace hydra {
namespace places {

using GvdLayer = FloodfillGraphExtractor::GvdLayer;
using GlobalIndexVector = voxblox::AlignedVector<GlobalIndex>;

FloodfillGraphExtractor::FloodfillGraphExtractor(const GraphExtractorConfig& config)
    : GraphExtractorInterface(config), config_(config.floodfill), next_edge_id_(0) {}

FloodfillGraphExtractor::~FloodfillGraphExtractor() = default;

void FloodfillGraphExtractor::clearGvdIndex(const GlobalIndex& index) {
  const auto& info_iter = index_graph_info_map_.find(index);
  if (info_iter == index_graph_info_map_.end()) {
    return;
  }

  if (info_iter->second.is_node) {
    clearNodeInfo(info_iter->second.id);
  } else {
    clearEdgeInfo(info_iter->second.edge_id);
  }
}

void FloodfillGraphExtractor::removeDistantIndex(const GlobalIndex& index) {
  const auto& info_iter = index_graph_info_map_.find(index);
  if (info_iter == index_graph_info_map_.end()) {
    return;
  }

  if (info_iter->second.is_node) {
    removeNodeIndex(info_iter->second.id);
  } else {
    removeEdgeIndices(info_iter->second.edge_id, true);
  }
}

void FloodfillGraphExtractor::clearNodeInfo(NodeId node_id) {
  removeGraphNode(node_id);
  modified_voxel_queue_.push(node_index_map_.at(node_id));

  index_graph_info_map_.erase(node_index_map_.at(node_id));
  node_index_map_.erase(node_id);

  // remove all GVD voxels that used to be flood-filled by this node
  for (const auto& index : node_child_map_.at(node_id)) {
    index_graph_info_map_.erase(index);
    modified_voxel_queue_.push(index);
  }
  node_child_map_.erase(node_id);

  // remove all edge book-keeping for the node
  for (size_t edge_id : node_edge_id_map_.at(node_id)) {
    clearEdgeInfo(edge_id, false);
  }
  node_edge_id_map_.erase(node_id);

  for (size_t edge_id : node_edge_connections_.at(node_id)) {
    edge_info_map_.at(edge_id).node_connections.erase(node_id);
  }
  node_edge_connections_.erase(node_id);
}

void FloodfillGraphExtractor::clearEdgeInfo(size_t edge_id, bool clear_indices) {
  const auto edge_iter = edge_info_map_.find(edge_id);
  if (edge_iter == edge_info_map_.end()) {
    // TODO(nathan) think about warning
    return;
  }

  {  // start info reference lifetime
    const EdgeInfo& info = edge_iter->second;
    const auto& node_iter = node_index_map_.find(info.source);
    if (node_iter != node_index_map_.end()) {
      modified_voxel_queue_.push(node_iter->second);
    }

    for (size_t other_edge_id : info.connections) {
      removeGraphEdge(info.source, edge_info_map_.at(other_edge_id).source);
      edge_info_map_.at(other_edge_id).connections.erase(edge_id);
    }

    for (NodeId node_id : info.node_connections) {
      removeGraphEdge(info.source, node_id);
      node_edge_connections_.at(node_id).erase(edge_id);
    }

    if (clear_indices) {
      // invalidate all indices for the specific edge
      for (const auto& index : info.indices) {
        index_graph_info_map_.erase(index);
        modified_voxel_queue_.push(index);

        node_child_map_.at(info.source).erase(index);
      }

      node_edge_id_map_.at(info.source).erase(edge_id);
    }
  }  // end info reference lifetime

  edge_info_map_.erase(edge_iter);
}

void FloodfillGraphExtractor::removeNodeIndex(NodeId node_id) {
  index_graph_info_map_.erase(node_index_map_.at(node_id));
  node_index_map_.erase(node_id);

  for (const auto& index : node_child_map_.at(node_id)) {
    index_graph_info_map_.erase(index);
  }
  node_child_map_.erase(node_id);

  for (size_t edge_id : node_edge_id_map_.at(node_id)) {
    removeEdgeIndices(edge_id, false);
  }
  node_edge_id_map_.erase(node_id);

  for (size_t edge_id : node_edge_connections_.at(node_id)) {
    edge_info_map_.at(edge_id).node_connections.erase(node_id);
  }
  node_edge_connections_.erase(node_id);
}

void FloodfillGraphExtractor::removeEdgeIndices(size_t edge_id, bool clear_indices) {
  const auto edge_iter = edge_info_map_.find(edge_id);
  if (edge_iter == edge_info_map_.end()) {
    return;
  }

  {  // start info reference lifetime
    const EdgeInfo& info = edge_iter->second;

    for (size_t other_edge_id : info.connections) {
      edge_info_map_.at(other_edge_id).connections.erase(edge_id);
    }

    for (NodeId node_id : info.node_connections) {
      node_edge_connections_.at(node_id).erase(edge_id);
    }

    if (clear_indices) {
      for (const auto& index : info.indices) {
        index_graph_info_map_.erase(index);
        node_child_map_.at(info.source).erase(index);
      }

      node_edge_id_map_.at(info.source).erase(edge_id);
    }
  }  // end info reference lifetime

  edge_info_map_.erase(edge_iter);
}

void FloodfillGraphExtractor::addNeighborToFrontier(const VoxelGraphInfo& info,
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

  index_graph_info_map_[neighbor_index] = neighbor_info;
  node_child_map_[info.id].insert(neighbor_index);
}

bool FloodfillGraphExtractor::updateEdgeMaps(const VoxelGraphInfo& info,
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

void FloodfillGraphExtractor::addEdgeToGraph(const GvdLayer& layer,
                                             const VoxelGraphInfo& curr_info,
                                             const VoxelGraphInfo& neighbor_info) {
  if (curr_info.is_node && neighbor_info.is_node) {
    // this case can happen (potentially frequently) if the basis count is noisy
    // ideally adjacent vertices get clustered downstream and pruned
    updateGraphEdge(curr_info.id,
                    neighbor_info.id,
                    makeEdgeInfo(layer, curr_info.id, neighbor_info.id));
    return;
  }

  if (!updateEdgeMaps(curr_info, neighbor_info)) {
    return;
  }

  // TODO(nathan) check if we really need symmetric lookup
  updateEdgeMaps(neighbor_info, curr_info);

  updateGraphEdge(curr_info.id,
                  neighbor_info.id,
                  makeEdgeInfo(layer, curr_info.id, neighbor_info.id));

  if (!curr_info.is_node) {
    connected_edges_.insert(curr_info.edge_id);
  }

  if (!neighbor_info.is_node) {
    connected_edges_.insert(neighbor_info.edge_id);
  }
}

void FloodfillGraphExtractor::findBadEdgeIndices(const EdgeInfo& info) {
  const GlobalIndex start = node_index_map_.at(info.source);
  voxblox::AlignedVector<GlobalIndex> indices(info.indices.begin(), info.indices.end());

  checked_edges_[info.id] = info.connections;
  for (auto other_edge : info.connections) {
    if (checked_edges_.count(other_edge) &&
        checked_edges_.at(other_edge).count(info.id)) {
      continue;  // we've seen this before from the other direction
    }

    const NodeId other_node = edge_info_map_.at(other_edge).source;
    const GlobalIndex end = node_index_map_.at(other_node);

    voxblox::AlignedVector<GlobalIndex> curr_indices(indices);
    curr_indices.insert(curr_indices.end(),
                        edge_info_map_.at(other_edge).indices.begin(),
                        edge_info_map_.at(other_edge).indices.end());

    FurthestIndexResult result =
        findFurthestIndexFromLine(curr_indices, start, end, indices.size());

    if (result.distance > config_.max_edge_deviation && result.valid) {
      edge_split_queue_.emplace(
          result.index, result.distance, result.from_source ? info.id : other_edge);
    }
  }

  for (auto other_node : info.node_connections) {
    const GlobalIndex end = node_index_map_.at(other_node);
    FurthestIndexResult result = findFurthestIndexFromLine(indices, start, end);

    if (result.distance > config_.max_edge_deviation && result.valid) {
      edge_split_queue_.emplace(result.index, result.distance, info.id);
    }
  }
}

void FloodfillGraphExtractor::findNewVertices(const GvdLayer& layer) {
  voxblox::LongIndexSet seen_nodes;

  while (!modified_voxel_queue_.empty()) {
    const GlobalIndex index = popFromModifiedQueue();

    if (seen_nodes.count(index)) {
      continue;
    }

    seen_nodes.insert(index);

    const auto& info_iter = index_graph_info_map_.find(index);
    if (info_iter != index_graph_info_map_.end() && info_iter->second.is_node &&
        info_iter->second.is_split_node) {
      // we don't check the vertex condition for nodes that were from edge
      // splits to reduce thrashing
      floodfill_frontier_.push(index);
      continue;
    }

    // TODO(nathan) slightly duplicated with neighbor flag extraction
    const GvdVoxel* voxel = layer.getVoxelPtrByGlobalIndex(index);
    if (voxel == nullptr) {
      VLOG(1) << "[Graph Extraction] Invalid index: " << index.transpose()
              << " found in modified queue";
      continue;
    }

    if (!isVertex(layer, *voxel, index)) {
      if (info_iter != index_graph_info_map_.end() && info_iter->second.is_node) {
        // node no longer matches criteria
        clearNodeInfo(info_iter->second.id);
      }
      continue;
    }

    if (info_iter != index_graph_info_map_.end()) {
      if (info_iter->second.is_node) {
        continue;  // don't try to do anything with a valid previous vertex
      }
      // we just found a vertex, but we had an edge previously, so throw out the
      // previous edge
      clearEdgeInfo(info_iter->second.edge_id);
    }

    floodfill_frontier_.push(index);
    // we use addPlaceToGraph for edge-splitting, so we need to grab the right node id
    // outside of addPlacetoGraph
    addNewPlaceNode(layer, *voxel, index);
  }
}

void FloodfillGraphExtractor::addNewPlaceNode(const GvdLayer& layer,
                                              const GvdVoxel& voxel,
                                              const GlobalIndex& index,
                                              bool is_from_split) {
  if (index_graph_info_map_.count(index)) {
    LOG(WARNING) << "[Graph Extractor] attempted to add duplicate node @ "
                 << index.transpose() << " with previous entry "
                 << index_graph_info_map_.at(index);
    return;
  }

  const NodeId new_node_id = addPlaceToGraph(layer, voxel, index);

  index_graph_info_map_.emplace(index, VoxelGraphInfo(new_node_id, is_from_split));
  node_child_map_[new_node_id] = voxblox::LongIndexSet();
  node_index_map_[new_node_id] = index;
  node_edge_id_map_[new_node_id] = std::set<size_t>();
  node_edge_connections_[new_node_id] = std::set<size_t>();
}

bool FloodfillGraphExtractor::attemptNodeMerge(const GvdLayer& layer,
                                               const VoxelGraphInfo& curr_info,
                                               const VoxelGraphInfo& neighbor_info) {
  const auto& graph = getGraph();
  const Eigen::Vector3d curr_pos =
      getVoxelPosition(layer, node_index_map_[curr_info.id]);
  const Eigen::Vector3d neighbor_pos =
      getVoxelPosition(layer, node_index_map_[neighbor_info.id]);
  if ((curr_pos - neighbor_pos).norm() >= config_.node_merge_distance_m) {
    return false;
  }

  const size_t curr_connections = graph.getNode(curr_info.id)->get().siblings().size();
  const size_t neighbor_connections =
      graph.getNode(neighbor_info.id)->get().siblings().size();

  if (curr_connections <= neighbor_connections) {
    clearNodeInfo(curr_info.id);
  } else {
    clearNodeInfo(neighbor_info.id);
  }

  return true;
}

void FloodfillGraphExtractor::extractEdges(const GvdLayer& layer, bool allow_merging) {
  GvdNeighborhood::IndexMatrix neighbor_indices;

  while (!floodfill_frontier_.empty()) {
    const GlobalIndex index = popFromFloodfillFrontier();
    GvdNeighborhood::getFromGlobalIndex(index, &neighbor_indices);
    if (!index_graph_info_map_.count(index)) {
      continue;  // partial wavefront from deleted node
    }

    VoxelGraphInfo curr_info = index_graph_info_map_.at(index);
    for (unsigned int n = 0u; n < neighbor_indices.cols(); ++n) {
      const GlobalIndex& neighbor_index = neighbor_indices.col(n);
      const GvdVoxel* neighbor = layer.getVoxelPtrByGlobalIndex(neighbor_index);
      if (!neighbor) {
        continue;
      }

      if (neighbor->num_extra_basis < config_.min_extra_basis) {
        continue;
      }

      const auto& neighbor_info_iter = index_graph_info_map_.find(neighbor_index);
      if (neighbor_info_iter == index_graph_info_map_.end()) {
        addNeighborToFrontier(curr_info, neighbor_index);
        continue;
      }

      if (neighbor_info_iter->second.id == curr_info.id) {
        continue;
      }

      if (allow_merging &&
          attemptNodeMerge(layer, curr_info, neighbor_info_iter->second)) {
        if (!node_index_map_.count(curr_info.id)) {
          break;  // we deleted ourselves, don't do anything else
        } else {
          // neighbor is gone, fine to continue edge
          addNeighborToFrontier(curr_info, neighbor_index);
          continue;
        }
      }

      addEdgeToGraph(layer, curr_info, neighbor_info_iter->second);
    }
  }
}

void FloodfillGraphExtractor::filterRemovedConnections() {
  std::vector<size_t> bad_edges;
  // prune removed edges before edge splitting
  for (const auto edge : connected_edges_) {
    if (!edge_info_map_.count(edge)) {
      bad_edges.push_back(edge);
    }
  }

  for (const auto bad_edge : bad_edges) {
    connected_edges_.erase(bad_edge);
  }
}

void FloodfillGraphExtractor::splitEdges(const GvdLayer& layer) {
  bool reached_max_iters = true;
  for (size_t iter = 0; iter < config_.max_edge_split_iterations; ++iter) {
    // identify best edge split candidates
    for (size_t edge_id : connected_edges_) {
      if (!edge_info_map_.count(edge_id)) {
        LOG(WARNING) << "[Graph Extractor] edge " << edge_id << "does not exists";
        continue;
      }
      findBadEdgeIndices(edge_info_map_.at(edge_id));
    }

    clearNewConnections(false);  // clear new edges that we processed

    if (edge_split_queue_.empty()) {
      reached_max_iters = false;
      break;  // no more work to do
    }

    while (!edge_split_queue_.empty()) {
      EdgeSplitSeed curr_voxel = edge_split_queue_.top();
      edge_split_queue_.pop();

      const auto& voxel_info_iter = index_graph_info_map_.find(curr_voxel.index);
      if (voxel_info_iter == index_graph_info_map_.end()) {
        continue;  // a split has cleared this index already
      }

      if (voxel_info_iter->second.is_node) {
        continue;  // a split has chosen this index as a node already
      }

      // add original node to floodfill frontier (to redo old edge)
      floodfill_frontier_.push(node_index_map_.at(voxel_info_iter->second.id));
      clearEdgeInfo(curr_voxel.edge_id);

      // add a new node and also push to floodfill frontier
      const GvdVoxel& voxel =
          *CHECK_NOTNULL(layer.getVoxelPtrByGlobalIndex(curr_voxel.index));
      addNewPlaceNode(layer, voxel, curr_voxel.index, true);
      floodfill_frontier_.push(curr_voxel.index);
    }

    // run flood-fill seeded from new nodes and split nodes
    // using node merging here means that edge splitting possible will not
    // terminate naturally (i.e. merged nodes and split nodes will thrash)
    extractEdges(layer, config_.edge_splitting_merge_nodes);
    if (config_.edge_splitting_merge_nodes) {
      filterRemovedConnections();
    }
  }

  // also clear indices from the modified voxel queue to prevent corruption
  clearNewConnections(true);

  if (reached_max_iters && config_.max_edge_split_iterations > 0) {
    if (VLOG_IS_ON(1)) {
      LOG_FIRST_N(WARNING, 1)
          << "[Graph Extractor] Splitting edges during graph-extraction reached "
             "the maximum number of "
             "iterations. Extracted graph edges may not match GVD edges. Consider "
             "increasing maximum edge splitting iterations";
    }
  }
}

void FloodfillGraphExtractor::extract(const GvdLayer& layer, uint64_t) {
  // find initial sparse graph
  findNewVertices(layer);
  extractEdges(layer, config_.merge_new_nodes);

  if (config_.merge_new_nodes) {
    // extract edges will maintain bad references in connected_edges_ when
    // merging nodes (can delete a newly added edge)
    filterRemovedConnections();
  }

  // repair graph to better match underlying gvd
  splitEdges(layer);
  updateHeuristicEdges(layer);
}

GlobalIndex FloodfillGraphExtractor::popFromFloodfillFrontier() {
  GlobalIndex index = floodfill_frontier_.front();
  floodfill_frontier_.pop();
  return index;
}

void FloodfillGraphExtractor::clearNewConnections(bool clear_modified_voxels) {
  checked_edges_.clear();
  connected_edges_.clear();
  if (clear_modified_voxels) {
    modified_voxel_queue_ = AlignedQueue<GlobalIndex>();
  }
}

bool FloodfillGraphExtractor::isVertex(const GvdLayer& layer,
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

}  // namespace places
}  // namespace hydra
