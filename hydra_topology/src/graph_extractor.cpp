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
#include "hydra_topology/graph_extractor.h"
#include "hydra_topology/nearest_neighbor_utilities.h"

namespace hydra {
namespace topology {

using GvdLayer = GraphExtractor::GvdLayer;
using Components = GraphExtractor::Components;
using GlobalIndexVector = voxblox::AlignedVector<GlobalIndex>;

// TODO(nathan) relocate
std::ostream& operator<<(std::ostream& out, const VoxelGraphInfo& info) {
  if (info.is_node) {
    out << "node " << NodeSymbol(info.id).getLabel();
  } else {
    out << "edge " << info.edge_id;
  }

  return out;
}

std::ostream& operator<<(std::ostream& out, const EdgeInfo& info) {
  out << "source: " << NodeSymbol(info.id).getLabel() << ", id: " << info.id
      << ", size: " << info.indices.size()
      << ", connections: " << info.connections.size()
      << ", node connections: " << info.node_connections.size();
  return out;
}

GraphExtractor::GraphExtractor(const GraphExtractorConfig& config)
    : config_(config),
      next_node_id_('p', 0),
      next_edge_id_(0),
      next_pseudo_edge_id_(0),
      graph_(new IsolatedSceneGraphLayer(DsgLayers::PLACES)) {}

std::unordered_set<NodeId> GraphExtractor::getActiveNodes() const {
  std::unordered_set<NodeId> nodes;
  for (const auto& id_index_pair : node_id_root_map_) {
    nodes.insert(id_index_pair.first);
  }
  return nodes;
}

std::unordered_set<NodeId> GraphExtractor::getDeletedNodes() const {
  return deleted_nodes_;
}

void GraphExtractor::clearDeletedNodes() { deleted_nodes_.clear(); }

void GraphExtractor::clearGvdIndex(const GlobalIndex& index) {
  const auto& info_iter = index_graph_info_map_.find(index);
  if (info_iter == index_graph_info_map_.end()) {
    return;
  }

  if (info_iter->second.is_node) {
    clearNodeInfo(info_iter->second.id);
  } else {
    clearEdgeInfo(info_iter->second.edge_id);
  }

  clearPseudoEdgeInfo(index);
}

void GraphExtractor::clearNodeInfo(NodeId node_id) {
  if (graph_->hasNode(node_id)) {
    for (const auto sibling : graph_->getNode(node_id).value().get().siblings()) {
      visited_nodes_.insert(sibling);  // unclear why this is required
    }
  }
  graph_->removeNode(node_id);
  deleted_nodes_.insert(node_id);
  modified_voxel_queue_.push(node_id_root_map_.at(node_id));

  index_graph_info_map_.erase(node_id_root_map_.at(node_id));
  node_id_root_map_.erase(node_id);

  // remove all GVD voxels that used to be flood-filled by this node
  for (const auto& index : node_id_index_map_.at(node_id)) {
    index_graph_info_map_.erase(index);
    modified_voxel_queue_.push(index);
  }
  node_id_index_map_.erase(node_id);

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

void GraphExtractor::clearEdgeInfo(size_t edge_id, bool clear_indices) {
  const auto edge_iter = edge_info_map_.find(edge_id);
  if (edge_iter == edge_info_map_.end()) {
    // TODO(nathan) think about warning
    return;
  }

  {  // start info reference lifetime
    const EdgeInfo& info = edge_iter->second;
    const auto& node_iter = node_id_root_map_.find(info.source);
    if (node_iter != node_id_root_map_.end()) {
      modified_voxel_queue_.push(node_iter->second);
    }

    visited_nodes_.insert(info.source);

    for (size_t other_edge_id : info.connections) {
      visited_nodes_.insert(edge_info_map_.at(other_edge_id).source);
      graph_->removeEdge(info.source, edge_info_map_.at(other_edge_id).source);
      edge_info_map_.at(other_edge_id).connections.erase(edge_id);
    }

    for (NodeId node_id : info.node_connections) {
      visited_nodes_.insert(node_id);
      graph_->removeEdge(info.source, node_id);
      node_edge_connections_.at(node_id).erase(edge_id);
    }

    if (clear_indices) {
      // invalidate all indices for the specific edge
      for (const auto& index : info.indices) {
        index_graph_info_map_.erase(index);
        modified_voxel_queue_.push(index);

        node_id_index_map_.at(info.source).erase(index);
      }

      node_edge_id_map_.at(info.source).erase(edge_id);
    }
  }  // end info reference lifetime

  edge_info_map_.erase(edge_iter);
}

void GraphExtractor::clearPseudoEdgeInfo(const GlobalIndex& root_index) {
  const auto& info_iter = pseudo_edge_map_.find(root_index);
  if (info_iter == pseudo_edge_map_.end()) {
    return;
  }

  std::unordered_set<size_t> edges_to_erase = info_iter->second;
  for (const auto edge_id : edges_to_erase) {
    const PseudoEdgeInfo& edge_info = pseudo_edge_info_.at(edge_id);

    for (const auto node : edge_info.nodes) {
      graph_->removeNode(node);
    }

    for (const auto index : edge_info.indices) {
      pseudo_edge_map_[index].erase(edge_id);

      if (pseudo_edge_map_[index].empty()) {
        pseudo_edge_map_.erase(index);
      }
    }

    pseudo_edge_info_.erase(edge_id);
  }
}

void GraphExtractor::removeDistantIndex(const GlobalIndex& index) {
  const auto& info_iter = index_graph_info_map_.find(index);
  if (info_iter == index_graph_info_map_.end()) {
    return;
  }

  if (info_iter->second.is_node) {
    removeNodeIndex(info_iter->second.id);
  } else {
    removeEdgeIndices(info_iter->second.edge_id, true);
  }

  clearPseudoEdgeInfo(index);
}

void GraphExtractor::removeNodeIndex(NodeId node_id) {
  index_graph_info_map_.erase(node_id_root_map_.at(node_id));
  node_id_root_map_.erase(node_id);

  for (const auto& index : node_id_index_map_.at(node_id)) {
    index_graph_info_map_.erase(index);
  }
  node_id_index_map_.erase(node_id);

  for (size_t edge_id : node_edge_id_map_.at(node_id)) {
    removeEdgeIndices(edge_id, false);
  }
  node_edge_id_map_.erase(node_id);

  for (size_t edge_id : node_edge_connections_.at(node_id)) {
    edge_info_map_.at(edge_id).node_connections.erase(node_id);
  }
  node_edge_connections_.erase(node_id);
}

void GraphExtractor::removeEdgeIndices(size_t edge_id, bool clear_indices) {
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
        node_id_index_map_.at(info.source).erase(index);
      }

      node_edge_id_map_.at(info.source).erase(edge_id);
    }
  }  // end info reference lifetime

  edge_info_map_.erase(edge_iter);
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

  index_graph_info_map_[neighbor_index] = neighbor_info;
  node_id_index_map_[info.id].insert(neighbor_index);
}

void GraphExtractor::addPlaceToGraph(const GvdLayer& layer,
                                     const GvdVoxel& voxel,
                                     const GlobalIndex& index,
                                     bool is_from_split) {
  if (index_graph_info_map_.count(index)) {
    LOG(WARNING) << "[Graph Extractor] attempted to add duplicate node @ "
                 << index.transpose() << " with previous entry "
                 << index_graph_info_map_.at(index);
    return;
  }

  PlaceNodeAttributes::Ptr attributes(
      new PlaceNodeAttributes(voxel.distance, voxel.num_extra_basis + 1));

  attributes->position = getVoxelPosition(layer, index);
  attributes->color = decltype(attributes->color)::Zero();

  index_graph_info_map_.emplace(index, VoxelGraphInfo(next_node_id_, is_from_split));
  node_id_index_map_[next_node_id_] = voxblox::LongIndexSet();
  node_id_root_map_[next_node_id_] = index;
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

EdgeAttributes::Ptr GraphExtractor::makeEdgeInfo(const GvdLayer& layer,
                                                 NodeId source_id,
                                                 NodeId target_id) const {
  const double source_dist =
      graph_->getNode(source_id)->get().attributes<PlaceNodeAttributes>().distance;
  const double target_dist =
      graph_->getNode(target_id)->get().attributes<PlaceNodeAttributes>().distance;

  double min_weight = std::min(source_dist, target_dist);

  const GlobalIndex source = node_id_root_map_.at(source_id);
  const GlobalIndex target = node_id_root_map_.at(target_id);
  GlobalIndexVector path = makeBresenhamLine(source, target);
  if (path.empty()) {
    // edge is smaller than voxel size, so we just take the min distance between two
    // voxels
    return std::make_unique<EdgeAttributes>(min_weight);
  }

  for (const auto& index : path) {
    const GvdVoxel* voxel = layer.getVoxelPtrByGlobalIndex(index);
    if (!voxel) {
      continue;
    }

    if (voxel->distance < min_weight) {
      min_weight = voxel->distance;
    }
  }

  return std::make_unique<EdgeAttributes>(min_weight);
}

void GraphExtractor::addEdgeToGraph(const GvdLayer& layer,
                                    const VoxelGraphInfo& curr_info,
                                    const VoxelGraphInfo& neighbor_info) {
  if (curr_info.is_node && neighbor_info.is_node) {
    // this case can happen (potentially frequently) if the basis count is noisy
    // ideally adjacent vertices get clustered downstream and pruned
    graph_->insertEdge(curr_info.id,
                       neighbor_info.id,
                       makeEdgeInfo(layer, curr_info.id, neighbor_info.id));
    return;
  }

  if (!updateEdgeMaps(curr_info, neighbor_info)) {
    return;
  }

  // TODO(nathan) check if we really need symmetric lookup
  updateEdgeMaps(neighbor_info, curr_info);

  graph_->insertEdge(curr_info.id,
                     neighbor_info.id,
                     makeEdgeInfo(layer, curr_info.id, neighbor_info.id));

  if (!curr_info.is_node) {
    connected_edges_.insert(curr_info.edge_id);
  }

  if (!neighbor_info.is_node) {
    connected_edges_.insert(neighbor_info.edge_id);
  }
}

void GraphExtractor::findBadEdgeIndices(const EdgeInfo& info) {
  const GlobalIndex start = node_id_root_map_.at(info.source);
  voxblox::AlignedVector<GlobalIndex> indices(info.indices.begin(), info.indices.end());

  checked_edges_[info.id] = info.connections;
  for (auto other_edge : info.connections) {
    if (checked_edges_.count(other_edge) &&
        checked_edges_.at(other_edge).count(info.id)) {
      continue;  // we've seen this before from the other direction
    }

    const NodeId other_node = edge_info_map_.at(other_edge).source;
    const GlobalIndex end = node_id_root_map_.at(other_node);

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
    const GlobalIndex end = node_id_root_map_.at(other_node);
    FurthestIndexResult result = findFurthestIndexFromLine(indices, start, end);

    if (result.distance > config_.max_edge_deviation && result.valid) {
      edge_split_queue_.emplace(result.index, result.distance, info.id);
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
    addPlaceToGraph(layer, *voxel, index);
  }
}

bool GraphExtractor::attemptNodeMerge(const GvdLayer& layer,
                                      const VoxelGraphInfo& curr_info,
                                      const VoxelGraphInfo& neighbor_info) {
  const Eigen::Vector3d curr_pos =
      getVoxelPosition(layer, node_id_root_map_[curr_info.id]);
  const Eigen::Vector3d neighbor_pos =
      getVoxelPosition(layer, node_id_root_map_[neighbor_info.id]);
  if ((curr_pos - neighbor_pos).norm() >= config_.node_merge_distance_m) {
    return false;
  }

  const size_t curr_connections =
      graph_->getNode(curr_info.id)->get().siblings().size();
  const size_t neighbor_connections =
      graph_->getNode(neighbor_info.id)->get().siblings().size();

  if (curr_connections <= neighbor_connections) {
    clearNodeInfo(curr_info.id);
  } else {
    clearNodeInfo(neighbor_info.id);
  }

  return true;
}

void GraphExtractor::extractEdges(const GvdLayer& layer, bool allow_merging) {
  GvdNeighborhood::IndexMatrix neighbor_indices;

  while (!floodfill_frontier_.empty()) {
    const GlobalIndex index = popFromFloodfillFrontier();
    GvdNeighborhood::getFromGlobalIndex(index, &neighbor_indices);
    if (!index_graph_info_map_.count(index)) {
      continue;  // partial wavefront from deleted node
    }

    VoxelGraphInfo curr_info = index_graph_info_map_.at(index);
    if (curr_info.is_node) {
      visited_nodes_.insert(curr_info.id);
    }

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
        if (!node_id_root_map_.count(curr_info.id)) {
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

void GraphExtractor::filterRemovedConnections() {
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

void GraphExtractor::splitEdges(const GvdLayer& layer) {
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
      floodfill_frontier_.push(node_id_root_map_.at(voxel_info_iter->second.id));
      clearEdgeInfo(curr_voxel.edge_id);

      // add a new node and also push to floodfill frontier
      const GvdVoxel& voxel =
          *CHECK_NOTNULL(layer.getVoxelPtrByGlobalIndex(curr_voxel.index));
      addPlaceToGraph(layer, voxel, curr_voxel.index, true);
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
    LOG_FIRST_N(WARNING, 1)
        << "[Graph Extractor] Splitting edges during graph-extraction reached "
           "the maximum number of "
           "iterations. Extracted graph edges may not match GVD edges. Consider "
           "increasing maximum edge splitting iterations";
  }
}

void GraphExtractor::findFreespaceEdges() {
  std::unordered_set<NodeId> root_nodes;
  for (const auto node : visited_nodes_) {
    // previous passes may delete nodes that we visited
    if (node_id_root_map_.count(node)) {
      root_nodes.insert(node);
    }
  }

  std::unordered_set<NodeId> active_neighborhood =
      graph_->getNeighborhood(root_nodes, config_.freespace_active_neighborhood_hops);

  NearestNodeFinder node_finder(*graph_, active_neighborhood);
  // kdtree will return the query node first
  const size_t num_to_find = config_.freespace_edge_num_neighbors + 1;

  for (const auto node : active_neighborhood) {
    node_finder.find(
        graph_->getPosition(node),
        num_to_find,
        true,
        [&](NodeId neighbor, size_t, double) {
          addFreespaceEdge(
              *graph_, node, neighbor, config_.freespace_edge_min_clearance_m);
        });
  }
}

Components GraphExtractor::filterComponents(const Components& to_filter) const {
  Components to_return;
  for (const auto& old_component : to_filter) {
    std::vector<NodeId> to_add;
    for (const auto node_id : old_component) {
      if (!node_id_root_map_.count(node_id)) {
        continue;  // gvd edge node
      }

      to_add.push_back(node_id);
    }

    if (to_add.empty()) {
      continue;
    }

    std::sort(to_add.begin(), to_add.end(), [&](const NodeId& lhs, const NodeId& rhs) {
      return getNodeGvdDistance(*graph_, lhs) > getNodeGvdDistance(*graph_, rhs);
    });

    to_return.push_back(to_add);
  }

  std::sort(to_return.begin(), to_return.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.size() > rhs.size();
  });

  return to_return;
}

bool GraphExtractor::addPseudoEdge(const GvdLayer& layer,
                                   NodeId node,
                                   NodeId other_node) {
  // TODO(nathan) remove when sure code is working
  CHECK(node_id_root_map_.count(node));
  CHECK(node_id_root_map_.count(other_node));
  const GlobalIndex source = node_id_root_map_[node];
  const GlobalIndex target = node_id_root_map_[other_node];

  if (graph_->hasEdge(node, other_node)) {
    return false;
  }

  // TODO(nathan) projective edge finding
  GlobalIndexVector path = makeBresenhamLine(source, target);
  if (path.empty()) {
    return false;
  }

  const double source_dist =
      graph_->getNode(node)->get().attributes<PlaceNodeAttributes>().distance;
  const double target_dist =
      graph_->getNode(other_node)->get().attributes<PlaceNodeAttributes>().distance;
  double min_weight = std::min(source_dist, target_dist);

  bool valid_path = true;
  for (const auto& index : path) {
    const GvdVoxel* voxel = layer.getVoxelPtrByGlobalIndex(index);
    if (!voxel) {
      return false;  // avoid adding edges along removed voxels
    }

    if (voxel->distance < min_weight) {
      min_weight = voxel->distance;
    }

    if (voxel->distance <= config_.component_min_clearance_m || !voxel->observed) {
      valid_path = false;
      break;
    }
  }

  if (!valid_path) {
    return false;
  }

  graph_->insertEdge(node, other_node, std::make_unique<EdgeAttributes>(min_weight));

  // no split nodes yet
  PseudoEdgeInfo info;
  info.indices = path;
  pseudo_edge_info_[next_pseudo_edge_id_] = info;

  for (const auto& index : path) {
    if (!pseudo_edge_map_.count(index)) {
      pseudo_edge_map_[index] = std::unordered_set<size_t>();
    }
    pseudo_edge_map_[index].insert(next_pseudo_edge_id_);
  }

  next_pseudo_edge_id_++;

  return true;
}

void GraphExtractor::findComponentConnections(const GvdLayer& layer) {
  std::unordered_set<NodeId> root_nodes;
  for (const auto& node_set : pseudo_edge_window_) {
    for (const auto node : node_set) {
      // previous passes may delete nodes that we visited
      if (node_id_root_map_.count(node)) {
        root_nodes.insert(node);
      }
    }
  }

  Components components = graph_utilities::getConnectedComponents<SceneGraphLayer>(
      *graph_, config_.connected_component_hops, root_nodes);

  Components filtered_components = filterComponents(components);
  if (filtered_components.size() <= 1) {
    return;  // nothing to do
  }

  std::vector<NodeId> largest_component = filtered_components.front();

  for (size_t i = 1; i < filtered_components.size(); ++i) {
    // TODO(nathan) this is potentially expensive (rebuilds the KD tree index every
    // time)
    NearestNodeFinder node_finder(*graph_, largest_component);

    const auto& component = filtered_components[i];
    const size_t num_to_check = component.size() < config_.component_nodes_to_check
                                    ? component.size()
                                    : config_.component_nodes_to_check;

    bool inserted_edge = false;
    for (size_t j = 0; j < num_to_check; ++j) {
      const NodeId node = component[j];
      const Eigen::Vector3d pos = graph_->getPosition(node);
      node_finder.find(pos,
                       config_.component_nearest_neighbors,
                       false,
                       [&](NodeId other_node, size_t, double distance) {
                         if (distance > config_.component_max_edge_length_m) {
                           return;
                         }
                         inserted_edge |= addPseudoEdge(layer, node, other_node);
                       });
    }

    if (inserted_edge) {
      // merge components if an edge was inserted
      largest_component.insert(
          largest_component.end(), component.begin(), component.end());
    }
  }
}

void GraphExtractor::filterIsolatedNodes() {
  std::list<NodeId> to_delete;
  for (const auto node : visited_nodes_) {
    if (!node_id_root_map_.count(node)) {
      if (graph_->hasNode(node)) {
        VLOG(1) << "node_id_root_map invariant broken!";
      }
      continue;
    }

    if (graph_->getNode(node).value().get().hasSiblings()) {
      continue;
    }

    to_delete.push_back(node);
  }

  VLOG(5) << "Deleting: " << displayNodeSymbolContainer(to_delete);

  for (const auto node : to_delete) {
    clearNodeInfo(node);
  }
  // check on isolated nodes
  std::list<NodeId> isolated;
  for (const auto& id_node_pair : graph_->nodes()) {
    if (!id_node_pair.second->hasSiblings()) {
      isolated.push_back(id_node_pair.first);
    }
  }
}

void GraphExtractor::extract(const GvdLayer& layer) {
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

  // add local connectivity
  if (config_.add_freespace_edges) {
    findFreespaceEdges();
  }

  // heuristically add global connectivity
  if (config_.add_component_connection_edges) {
    pseudo_edge_window_.push_back(visited_nodes_);
    if (pseudo_edge_window_.size() > config_.connected_component_window) {
      pseudo_edge_window_.pop_front();
    }

    findComponentConnections(layer);
  }

  if (config_.remove_isolated_nodes) {
    filterIsolatedNodes();
  }

  visited_nodes_.clear();

  // TODO(nathan) remove when we're more certain
  size_t num_valid = 0;
  size_t num_total = 0;
  for (const auto& id_edge_pair : graph_->edges()) {
    num_total++;
    if (id_edge_pair.second.info->weighted) {
      num_valid++;
    }
  }

  CHECK_EQ(num_valid, num_total) << num_valid << " / " << num_total;
}

void GraphExtractor::assignMeshVertices(const GvdLayer& gvd,
                                        const GvdParentMap& parents,
                                        const GvdVertexMap& parent_vertices) {
  for (const auto& id_index_pair : node_id_root_map_) {
    const NodeId node_id = id_index_pair.first;
    const GlobalIndex& node_index = id_index_pair.second;

    auto& attrs = graph_->getNode(node_id)->get().attributes<PlaceNodeAttributes>();
    attrs.voxblox_mesh_connections.clear();

    const GvdVoxel* voxel = CHECK_NOTNULL(gvd.getVoxelPtrByGlobalIndex(node_index));
    const GlobalIndex curr_parent = Eigen::Map<const GlobalIndex>(voxel->parent);
    auto iter = parent_vertices.find(curr_parent);
    if (iter != parent_vertices.end()) {
      const auto& parent_info = iter->second;
      NearestVertexInfo info;
      std::memcpy(info.block, parent_info.block, sizeof(info.block));
      std::memcpy(info.voxel_pos, parent_info.pos, sizeof(info.voxel_pos));
      info.vertex = parent_info.vertex;
      attrs.voxblox_mesh_connections.push_back(info);
    }

    CHECK(parents.count(node_index));
    for (const auto& parent : parents.at(node_index)) {
      if (!parent_vertices.count(parent)) {
        continue;
      }

      const auto& parent_info = parent_vertices.at(parent);
      NearestVertexInfo info;
      std::memcpy(info.block, parent_info.block, sizeof(info.block));
      std::memcpy(info.voxel_pos, parent_info.pos, sizeof(info.voxel_pos));
      info.vertex = parent_info.vertex;
      attrs.voxblox_mesh_connections.push_back(info);
    }
  }
}

}  // namespace topology
}  // namespace hydra
