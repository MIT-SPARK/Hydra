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
#include "hydra/places/graph_extractor_interface.h"

#include "hydra/places/graph_extractor_utilities.h"
#include "hydra/places/gvd_parent_tracker.h"

namespace hydra::places {

GraphExtractorInterface::GraphExtractorInterface(const GraphExtractorConfig& config)
    : config_(config),
      next_node_id_('p', 0),
      gvd_(new GvdGraph()),
      graph_(new IsolatedSceneGraphLayer(DsgLayers::PLACES)) {}

GraphExtractorInterface::~GraphExtractorInterface() = default;

void GraphExtractorInterface::pushGvdIndex(const GlobalIndex& index) {
  modified_voxel_queue_.push(index);
}

NearestVertexInfo convertInfo(const GvdVertexInfo& parent_info) {
  NearestVertexInfo info;
  info.voxel_pos[0] = parent_info.pos[0];
  info.voxel_pos[1] = parent_info.pos[1];
  info.voxel_pos[2] = parent_info.pos[2];
  return info;
}

void GraphExtractorInterface::fillParentInfo(const GvdLayer& gvd,
                                             const GvdParentTracker& tracker) {
  for (auto&& [node_id, node_index] : node_index_map_) {
    auto& attrs = graph_->getNode(node_id).attributes<PlaceNodeAttributes>();
    attrs.voxblox_mesh_connections.clear();

    const auto* voxel = gvd.getVoxelPtr(node_index);
    if (!voxel) {
      // the compression-based extractor can have nodes pointing to archived voxels
      continue;
    }

    CHECK(tracker.parents.count(node_index))
        << "bad gvd voxel: " << *voxel << " @ " << node_index.transpose();

    // save primary parent first
    const GlobalIndex curr_parent = voxel->parent;
    auto iter = tracker.parent_vertices.find(curr_parent);
    if (iter != tracker.parent_vertices.end()) {
      attrs.voxblox_mesh_connections.push_back(convertInfo(iter->second));
    }

    // save all other basis points
    for (const auto& parent : tracker.parents.at(node_index)) {
      if (!tracker.parent_vertices.count(parent)) {
        continue;
      }

      const auto& parent_info = tracker.parent_vertices.at(parent);
      attrs.voxblox_mesh_connections.push_back(convertInfo(parent_info));
    }
  }
}

const SceneGraphLayer& GraphExtractorInterface::getGraph() const { return *graph_; }

const GvdGraph& GraphExtractorInterface::getGvdGraph() const { return *gvd_; }

std::unordered_set<NodeId> GraphExtractorInterface::getActiveNodes() const {
  std::unordered_set<NodeId> nodes;
  for (const auto& id_index_pair : node_index_map_) {
    nodes.insert(id_index_pair.first);
  }
  return nodes;
}

const std::unordered_set<NodeId>& GraphExtractorInterface::getDeletedNodes() const {
  return deleted_nodes_;
}

const std::vector<NodeId>& GraphExtractorInterface::getDeletedEdges() const {
  return deleted_edges_;
}

void GraphExtractorInterface::clearDeleted() {
  deleted_nodes_.clear();
  deleted_edges_.clear();
}

NodeId GraphExtractorInterface::addPlaceToGraph(const GvdLayer& layer,
                                                const GvdVoxel& voxel,
                                                const GlobalIndex& index) {
  const auto distance = voxel.distance;
  const auto basis = voxel.num_extra_basis + 1;

  PlaceNodeAttributes::Ptr attrs(new PlaceNodeAttributes(distance, basis));
  attrs->position = layer.getVoxelPosition(index).cast<double>();
  graph_->emplaceNode(next_node_id_, std::move(attrs));

  NodeId new_node = next_node_id_;
  next_node_id_++;
  return new_node;
}

EdgeAttributes::Ptr GraphExtractorInterface::makeEdgeInfo(const GvdLayer& layer,
                                                          NodeId source_id,
                                                          NodeId target_id) const {
  const double source_dist =
      graph_->getNode(source_id).attributes<PlaceNodeAttributes>().distance;
  const double target_dist =
      graph_->getNode(target_id).attributes<PlaceNodeAttributes>().distance;

  double min_weight = std::min(source_dist, target_dist);

  const GlobalIndex source = node_index_map_.at(source_id);
  const GlobalIndex target = node_index_map_.at(target_id);
  auto path = makeBresenhamLine(source, target);
  if (path.empty()) {
    // edge is smaller than voxel size, so we just take the min distance between two
    // voxels
    return std::make_unique<EdgeAttributes>(min_weight);
  }

  for (const auto& index : path) {
    const GvdVoxel* voxel = layer.getVoxelPtr(index);
    if (!voxel) {
      continue;
    }

    if (voxel->distance < min_weight) {
      min_weight = voxel->distance;
    }
  }

  return std::make_unique<EdgeAttributes>(min_weight);
}

GlobalIndex GraphExtractorInterface::popFromModifiedQueue() {
  GlobalIndex index = modified_voxel_queue_.front();
  modified_voxel_queue_.pop();
  return index;
}

void GraphExtractorInterface::removeGraphNode(NodeId node) {
  graph_->removeNode(node);
  deleted_nodes_.insert(node);
}

void GraphExtractorInterface::updateGraphEdge(NodeId source,
                                              NodeId target,
                                              EdgeAttributes::Ptr&& attrs,
                                              bool is_heursitic) {
  auto prev_edge = graph_->findEdge(source, target);
  if (prev_edge) {
    *prev_edge->info = *attrs;
    if (!is_heursitic) {
      // make sure any non-heurstic source of an edge takes precedence
      heuristic_edges_.erase(EdgeKey(source, target));
    }
  } else {
    graph_->insertEdge(source, target, std::move(attrs));
  }
}

void GraphExtractorInterface::addGraphNode(NodeId node,
                                           PlaceNodeAttributes::Ptr&& attrs) {
  graph_->emplaceNode(
      node, attrs ? std::move(attrs) : std::make_unique<PlaceNodeAttributes>());
}

void GraphExtractorInterface::removeGraphEdge(NodeId source, NodeId target) {
  graph_->removeEdge(source, target);
  deleted_edges_.push_back(source);
  deleted_edges_.push_back(target);
}

void GraphExtractorInterface::mergeGraphNodes(NodeId from, NodeId to) {
  graph_->mergeNodes(from, to);
  deleted_nodes_.insert(from);
}

void GraphExtractorInterface::updateHeuristicEdges(const GvdLayer& gvd) {
  const auto active_nodes = getActiveNodes();

  auto iter = heuristic_edges_.begin();
  while (iter != heuristic_edges_.end()) {
    const auto source = iter->first.k1;
    const auto target = iter->first.k2;
    if (!active_nodes.count(source) || !active_nodes.count(target)) {
      iter = heuristic_edges_.erase(iter);
      continue;
    }

    EdgeAttributes::Ptr info;
    if (iter->second) {
      info = getOverlapEdgeInfo(
          *graph_, source, target, config_.overlap_edges.min_clearance_m);
    } else {
      info = getFreespaceEdgeInfo(*graph_,
                                  gvd,
                                  node_index_map_,
                                  source,
                                  target,
                                  config_.freespace_edges.min_clearance_m);
    }

    if (!info) {
      removeGraphEdge(source, target);
      iter = heuristic_edges_.erase(iter);
      continue;
    }

    updateGraphEdge(source, target, std::move(info), true);
    ++iter;
  }

  // add local connectivity
  if (config_.add_overlap_edges) {
    EdgeInfoMap proposed_edges;
    findOverlapEdges(config_.overlap_edges, *graph_, active_nodes, proposed_edges);
    for (auto& key_edge_pair : proposed_edges) {
      updateGraphEdge(key_edge_pair.first.k1,
                      key_edge_pair.first.k2,
                      std::move(key_edge_pair.second),
                      true);
      heuristic_edges_.emplace(key_edge_pair.first, true);
    }
  }

  if (config_.add_freespace_edges) {
    EdgeInfoMap proposed_edges;
    findFreespaceEdges(config_.freespace_edges,
                       *graph_,
                       gvd,
                       active_nodes,
                       node_index_map_,
                       proposed_edges);

    for (auto& key_edge_pair : proposed_edges) {
      updateGraphEdge(key_edge_pair.first.k1,
                      key_edge_pair.first.k2,
                      std::move(key_edge_pair.second),
                      true);
      heuristic_edges_.emplace(key_edge_pair.first, false);
    }
  }
}

}  // namespace hydra::places
