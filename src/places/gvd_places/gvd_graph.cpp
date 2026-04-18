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
#include "hydra/places/gvd_places/gvd_graph.h"

#include <glog/logging.h>
#include <glog/stl_logging.h>

namespace hydra::places {

using spark_dsg::EdgeKey;
using Node = GvdGraph::Node;
using Nodes = GvdGraph::Nodes;
using CompressedNode = GvdGraph::CompressedNode;
using CompressedNodes = GvdGraph::CompressedNodes;

CompressedNode::CompressedNode(uint64_t id) : node_id(id) {}

std::vector<uint64_t> CompressedNode::refs() const {
  std::vector<uint64_t> all_refs(active_refs.begin(), active_refs.end());
  all_refs.insert(all_refs.end(), archived_refs.begin(), archived_refs.end());
  return all_refs;
}

bool CompressedNode::archived() const {
  return active_refs.empty() && !archived_refs.empty();
}

GvdGraph::GvdGraph(float voxel_resolution_m, float compression_resolution_m)
    : voxel_grid(voxel_resolution_m),
      compression_grid(compression_resolution_m),
      neighbor_search(26),
      next_uncompressed_id_(0),
      next_compressed_id_(0) {}

void GvdGraph::add(const GlobalIndex& vindex, double distance, uint8_t basis) {
  auto node = add_uncompressed(vindex, distance, basis);
  if (!node) {
    return;
  }

  for (const auto& n_idx : neighbor_search.neighborIndices(vindex)) {
    auto neighbor = uncompressed_by_index(n_idx);
    if (!neighbor) {
      continue;
    }

    neighbor->siblings.insert(node->id);
    node->siblings.insert(neighbor->id);
  }

  // always insert an active ref and add to updated
  const auto cluster = assign_to_cluster(*node);
  updated_.insert(cluster);

  // construct edges by checking to see if any neighbors map to a different cluster
  auto& cell_clusters = compressed_index_map_.at(compressed_id_map_.at(cluster));
  for (const auto neighbor : node->siblings) {
    const auto neighbor_cluster = cluster_for_gvd(neighbor);
    if (!neighbor_cluster || neighbor_cluster == cluster) {
      continue;
    }

    EdgeKey key(cluster, *neighbor_cluster);
    if (cell_clusters.count(*neighbor_cluster)) {
      // voxel connected two neighboring clusters for the same index
      merge_clusters(cluster, *neighbor_cluster);
      cell_clusters.erase(*neighbor_cluster);
      continue;
    }

    auto iter = compressed_edge_support_.find(key);
    if (iter == compressed_edge_support_.end()) {
      iter = compressed_edge_support_.emplace(key, std::set<EdgeKey>{}).first;
      compressed_.at(key.k1).siblings.insert(key.k2);
      compressed_.at(key.k2).siblings.insert(key.k1);
    }

    iter->second.insert({node->id, neighbor});
  }
}

void GvdGraph::remove(const GlobalIndex& index) {
  // TODO(nathan) this needs to be able to split nodes technically
  const auto maybe_gvd_id = drop_uncompressed(index);
  if (!maybe_gvd_id) {
    return;
  }

  const auto gvd_id = *maybe_gvd_id;
  const auto compressed_id = compression_map_.at(gvd_id);
  compression_map_.erase(gvd_id);

  std::set<uint64_t> gvd_siblings;
  {  // scope limiting iter
    auto iter = uncompressed_.find(gvd_id);
    gvd_siblings = iter->second.siblings;
    uncompressed_id_queue_.push_back(gvd_id);
    uncompressed_.erase(iter);
  }

  auto iter = compressed_.find(compressed_id);
  auto& node = iter->second;
  node.active_refs.erase(gvd_id);
  for (const auto sibling : gvd_siblings) {
    uncompressed_.at(sibling).siblings.erase(gvd_id);
    const auto compressed_sibling = compression_map_.at(sibling);
    if (compressed_sibling == compressed_id) {
      continue;  // no need to update compressed edges if sibling in same cluster
    }

    const EdgeKey key{compressed_id, compressed_sibling};
    auto edge_iter = compressed_edge_support_.find(key);
    if (edge_iter == compressed_edge_support_.end()) {
      continue;
    }

    edge_iter->second.erase({gvd_id, sibling});
    if (edge_iter->second.empty()) {
      compressed_edge_support_.erase(edge_iter);
      compressed_.at(compressed_id).siblings.erase(compressed_sibling);
      compressed_.at(compressed_sibling).siblings.erase(compressed_id);
    }
  }

  if (!node.active_refs.empty() || !node.archived_refs.empty()) {
    return;
  }

  drop_compressed_id(compressed_id);
  for (const auto sibling : node.siblings) {
    compressed_.at(sibling).siblings.erase(compressed_id);
  }

  compressed_.erase(iter);
}

void GvdGraph::archive(const GlobalIndex& index) {
  const auto gvd_id = drop_uncompressed(index);
  if (!gvd_id) {
    return;
  }

  // we move entries from an active set to an archived set
  auto compressed_id = compression_map_.at(*gvd_id);
  auto& node = compressed_.at(compressed_id);
  node.active_refs.erase(*gvd_id);
  node.archived_refs.insert(*gvd_id);
}

void GvdGraph::dropCompressed(uint64_t compressed_id) {
  auto iter = compressed_.find(compressed_id);
  if (iter == compressed_.end()) {
    return;
  }

  const auto refs = iter->second.refs();
  for (const auto gvd_id : refs) {
    auto niter = uncompressed_.find(gvd_id);
    uncompressed_index_map_.erase(niter->second.info.index);
    for (const auto sibling : niter->second.siblings) {
      uncompressed_.at(sibling).siblings.erase(gvd_id);
    }

    uncompressed_.erase(niter);
    uncompressed_id_queue_.push_back(gvd_id);
  }

  for (const auto sibling : iter->second.siblings) {
    compressed_edge_support_.erase({compressed_id, sibling});
    compressed_.at(sibling).siblings.erase(compressed_id);
  }

  drop_compressed_id(compressed_id);
  compressed_.erase(iter);
}

const GvdMemberInfo* GvdGraph::get(uint64_t node) const {
  auto iter = uncompressed_.find(node);
  return iter == uncompressed_.end() ? nullptr : &iter->second.info;
}

const Nodes& GvdGraph::uncompressed() const { return uncompressed_; }

const CompressedNodes& GvdGraph::compressed() const { return compressed_; }

const GvdGraph::NodeRemapping& GvdGraph::remapping() const { return compression_map_; }

uint64_t GvdGraph::next_uncompressed_id() {
  uint64_t new_id;
  if (uncompressed_id_queue_.empty()) {
    new_id = next_uncompressed_id_;
    next_uncompressed_id_++;
  } else {
    new_id = uncompressed_id_queue_.front();
    uncompressed_id_queue_.pop_front();
  }

  return new_id;
}

uint64_t GvdGraph::next_compressed_id() {
  // TODO(nathan) make circular queue once backend works better
  uint64_t new_id = next_compressed_id_;
  ++next_compressed_id_;
  return new_id;
}

Node* GvdGraph::add_uncompressed(const GlobalIndex& index, double dist, uint8_t basis) {
  auto iter = uncompressed_index_map_.find(index);
  if (iter != uncompressed_index_map_.end()) {
    auto& node = uncompressed_.at(iter->second);
    node.info.distance = dist;
    node.info.num_basis_points = basis;
    return nullptr;
  }

  const auto pos = voxel_grid.toPoint(index);
  const auto id = next_uncompressed_id();
  uncompressed_index_map_.emplace(index, id);

  auto niter = uncompressed_.emplace(id, Node{id, {dist, basis, pos, index}, {}}).first;
  return &niter->second;
}

Node* GvdGraph::uncompressed_by_index(const GlobalIndex& index) {
  auto iter = uncompressed_index_map_.find(index);
  return iter == uncompressed_index_map_.end() ? nullptr
                                               : &uncompressed_.at(iter->second);
}

uint64_t GvdGraph::assign_to_cluster(const Node& node) {
  const auto index = compression_grid.toIndex(node.info.position);
  auto iter = compressed_index_map_.find(index);
  if (iter == compressed_index_map_.end()) {
    // add new hash to lookup if we don't have any nodes under the index
    iter = compressed_index_map_.emplace(index, std::set<uint64_t>{}).first;
  }

  std::optional<uint64_t> compressed_id;
  for (const auto& sibling : node.siblings) {
    const auto candidate = compression_map_.at(sibling);
    if (iter->second.count(candidate)) {
      compressed_id = candidate;
      break;
    }
  }

  if (!compressed_id) {
    compressed_id = next_compressed_id();
    iter->second.insert(*compressed_id);
    compressed_id_map_.emplace(*compressed_id, index);
    compressed_.emplace(*compressed_id, CompressedNode(*compressed_id));
  }

  auto& info = compressed_.at(*compressed_id);
  info.active_refs.insert(node.id);
  compression_map_[node.id] = *compressed_id;
  return *compressed_id;
}

std::optional<uint64_t> GvdGraph::cluster_for_gvd(uint64_t gvd_id) const {
  auto iter = compression_map_.find(gvd_id);
  return iter == compression_map_.end() ? std::optional<uint64_t>(iter->second)
                                        : std::nullopt;
}

void GvdGraph::merge_clusters(uint64_t target, uint64_t candidate) {
  auto iter = compressed_.find(candidate);
  auto& candidate_node = iter->second;
  auto& target_node = compressed_.at(target);
  for (const auto child : candidate_node.active_refs) {
    target_node.active_refs.insert(child);
    compression_map_[child] = target;
  }

  for (const auto child : candidate_node.archived_refs) {
    target_node.archived_refs.insert(child);
    compression_map_[child] = target;
  }

  for (auto sibling : candidate_node.siblings) {
    target_node.siblings.insert(sibling);
    compressed_.at(sibling).siblings.insert(target);

    const EdgeKey old_key{sibling, candidate};
    const auto old_iter = compressed_edge_support_.find(old_key);

    const EdgeKey new_key{sibling, target};
    auto new_iter = compressed_edge_support_.find(new_key);
    if (new_iter == compressed_edge_support_.end()) {
      new_iter = compressed_edge_support_.emplace(new_key, std::set<EdgeKey>{}).first;
    }

    new_iter->second.insert(old_iter->second.begin(), old_iter->second.end());
    compressed_edge_support_.erase(old_iter);
  }

  compressed_id_map_.erase(candidate);
  updated_.erase(candidate);
  compressed_.erase(iter);
}

std::optional<uint64_t> GvdGraph::drop_uncompressed(const GlobalIndex& idx) {
  auto iter = uncompressed_index_map_.find(idx);
  if (iter == uncompressed_index_map_.end()) {
    return std::nullopt;
  }

  const auto gvd_id = iter->second;
  uncompressed_index_map_.erase(iter);
  return gvd_id;
}

void GvdGraph::drop_compressed_id(uint64_t compressed_id) {
  const auto id_iter = compressed_id_map_.find(compressed_id);
  auto iter = compressed_index_map_.find(id_iter->second);
  iter->second.erase(compressed_id);
  if (iter->second.empty()) {
    compressed_index_map_.erase(iter);
  }

  compressed_id_map_.erase(id_iter);
}

}  // namespace hydra::places
