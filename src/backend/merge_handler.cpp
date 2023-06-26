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
#include "hydra/backend/merge_handler.h"

#include <glog/logging.h>

namespace hydra {

using PlaceAttrs = PlaceNodeAttributes;
using ObjectAttrs = ObjectNodeAttributes;

// TODO(nathan) this seems useful for spark_dsg
void fillConnections(const SceneGraphNode& node, std::vector<NodeId>& connections) {
  connections.clear();
  connections.insert(connections.end(), node.siblings().begin(), node.siblings().end());
  connections.insert(connections.end(), node.children().begin(), node.children().end());
  // TODO(nathan) this changes when we can have multiple parents
  if (node.hasParent()) {
    connections.push_back(*node.getParent());
  }
}

MergeHandler::MergeHandler(const std::shared_ptr<ObjectUpdater>& object_updater,
                           const std::shared_ptr<PlaceUpdater> place_updater,
                           bool undo_allowed)
    : object_updater_(object_updater),
      place_updater_(place_updater),
      undo_allowed_(undo_allowed) {
  if (undo_allowed_ && (!object_updater_ || !place_updater_)) {
    LOG(ERROR) << "[Hydra Backend] Invalid node updaters when updating node caches";
    throw std::runtime_error("place and object update functors required!");
    return;
  }

  if (undo_allowed_) {
    LOG(INFO) << "[Hydra Backend] Undoing merges enabled!";
  }
}

void MergeHandler::updateFromUnmergedGraph(const DynamicSceneGraph& graph) {
  clearRemovedNodes(graph);
  if (!undo_allowed_) {
    return;
  }

  for (const auto& id_entry_pair : merged_nodes_cache_) {
    auto& entry = *id_entry_pair.second;
    if (!entry.is_active) {
      continue;
    }

    // we want to make sure we insert the most up-to-date version
    // of the previously merged node so we copy over current attributes
    // and connections
    const auto node_id = id_entry_pair.first;
    const SceneGraphNode& node = graph.getNode(node_id).value();
    updateNodeEntry(node, entry);
    // make sure we grab the latest backend version of the attributes
    // before finally caching the attributes of the node
    entry.need_backend_update = true;
  }
}

void MergeHandler::clearRemovedNodes(const DynamicSceneGraph& graph) {
  auto iter = merged_nodes_.begin();
  while (iter != merged_nodes_.end()) {
    const auto child = iter->first;
    if (graph.hasNode(child)) {
      ++iter;
      continue;
    }

    if (undo_allowed_) {
      merged_nodes_cache_.erase(child);
    }

    const auto parent = iter->second;
    if (!merged_nodes_parents_.count(parent)) {
      continue;
    }
    CHECK(merged_nodes_parents_.count(parent)) << NodeSymbol(parent).getLabel();
    auto& parent_set = merged_nodes_parents_.find(parent)->second;

    parent_set.erase(child);
    if (parent_set.empty()) {
      merged_nodes_parents_.erase(parent);
      if (undo_allowed_) {
        parent_nodes_cache_.erase(parent);
      }
    }

    iter = merged_nodes_.erase(iter);
  }
}

size_t MergeHandler::checkAndUndo(DynamicSceneGraph& graph, const UpdateInfo& info) {
  updateInfoCaches(graph, info);

  const size_t num_before = merged_nodes_.size();

  std::map<NodeId, NodeId> to_undo;
  for (const auto& id_entry_pair : merged_nodes_cache_) {
    auto& from_entry = *id_entry_pair.second;
    if (!from_entry.is_active && !info.loop_closure_detected) {
      continue;
    }

    const NodeId from_node = id_entry_pair.first;
    const NodeId to_node = merged_nodes_.at(from_node);
    const auto& to_entry = *parent_nodes_cache_.at(to_node);
    if (!shouldUndo(from_entry, to_entry)) {
      continue;
    }

    to_undo[from_node] = to_node;
  }

  // undoMerge invalidates merged_nodes_cache_
  for (const auto& undo_pair : to_undo) {
    auto parent = merged_nodes_.find(undo_pair.first);
    if (parent == merged_nodes_.end()) {
      // we may have been unmerged by another child
      continue;
    }

    if (parent->second != undo_pair.second) {
      // we've been repaired with a different node, skip
      continue;
    }

    undoMerge(graph, info, undo_pair.first, undo_pair.second);
  }

  const size_t num_after = merged_nodes_.size();
  if (num_after > num_before) {
    LOG(WARNING) << "More merges created from undoing merges: " << num_before << " < "
                 << num_after;
    return 0;
  }

  return num_after - num_before;
}

void MergeHandler::updateMerges(const std::map<NodeId, NodeId>& new_merges,
                                DynamicSceneGraph& graph) {
  if (new_merges.empty()) {
    return;
  }

  VLOG(1) << "[Hydra Backend] processing " << new_merges.size()
          << " proposed node merges";

  for (auto miter = new_merges.rbegin(); miter != new_merges.rend(); ++miter) {
    NodeId from, to;
    std::tie(from, to) = *miter;
    VLOG(5) << "[Hydra Backend] Considering merge: " << NodeSymbol(from).getLabel()
            << " -> " << NodeSymbol(to).getLabel();

    auto parent = merged_nodes_.find(to);
    if (parent != merged_nodes_.end()) {
      if (parent->second == from) {
        // opposed merge: both to -> from and from -> to were in new_merges
        VLOG(4) << "[Hydra Backend] Dropping proposed merge "
                << NodeSymbol(from).getLabel() << " -> " << NodeSymbol(to).getLabel();
        continue;
      }

      to = parent->second;
    }

    auto iter = merged_nodes_parents_.find(to);
    if (iter == merged_nodes_parents_.end()) {
      iter = merged_nodes_parents_.emplace(to, std::set<NodeId>()).first;
    }

    iter->second.insert(from);
    merged_nodes_[from] = to;

    if (undo_allowed_) {
      addNodeToCache(graph.getNode(from).value(), merged_nodes_cache_, false);
      addNodeToCache(graph.getNode(to).value(), parent_nodes_cache_, true);
    }

    VLOG(3) << "[Hydra Backend] Merging " << NodeSymbol(from).getLabel() << " -> "
            << NodeSymbol(to).getLabel();
    graph.mergeNodes(from, to);

    auto old_iter = merged_nodes_parents_.find(from);
    if (old_iter == merged_nodes_parents_.end()) {
      continue;
    }

    for (const auto child : old_iter->second) {
      merged_nodes_[child] = to;
      iter->second.insert(child);
    }

    merged_nodes_parents_.erase(old_iter);
  }

  // consider updating here?
}

void MergeHandler::reset() {
  merged_nodes_cache_.clear();
  parent_nodes_cache_.clear();
  merged_nodes_.clear();
  merged_nodes_parents_.clear();
}

void MergeHandler::updateNodeEntry(const SceneGraphNode& node, NodeInfo& entry) {
  entry.layer = node.layer;
  entry.attrs = node.attributes().clone();
  entry.is_active = node.attributes().is_active;
  fillConnections(node, entry.neighbors);
}

void MergeHandler::addNodeToCache(const SceneGraphNode& node,
                                  std::map<NodeId, NodeInfo::Ptr>& cache,
                                  bool check_if_present) {
  if (node.layer != DsgLayers::OBJECTS && node.layer != DsgLayers::PLACES) {
    return;
  }

  if (check_if_present && cache.count(node.id)) {
    return;
  }

  // TODO(nathan) dynamic layers might be a problem here
  auto ret = cache.emplace(node.id, std::make_unique<NodeInfo>());
  updateNodeEntry(node, *ret.first->second);
}

void MergeHandler::updateCacheEntryFromInfo(const MeshVertices::Ptr& mesh,
                                            const UpdateInfo& info,
                                            NodeId node,
                                            NodeInfo& entry) {
  if (entry.layer == DsgLayers::PLACES) {
    if (!info.places_values->exists(node)) {
      VLOG(5) << "[Hydra Backend] missing merged place " << NodeSymbol(node).getLabel()
              << " from places factors.";
      return;
    }

    auto& place_attrs = dynamic_cast<PlaceAttrs&>(*entry.attrs);
    place_updater_->updatePlace(*info.places_values, node, place_attrs);
  } else {
    auto& object_attrs = dynamic_cast<ObjectAttrs&>(*entry.attrs);
    object_updater_->updateObject(mesh, node, object_attrs);
  }
}

void MergeHandler::updateInfoCaches(const DynamicSceneGraph& graph,
                                    const UpdateInfo& info) {
  const auto mesh = graph.getMeshVertices();
  if (!mesh || !info.places_values) {
    LOG(ERROR) << "[Hydra Backend] Invalid mesh or places values";
    return;
  }

  for (const auto& id_entry_pair : merged_nodes_cache_) {
    auto& entry = *id_entry_pair.second;
    if (!entry.is_active && !info.loop_closure_detected && !entry.need_backend_update) {
      continue;
    }

    updateCacheEntryFromInfo(mesh, info, id_entry_pair.first, entry);
    entry.need_backend_update = false;
  }

  if (!info.loop_closure_detected) {
    return;
  }

  for (const auto& id_entry_pair : parent_nodes_cache_) {
    auto& entry = *id_entry_pair.second;
    updateCacheEntryFromInfo(mesh, info, id_entry_pair.first, entry);
  }
}

void addEdgesFromEntry(DynamicSceneGraph& graph, NodeId node_id, NodeInfo& node_entry) {
  for (const auto& target : node_entry.neighbors) {
    graph.insertEdge(node_id, target);
  }
}

void MergeHandler::undoMerge(DynamicSceneGraph& graph,
                             const UpdateInfo& info,
                             NodeId from_node,
                             NodeId to_node) {
  VLOG(1) << "[Hydra Backend] undoing merge: " << NodeSymbol(from_node).getLabel()
          << " -> " << NodeSymbol(to_node).getLabel();
  // easier to remove than doing book-keeping on the to_node edges
  graph.removeNode(to_node);

  // n.b.: entry attributes pointers and entries themselves may be invalidated
  // from this point forward. operation order matters here!
  std::map<NodeId, NodeInfo::Ptr> entries;

  // erase from_node entry
  auto from_iter = merged_nodes_cache_.find(from_node);
  entries.emplace(from_node, std::move(from_iter->second));
  merged_nodes_cache_.erase(from_iter);

  // erase to_node entry
  auto to_iter = parent_nodes_cache_.find(to_node);
  entries.emplace(to_node, std::move(to_iter->second));
  parent_nodes_cache_.erase(to_iter);

  auto& children = merged_nodes_parents_.at(to_node);
  children.erase(from_node);

  // force updates for all archived nodes and invalidate all entries for the children
  for (const auto& child : children) {
    auto iter = merged_nodes_cache_.find(child);
    auto& entry = iter->second;

    if (!info.loop_closure_detected && !entry->is_active) {
      LOG(WARNING) << "Updating archived: " << NodeSymbol(child).getLabel();
      updateCacheEntryFromInfo(graph.getMeshVertices(), info, child, *entry);
    }

    entries.emplace(child, std::move(entry));
    merged_nodes_cache_.erase(iter);
    merged_nodes_.erase(child);
  }

  merged_nodes_.erase(from_node);
  merged_nodes_parents_.erase(to_node);

  // check for sub-merges
  std::map<NodeId, NodeId> proposed_merges;
  for (const auto& from_pair : entries) {
    for (const auto& to_pair : entries) {
      if (from_pair.first == to_pair.first) {
        continue;
      }

      if (!shouldUndo(*from_pair.second, *to_pair.second)) {
        proposed_merges[from_pair.first] = to_pair.first;
      }
    }
  }

  // all entry attributes are invalidated
  for (auto& id_entry_pair : entries) {
    const auto node_id = id_entry_pair.first;
    auto& entry = *id_entry_pair.second;
    graph.emplaceNode(entry.layer, node_id, std::move(entry.attrs));
  }

  for (auto& id_entry_pair : entries) {
    const auto node_id = id_entry_pair.first;
    auto& entry = *id_entry_pair.second;
    addEdgesFromEntry(graph, node_id, entry);
  }

  // pass submerges on
  updateMerges(proposed_merges, graph);
}

bool MergeHandler::shouldUndo(const NodeInfo& from_info,
                              const NodeInfo& to_info) const {
  if (from_info.layer == DsgLayers::PLACES) {
    return !place_updater_->shouldMerge(
        dynamic_cast<const PlaceAttrs&>(*from_info.attrs),
        dynamic_cast<const PlaceAttrs&>(*to_info.attrs));
  } else {
    return !object_updater_->shouldMerge(
        dynamic_cast<const ObjectAttrs&>(*from_info.attrs),
        dynamic_cast<const ObjectAttrs&>(*to_info.attrs));
  }
}

}  // namespace hydra
