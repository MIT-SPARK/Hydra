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
#include "hydra/backend/update_objects_functor.h"

#include <glog/logging.h>

#include "hydra/backend/backend_utilities.h"
#include "hydra/utils/mesh_utilities.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using timing::ScopedTimer;
using SemanticLabel = SemanticNodeAttributes::Label;
using MergeId = std::optional<NodeId>;

NodeAttributes::Ptr mergeObjectAttributes(const DynamicSceneGraph& graph,
                                          const std::vector<NodeId>& nodes) {
  if (nodes.empty()) {
    return nullptr;
  }

  auto iter = nodes.begin();
  auto attrs_ptr = graph.getNode(*iter).attributes().clone();
  auto& new_attrs =
      *CHECK_NOTNULL(dynamic_cast<ObjectNodeAttributes*>(attrs_ptr.get()));
  ++iter;
  while (iter != nodes.end()) {
    const auto& from_attrs = graph.getNode(*iter).attributes<ObjectNodeAttributes>();
    utils::mergeIndices(from_attrs.mesh_connections, new_attrs.mesh_connections);
    ++iter;
  }

  if (new_attrs.mesh_connections.empty()) {
    VLOG(2) << "Merge is empty: " << displayNodeSymbolContainer(nodes);
    return attrs_ptr;
  }

  auto mesh = graph.mesh();
  if (!updateObjectGeometry(*mesh, new_attrs)) {
    VLOG(2) << "Merge geometry invalid: " << displayNodeSymbolContainer(nodes);
  }

  return attrs_ptr;
}

UpdateObjectsFunctor::UpdateObjectsFunctor() {}

UpdateFunctor::Hooks UpdateObjectsFunctor::hooks() const {
  auto my_hooks = UpdateFunctor::hooks();
  if (allow_connection_merging) {
    my_hooks.merge = &mergeObjectAttributes;
  }
  return my_hooks;
}

MergeList UpdateObjectsFunctor::call(const DynamicSceneGraph& unmerged,
                                     SharedDsgInfo& dsg,
                                     const UpdateInfo::ConstPtr& info) const {
  ScopedTimer spin_timer("backend/update_objects", info->timestamp_ns);
  if (!unmerged.hasLayer(DsgLayers::OBJECTS)) {
    VLOG(5) << "Skipping object update due to missing layer";
    return {};
  }

  // we want to use the optimized mesh (unmerged doesn't have a mesh)
  const auto mesh = dsg.graph->mesh();
  const auto new_loopclosure = info->loop_closure_detected;
  // we want to use the unmerged graph for most things
  const auto& objects = unmerged.getLayer(DsgLayers::OBJECTS);
  makeSemanticNodeFinders(objects, node_finders);
  // we want to iterate over the unmerged graph
  LayerView view = new_loopclosure ? LayerView(objects) : active_tracker.view(objects);

  // apply updates to every attribute that may have changed since the last call
  size_t num_changed = 0;
  std::list<NodeId> seen_nodes;
  for (const auto& node : view) {
    ++num_changed;
    auto& attrs = node.attributes<ObjectNodeAttributes>();
    VLOG(10) << "Processing object " << NodeSymbol(node.id).getLabel()
             << " with attributes:\n"
             << attrs;
    if (attrs.mesh_connections.empty()) {
      VLOG(2) << "Found empty object node " << NodeSymbol(node.id).getLabel();
      continue;
    }

    if (!updateObjectGeometry(*mesh, attrs)) {
      VLOG(2) << "Invalid centroid for object " << NodeSymbol(node.id).getLabel();
    }

    seen_nodes.push_back(node.id);
    // TODO(nathan) this is sloppy and needs to be cleaned up
    dsg.graph->setNodeAttributes(node.id, attrs.clone());
  }

  active_tracker.clear();
  VLOG(2) << "[Hydra Backend] Object update: " << num_changed << " node(s)";

  auto iter = info->given_merges.find(DsgLayers::OBJECTS);
  if (iter != info->given_merges.end()) {
    return iter->second;
  }

  if (!info->allow_node_merging) {
    return {};
  }

  MergeList proposals;
  for (const auto& node_id : seen_nodes) {
    const auto& attrs = unmerged.getNode(node_id).attributes<ObjectNodeAttributes>();
    const auto proposed = proposeMerge(objects, attrs);
    if (proposed) {
      proposals.push_back({node_id, *proposed});
    }
  }

  return proposals;
}

MergeId UpdateObjectsFunctor::proposeMerge(const SceneGraphLayer& layer,
                                           const ObjectNodeAttributes& attrs) const {
  // TODO(nathan) push to node finders
  const auto iter = node_finders.find(attrs.semantic_label);
  if (iter == node_finders.end()) {
    return std::nullopt;
  }

  // we skip the first entry if the node attributes aren't active (to avoid returning
  // the same node)
  std::list<NodeId> candidates;
  (*iter).second->find(attrs.position,
                       num_merges_to_consider,
                       !attrs.is_active,
                       [&candidates](NodeId object_id, size_t, double) {
                         candidates.push_back(object_id);
                       });

  for (const auto& id : candidates) {
    const auto& candiate = layer.getNode(id).attributes<ObjectNodeAttributes>();
    if (attrs.bounding_box.contains(candiate.position) ||
        candiate.bounding_box.contains(attrs.position)) {
      return id;
    }
  }

  return std::nullopt;
}

}  // namespace hydra
