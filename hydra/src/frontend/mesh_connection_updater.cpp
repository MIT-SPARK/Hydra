#include "hydra/frontend/mesh_connection_updater.h"

#include "hydra/frontend/mesh_segmenter.h"
#include "hydra/utils/mesh_utilities.h"

namespace hydra {

void MeshConnectionUpdater::updateObjects(const MeshSegmenter& segmenter,
                                          const MeshUpdateInfo& info,
                                          spark_dsg::SceneGraph& graph) {
  const auto& offsets = info.offsets;
  for (const auto& index : info.archived_blocks) {
    active_blocks_.erase(index);
  }
  for (auto& [key, entry] : mappings_) {
    for (auto& index : entry.vertices) {
      if (index) {
        index = offsets.remapGlobalVertex(*index);
      }
    }
  }
  for (const auto& [block, mapping] : info.blocks) {
    mappings_[block] = mapping;
    active_blocks_[mapping.block->index] = block;
  }

  for (auto& [id, connections] : objects_) {
    offsets.remapVertexIndices(connections.archived);
    connections.archived.sort();
    connections.archived.unique();
  }
  for (const auto& [from, into] : segmenter.merges()) {
    auto it = objects_.find(from);
    if (it != objects_.end()) {
      auto& target = objects_[into].archived;
      target.splice(target.end(), it->second.archived);
      objects_.erase(it);
    }
    graph.removeNode(from);
  }

  const auto resolve = [&](const std::vector<ObjectMeshVertex>& vertices,
                           std::list<size_t>& output) {
    for (const auto& source : vertices) {
      const auto block = mappings_.find(source.block);
      if (block == mappings_.end()) {
        throw std::logic_error("Missing object source block mapping");
      }
      const auto index = block->second.vertices.at(source.vertex);
      if (index) {
        output.push_back(*index);
      }
    }
  };

  for (const auto& [id, object] : segmenter.objects()) {
    if (!object.is_active && !object.has_archived) {
      graph.removeNode(id);
      objects_.erase(id);
      continue;
    }
    auto& connections = objects_[id];
    connections.has_active_support = object.is_active;
    resolve(object.archived_vertices, connections.archived);
    connections.archived.sort();
    connections.archived.unique();

    if (!graph.hasNode(id)) {
      auto attrs = std::make_unique<spark_dsg::ObjectNodeAttributes>();
      attrs->semantic_label = object.label;
      attrs->bounding_box.type = segmenter.config.bounding_box_type;
      graph.emplaceNode(segmenter.config.layer_id, id, std::move(attrs));
    }
    auto& attrs = graph.getNode(id).attributes<spark_dsg::ObjectNodeAttributes>();
    attrs.last_update_time_ns = object.timestamp_ns;
    attrs.mesh_connections = connections.archived;
    resolve(object.vertices, attrs.mesh_connections);
    attrs.mesh_connections.sort();
    attrs.mesh_connections.unique();
    attrs.is_active = object.is_active ||
                      (!attrs.mesh_connections.empty() &&
                       attrs.mesh_connections.back() >= offsets.archived_vertices);
    updateObjectGeometry(
        *graph.mesh(), attrs, nullptr, segmenter.config.bounding_box_type);
  }

  // An object can leave the volumetric window before shared compressed vertices
  // archive. Keep its graph node active and remap those connections until they
  // become permanent, even though it no longer participates in association.
  for (auto it = objects_.begin(); it != objects_.end();) {
    auto& [id, connections] = *it;
    if (connections.has_active_support) {
      ++it;
      continue;
    }
    const bool pending = !connections.archived.empty() &&
                         connections.archived.back() >= offsets.archived_vertices;
    if (graph.hasNode(id)) {
      auto& attrs = graph.getNode(id).attributes<spark_dsg::ObjectNodeAttributes>();
      attrs.mesh_connections = connections.archived;
      attrs.is_active = pending;
      if (!attrs.mesh_connections.empty()) {
        updateObjectGeometry(
            *graph.mesh(), attrs, nullptr, segmenter.config.bounding_box_type);
      }
    }
    if (!pending) {
      it = objects_.erase(it);
    } else {
      ++it;
    }
  }

  std::unordered_set<const MeshBlock*> active;
  for (const auto& [index, block] : active_blocks_) {
    active.insert(block);
  }
  for (auto it = mappings_.begin(); it != mappings_.end();) {
    if (!active.count(it->first)) {
      it = mappings_.erase(it);
    } else {
      ++it;
    }
  }
}
}  // namespace hydra
