#pragma once
#include <kimera_dsg/dynamic_scene_graph.h>

#include <atomic>
#include <map>
#include <memory>
#include <mutex>

namespace kimera {
namespace incremental {

struct SharedDsgInfo {
  using Ptr = std::shared_ptr<SharedDsgInfo>;

  SharedDsgInfo(const std::map<LayerId, char>& layer_id_map, LayerId mesh_layer_id)
      : updated(false) {
    DynamicSceneGraph::LayerIds layer_ids;
    for (const auto& id_key_pair : layer_id_map) {
      CHECK(id_key_pair.first != mesh_layer_id)
          << "Found duplicate layer id " << id_key_pair.first
          << " with mesh: " << mesh_layer_id;

      layer_ids.push_back(id_key_pair.first);
    }

    graph.reset(new DynamicSceneGraph(layer_ids, mesh_layer_id));
  }

  std::mutex mutex;
  DynamicSceneGraph::Ptr graph;
  std::atomic<bool> updated;
};

}  // namespace incremental
}  // namespace kimera
