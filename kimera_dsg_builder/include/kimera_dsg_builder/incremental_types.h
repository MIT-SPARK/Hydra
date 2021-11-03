#pragma once
#include <kimera_dsg/dynamic_scene_graph.h>
#include <kimera_pgmo/utils/CommonStructs.h>

#include <atomic>
#include <map>
#include <memory>
#include <mutex>

namespace kimera {
namespace incremental {

typedef std::unordered_set<NodeId> NodeIdSet;

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
    block_mesh_mapping.reset(new kimera_pgmo::VoxbloxIndexMapping);
    latest_places.reset(new NodeIdSet);
  }

  std::mutex mutex;
  DynamicSceneGraph::Ptr graph;
  std::shared_ptr<kimera_pgmo::VoxbloxIndexMapping> block_mesh_mapping;
  std::shared_ptr<NodeIdSet> latest_places;
  std::atomic<bool> updated;
};

struct DsgBackendStatus {
  size_t total_loop_closures_;
  size_t new_loop_closures_;
  size_t total_factors_;
  size_t total_values_;
  size_t new_factors_;
  size_t new_graph_factors_;
  size_t trajectory_len_;

  void reset() {
    total_loop_closures_ = 0;
    new_loop_closures_ = 0;
    total_factors_ = 0;
    total_values_ = 0;
    new_factors_ = 0;
    new_graph_factors_ = 0;
    trajectory_len_ = 0;
  }
};

}  // namespace incremental
}  // namespace kimera
