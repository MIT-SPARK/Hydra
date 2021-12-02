#pragma once
#include <kimera_dsg/dynamic_scene_graph.h>
#include <kimera_pgmo/utils/CommonStructs.h>
#include <gtsam/geometry/Pose3.h>

#include <atomic>
#include <map>
#include <memory>
#include <mutex>

namespace kimera {

namespace lcd {

struct DsgRegistrationSolution {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool valid = false;
  NodeId from_node;
  NodeId to_node;
  gtsam::Pose3 to_T_from;
  int64_t level;
};

}  // namespace lcd

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
    latest_places.reset(new NodeIdSet);
  }

  std::mutex mutex;
  std::atomic<bool> updated;
  DynamicSceneGraph::Ptr graph;
  std::shared_ptr<NodeIdSet> latest_places;
  std::mutex lcd_mutex;
  std::queue<lcd::DsgRegistrationSolution> loop_closures;
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
