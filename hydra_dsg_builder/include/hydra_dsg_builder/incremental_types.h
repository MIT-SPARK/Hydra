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
#pragma once
#include <gtsam/geometry/Pose3.h>
#include <hydra_utils/dsg_types.h>
#include <kimera_pgmo/utils/CommonStructs.h>

#include <atomic>
#include <map>
#include <memory>
#include <mutex>

namespace hydra {

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
  uint64_t last_update_time;
  DynamicSceneGraph::Ptr graph;
  std::shared_ptr<NodeIdSet> latest_places;

  std::map<NodeId, size_t> agent_key_map;
  NodeIdSet archived_places;

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
}  // namespace hydra
