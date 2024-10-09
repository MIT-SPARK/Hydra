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
#include "hydra/active_window/active_window_output.h"

#include <algorithm>

namespace hydra {

using spark_dsg::NodeAttributes;

const VolumetricMap& ActiveWindowOutput::map() const {
  CHECK(map_) << "Invalid map!";
  return *map_;
}

void ActiveWindowOutput::updateFrom(ActiveWindowOutput&& msg, bool clone_map) {
  timestamp_ns = msg.timestamp_ns;
  world_t_body = msg.world_t_body;
  world_R_body = msg.world_R_body;
  sensor_data = msg.sensor_data;
  // TODO(nathan) this leads to incorrect behavior if we add in one update but then archive in another
  archived_mesh_indices.insert(archived_mesh_indices.end(), msg.archived_mesh_indices.begin(), msg.archived_mesh_indices.end());

  // append graph updates to current message
  for (auto&& [layer_id, layer_update] : msg.graph_update) {
    if (!layer_update) {
      continue;
    }

    auto iter = graph_update.find(layer_id);
    if (iter == graph_update.end()) {
      graph_update.emplace(layer_id, layer_update);
    } else {
      iter->second->append(std::move(*layer_update));
    }
  }

  msg.graph_update.clear();

  if (!msg.map_) {
    LOG(ERROR) << "Reconstruction output message contained no map!";
    return;
  }

  if (!map_) {
    // avoid copying the first map if possible
    map_ = !clone_map ? msg.map_ : std::make_shared<VolumetricMap>(msg.map_->config);
  }

  map_->updateFrom(*msg.map_);
}

void ActiveWindowOutput::setMap(const VolumetricMap& map) {
  auto new_map = map.clone();
  map_.reset(new_map.release());
}

void ActiveWindowOutput::setMap(const std::shared_ptr<VolumetricMap>& map) {
  map_ = map;
}

ActiveWindowOutput::Ptr ActiveWindowOutput::fromInput(const InputPacket& msg) {
  auto new_msg = std::make_shared<ActiveWindowOutput>();
  new_msg->timestamp_ns = msg.timestamp_ns;
  new_msg->world_t_body = msg.world_t_body;
  new_msg->world_R_body = msg.world_R_body;
  return new_msg;
}

}  // namespace hydra
