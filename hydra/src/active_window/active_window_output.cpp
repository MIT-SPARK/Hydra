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

#include <glog/logging.h>

#include <algorithm>
#include <iterator>

namespace hydra {

const VolumetricMap& ActiveWindowOutput::map() const {
  CHECK(map_) << "Invalid map!";
  return *map_;
}

std::vector<MeshUpdateBatch> ActiveWindowOutput::meshUpdates() const {
  if (!mesh_updates_.empty()) {
    return mesh_updates_;
  }

  MeshUpdateBatch batch;
  batch.timestamp_ns = timestamp_ns;
  batch.archived = archived;
  if (map_) {
    const auto& layer = map_->getMeshLayer();
    for (const auto& block : layer) {
      batch.blocks.push_back(layer.getBlockPtr(block.index));
    }
    std::sort(
        batch.blocks.begin(), batch.blocks.end(), [](const auto& a, const auto& b) {
          for (int i = 0; i < 3; ++i) {
            if (a->index[i] != b->index[i]) {
              return a->index[i] < b->index[i];
            }
          }
          return false;
        });
  }
  return {std::move(batch)};
}

void ActiveWindowOutput::updateFrom(ActiveWindowOutput&& msg, bool clone_map) {
  auto batches = meshUpdates();
  auto incoming = msg.meshUpdates();
  batches.insert(batches.end(),
                 std::make_move_iterator(incoming.begin()),
                 std::make_move_iterator(incoming.end()));
  spatial_hash::IndexHashMap<std::pair<size_t, size_t>> pending;
  for (size_t i = 0; i < batches.size(); ++i) {
    auto& batch = batches[i];
    for (const auto& index : batch.archived) {
      // An archive closes a lifetime. Its final payload must survive a later
      // replacement at the same block index.
      pending.erase(index);
    }
    for (size_t j = 0; j < batch.blocks.size(); ++j) {
      const auto& block = batch.blocks[j];
      auto previous = pending.find(block->index);
      if (previous != pending.end()) {
        const auto [old_batch, old_block] = previous->second;
        batches[old_batch].blocks[old_block].reset();
      }
      pending[block->index] = {i, j};
    }
  }
  for (auto& batch : batches) {
    auto& blocks = batch.blocks;
    blocks.erase(std::remove(blocks.begin(), blocks.end(), nullptr), blocks.end());
  }
  batches.erase(std::remove_if(batches.begin(),
                               batches.end(),
                               [](const auto& batch) {
                                 return batch.blocks.empty() && batch.archived.empty();
                               }),
                batches.end());

  // One snapshot per archive barrier: without an intervening archive all
  // retained replacements can be processed together, avoiding repeated full
  // clustering/compression when the frontend falls behind.
  std::vector<MeshUpdateBatch> compacted;
  for (auto& batch : batches) {
    if (!compacted.empty() && batch.archived.empty()) {
      auto& previous = compacted.back();
      previous.timestamp_ns = batch.timestamp_ns;
      previous.blocks.insert(
          previous.blocks.end(), batch.blocks.begin(), batch.blocks.end());
    } else {
      compacted.push_back(std::move(batch));
    }
  }
  batches = std::move(compacted);

  timestamp_ns = msg.timestamp_ns;
  sensor_data = msg.sensor_data;
  archived.insert(archived.end(), msg.archived.begin(), msg.archived.end());

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

  // Detach replaced mesh blocks before merging: retained events may still refer
  // to the previous generation. Other volumetric layers need only the final state.
  if (map_ != msg.map_) {
    map_->removeBlocks(msg.archived);
    for (const auto& block : msg.map_->getMeshLayer()) {
      map_->getMeshLayer().removeBlock(block.index);
    }
    map_->updateFrom(*msg.map_);
  }

  // Share the final payload with the collated map instead of keeping a second
  // copy. Older generations remain owned only by their event records.
  spatial_hash::IndexSet seen;
  for (auto it = batches.rbegin(); it != batches.rend(); ++it) {
    for (auto& block : it->blocks) {
      if (seen.insert(block->index).second) {
        const auto latest = map_->getMeshLayer().getBlockPtr(block->index);
        if (latest) {
          block = latest;
        }
      }
    }
    seen.insert(it->archived.begin(), it->archived.end());
  }
  mesh_updates_ = std::move(batches);
}

void ActiveWindowOutput::setMap(const VolumetricMap& map) {
  mesh_updates_.clear();
  auto new_map = map.clone();
  map_.reset(new_map.release());
}

void ActiveWindowOutput::setMap(const std::shared_ptr<VolumetricMap>& map) {
  mesh_updates_.clear();
  map_ = map;
}

}  // namespace hydra
