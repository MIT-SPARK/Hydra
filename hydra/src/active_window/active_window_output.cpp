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
namespace {

bool containsAny(const VolumetricMap& map, const BlockIndices& indices) {
  for (const auto& index : indices) {
    if (map.getTsdfLayer().hasBlock(index) || map.getMeshLayer().hasBlock(index) ||
        (map.hasSemantics() && map.getSemanticLayer()->hasBlock(index)) ||
        (map.hasTracking() && map.getTrackingLayer()->hasBlock(index))) {
      return true;
    }
  }

  return false;
}

template <typename Layer, typename Blocks>
void collectBlocks(const Layer& layer, Blocks& blocks) {
  blocks.reserve(layer.numBlocks());
  for (const auto& block : layer) {
    blocks.push_back(layer.getBlockPtr(block.index));
  }
}

template <typename Member>
void compactLayer(std::vector<MapUpdateBatch>& batches, Member member) {
  spatial_hash::IndexHashMap<std::pair<size_t, size_t>> pending;
  for (size_t i = 0; i < batches.size(); ++i) {
    auto& batch = batches[i];
    for (const auto& index : batch.archived) {
      // An archive closes this lifetime, preserving its final payload.
      pending.erase(index);
    }

    auto& blocks = batch.*member;
    for (size_t j = 0; j < blocks.size(); ++j) {
      auto [iter, inserted] = pending.emplace(blocks[j]->index, std::make_pair(i, j));
      if (!inserted) {
        const auto [old_batch, old_block] = iter->second;
        (batches[old_batch].*member)[old_block].reset();
        iter->second = {i, j};
      }
    }
  }

  for (auto& batch : batches) {
    auto& blocks = batch.*member;
    blocks.erase(std::remove(blocks.begin(), blocks.end(), nullptr), blocks.end());
  }
}

// Replacing a block must not overwrite payloads retained by earlier batches.
template <typename Layer>
void detachBlocks(const Layer& incoming, Layer& current) {
  for (const auto& block : incoming) {
    const auto previous = current.getBlockPtr(block.index);
    // The layer and this local pointer account for two owners. Reuse voxel
    // storage only when no batch or external reader retains the old payload.
    if (previous && previous.use_count() > 2) {
      current.removeBlock(block.index);
    }
  }
}

template <typename Blocks>
void appendBlocks(Blocks& target, Blocks& source) {
  target.insert(target.end(),
                std::make_move_iterator(source.begin()),
                std::make_move_iterator(source.end()));
}

std::vector<MapUpdateBatch> compactBatches(std::vector<MapUpdateBatch> batches) {
  compactLayer(batches, &MapUpdateBatch::tsdf);
  compactLayer(batches, &MapUpdateBatch::mesh);
  compactLayer(batches, &MapUpdateBatch::semantic);
  compactLayer(batches, &MapUpdateBatch::tracking);

  std::vector<MapUpdateBatch> result;
  for (auto& batch : batches) {
    if (batch.empty()) {
      continue;
    }

    if (result.empty() || !batch.archived.empty()) {
      result.push_back(std::move(batch));
      continue;
    }

    auto& previous = result.back();
    previous.timestamp_ns = batch.timestamp_ns;
    appendBlocks(previous.tsdf, batch.tsdf);
    appendBlocks(previous.mesh, batch.mesh);
    appendBlocks(previous.semantic, batch.semantic);
    appendBlocks(previous.tracking, batch.tracking);
  }

  return result;
}

template <typename Layer, typename Member>
void shareFinalBlocks(const Layer& layer,
                      std::vector<MapUpdateBatch>& batches,
                      Member member) {
  spatial_hash::IndexSet seen;
  for (auto it = batches.rbegin(); it != batches.rend(); ++it) {
    for (auto& block : (*it).*member) {
      if (!seen.insert(block->index).second) {
        continue;
      }

      const auto latest = layer.getBlockPtr(block->index);
      if (latest) {
        block = latest;
      }
    }

    seen.insert(it->archived.begin(), it->archived.end());
  }
}

}  // namespace

bool MapUpdateBatch::empty() const {
  return archived.empty() && tsdf.empty() && mesh.empty() && semantic.empty() &&
         tracking.empty();
}

const VolumetricMap& ActiveWindowOutput::map() const {
  CHECK(map_) << "Invalid map!";
  return *map_;
}

std::vector<MapUpdateBatch> ActiveWindowOutput::mapUpdates() const {
  if (!map_updates_.empty()) {
    return map_updates_;
  }

  MapUpdateBatch batch;
  batch.timestamp_ns = timestamp_ns;
  batch.archived = archived;
  if (map_) {
    collectBlocks(map_->getTsdfLayer(), batch.tsdf);
    collectBlocks(map_->getMeshLayer(), batch.mesh);
    if (map_->hasSemantics()) {
      collectBlocks(*map_->getSemanticLayer(), batch.semantic);
    }

    if (map_->hasTracking()) {
      collectBlocks(*map_->getTrackingLayer(), batch.tracking);
    }
  }

  return {std::move(batch)};
}

std::vector<MeshUpdateBatch> ActiveWindowOutput::meshUpdates() const {
  std::vector<MeshUpdateBatch> result;
  for (auto& batch : mapUpdates()) {
    if (!batch.mesh.empty() || !batch.archived.empty()) {
      result.push_back(
          {batch.timestamp_ns, std::move(batch.archived), std::move(batch.mesh)});
    }
  }

  return result;
}

bool ActiveWindowOutput::canCollate(const ActiveWindowOutput& msg) const {
  return !(map_ && containsAny(*map_, msg.archived)) &&
         !(msg.map_ && containsAny(*msg.map_, archived));
}

void ActiveWindowOutput::updateFrom(ActiveWindowOutput&& msg, bool clone_map) {
  auto batches = mapUpdates();
  auto incoming = msg.mapUpdates();
  appendBlocks(batches, incoming);
  batches = compactBatches(std::move(batches));

  map_updates_.clear();
  updateMap(msg, clone_map);
  if (map_) {
    shareFinalBlocks(map_->getTsdfLayer(), batches, &MapUpdateBatch::tsdf);
    shareFinalBlocks(map_->getMeshLayer(), batches, &MapUpdateBatch::mesh);
    if (map_->hasSemantics()) {
      shareFinalBlocks(*map_->getSemanticLayer(), batches, &MapUpdateBatch::semantic);
    }

    if (map_->hasTracking()) {
      shareFinalBlocks(*map_->getTrackingLayer(), batches, &MapUpdateBatch::tracking);
    }
  }

  map_updates_ = std::move(batches);
  timestamp_ns = msg.timestamp_ns;
  sensor_data = msg.sensor_data;
  archived.insert(archived.end(), msg.archived.begin(), msg.archived.end());
  appendGraphUpdate(msg.graph_update);
}

void ActiveWindowOutput::appendGraphUpdate(GraphUpdate& update) {
  for (auto&& [layer_id, layer_update] : update) {
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

  update.clear();
}

void ActiveWindowOutput::updateMap(const ActiveWindowOutput& msg, bool clone_map) {
  if (!msg.map_) {
    if (map_) {
      map_->removeBlocks(msg.archived);
    }

    return;
  }

  if (!map_) {
    map_ = !clone_map ? msg.map_ : std::make_shared<VolumetricMap>(msg.map_->config);
  }

  if (map_ == msg.map_) {
    return;
  }

  map_->removeBlocks(msg.archived);
  detachBlocks(msg.map_->getTsdfLayer(), map_->getTsdfLayer());
  for (const auto& block : msg.map_->getMeshLayer()) {
    map_->getMeshLayer().removeBlock(block.index);
  }

  if (map_->hasSemantics() && msg.map_->hasSemantics()) {
    detachBlocks(*msg.map_->getSemanticLayer(), *map_->getSemanticLayer());
  }

  if (map_->hasTracking() && msg.map_->hasTracking()) {
    detachBlocks(*msg.map_->getTrackingLayer(), *map_->getTrackingLayer());
  }

  map_->updateFrom(*msg.map_);
}

void ActiveWindowOutput::setMap(const VolumetricMap& map) {
  auto copy = std::make_shared<VolumetricMap>(map.config);
  copy->updateFrom(map);
  map_updates_.clear();
  map_ = std::move(copy);
}

void ActiveWindowOutput::setMap(const std::shared_ptr<VolumetricMap>& map) {
  map_updates_.clear();
  map_ = map;
}

}  // namespace hydra
