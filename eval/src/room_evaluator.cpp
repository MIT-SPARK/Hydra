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
#include "hydra/eval/room_evaluator.h"

#include <glog/logging.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/utils/planning_utils.h>

namespace hydra::eval {

using voxblox::BlockIndex;
using voxblox::Layer;
using voxblox::TsdfVoxel;
using voxblox::VoxelIndex;

RoomEvaluator::RoomEvaluator(const Config& config,
                             const RoomGeometry& rooms,
                             const Layer<TsdfVoxel>::Ptr& tsdf)
    : config(config), rooms_(rooms), tsdf_(CHECK_NOTNULL(tsdf)) {
  computeRoomIndices();
}

void RoomEvaluator::computeRoomIndices() {
  for (const auto& room_id : rooms_.getRoomIds()) {
    room_indices_[room_id] = {};
  }

  voxblox::BlockIndexList blocks;
  tsdf_->getAllAllocatedBlocks(&blocks);

  for (const auto& block_idx : blocks) {
    const auto block = tsdf_->getBlockPtrByIndex(block_idx);
    for (size_t idx = 0; idx < block->num_voxels(); ++idx) {
      const auto& voxel = block->getVoxelByLinearIndex(idx);
      if (voxel.weight < config.min_weight || voxel.distance < config.min_distance) {
        continue;
      }

      const auto pos = block->computeCoordinatesFromLinearIndex(idx);
      const auto room_id = rooms_.findRoomIndex(pos);
      if (!room_id) {
        continue;
      }

      const auto voxel_idx = block->computeVoxelIndexFromLinearIndex(idx);
      const auto global_index = lookupGlobalIndex(block_idx, voxel_idx);
      room_indices_[*room_id].insert(global_index);
    }
  }
}

const RoomIndices& RoomEvaluator::getRoomIndices() const { return room_indices_; }

void RoomEvaluator::computeDsgIndices(const DynamicSceneGraph& graph,
                                      RoomIndices& indices) const {
  const auto& rooms = graph.getLayer(DsgLayers::ROOMS);
  for (auto&& [room, room_node] : rooms.nodes()) {
    indices[room] = {};

    for (const auto& child : room_node->children()) {
      const auto& place_node = graph.getNode(child).value().get();
      const auto& attrs = place_node.attributes<PlaceNodeAttributes>();
      const voxblox::Point pos = attrs.position.cast<float>();

      voxblox::HierarchicalIndexMap sphere;
      voxblox::utils::getSphereAroundPoint(*tsdf_, pos, attrs.distance, &sphere);

      for (auto&& [block_idx, local_indices] : sphere) {
        const auto block = tsdf_->getBlockPtrByIndex(block_idx);
        if (!block) {
          continue;
        }

        for (const auto& voxel_idx : local_indices) {
          const auto& voxel = block->getVoxelByVoxelIndex(voxel_idx);
          if (voxel.weight < config.min_weight ||
              voxel.distance < config.min_distance) {
            continue;
          }

          if (config.only_labeled) {
            const auto pos = block->computeCoordinatesFromVoxelIndex(voxel_idx);
            const auto room_id = rooms_.findRoomIndex(pos);
            if (!room_id) {
              continue;
            }
          }

          indices[room].insert(lookupGlobalIndex(block_idx, voxel_idx));
        }
      }
    }
  }
}

RoomMetrics RoomEvaluator::eval(const std::string& graph_filepath) const {
  const auto graph = DynamicSceneGraph::load(graph_filepath);
  if (!graph->hasLayer(DsgLayers::ROOMS)) {
    LOG(ERROR) << "Graph file: " << graph_filepath << " does not have rooms";
    return {};
  }

  RoomIndices est_indices;
  computeDsgIndices(*graph, est_indices);
  return scoreRooms(room_indices_, est_indices);
}

RoomEvaluator::Ptr RoomEvaluator::fromFile(const Config& config,
                                           const std::string& room_filepath,
                                           const std::string& tsdf_filepath) {
  const auto rooms = RoomGeometry::fromFile(room_filepath);

  Layer<TsdfVoxel>::Ptr tsdf;
  if (!voxblox::io::LoadLayer<TsdfVoxel>(tsdf_filepath, &tsdf)) {
    LOG(ERROR) << "Failed to load TSDF from: " << tsdf_filepath;
    return nullptr;
  }

  return std::make_unique<RoomEvaluator>(config, rooms, tsdf);
}

std::array<int64_t, 3> RoomEvaluator::lookupGlobalIndex(const BlockIndex& block,
                                                        const VoxelIndex& local) const {
  const auto global_idx = voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
      block, local, tsdf_->voxels_per_side());
  return {global_idx.x(), global_idx.y(), global_idx.z()};
}

}  // namespace hydra::eval
