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

#include "hydra/reconstruction/voxel_types.h"
#include "hydra/utils/layer_io.h"

namespace hydra::eval {

RoomEvaluator::RoomEvaluator(const Config& config,
                             const RoomGeometry& rooms,
                             const TsdfLayer::Ptr& tsdf)
    : config(config), rooms_(rooms), tsdf_(CHECK_NOTNULL(tsdf)) {
  computeRoomIndices();
}

void RoomEvaluator::computeRoomIndices() {
  for (const auto& room_id : rooms_.getRoomIds()) {
    room_indices_[room_id] = {};
  }

  for (const auto& block : *tsdf_) {
    for (size_t idx = 0; idx < block.numVoxels(); ++idx) {
      const auto& voxel = block.getVoxel(idx);
      if (voxel.weight < config.min_weight || voxel.distance < config.min_distance) {
        continue;
      }

      const auto pos = block.getVoxelPosition(idx);
      const auto room_id = rooms_.findRoomIndex(pos);
      if (!room_id) {
        continue;
      }

      const auto global_index = block.getGlobalVoxelIndex(idx);
      room_indices_[*room_id].insert(
          {global_index.x(), global_index.y(), global_index.z()});
    }
  }
}

const RoomIndices& RoomEvaluator::getRoomIndices() const { return room_indices_; }

void RoomEvaluator::computeDsgIndices(const DynamicSceneGraph& graph,
                                      RoomIndices& indices) const {
  const auto& rooms = graph.getLayer(DsgLayers::ROOMS);
  for (auto&& [room, room_node] : rooms.nodes()) {
    if (room_node->children().size() < config.min_room_nodes) {
      VLOG(5) << "skipping room of size: " << room_node->children().size() << " (vs. "
              << config.min_room_nodes << ")";
      continue;
    }

    indices[room] = {};

    for (const auto& child : room_node->children()) {
      const auto& place_node = graph.getNode(child);
      const auto& attrs = place_node.attributes<PlaceNodeAttributes>();
      const Point pos = attrs.position.cast<float>();
      const auto sphere = getSphereAroundPoint(*tsdf_, pos, attrs.distance);

      for (const auto& global_index : sphere) {
        auto* voxel = tsdf_->getVoxelPtr(global_index);
        if (!voxel) {
          continue;
        }
        if (voxel->weight < config.min_weight ||
            voxel->distance < config.min_distance) {
          continue;
        }

        if (config.only_labeled) {
          const auto pos = tsdf_->getVoxelPosition(global_index);
          const auto room_id = rooms_.findRoomIndex(pos);
          if (!room_id) {
            continue;
          }
        }
        indices[room].insert({global_index.x(), global_index.y(), global_index.z()});
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

GlobalIndices RoomEvaluator::getSphereAroundPoint(const TsdfLayer& layer,
                                                  const Point& center,
                                                  float radius) {
  GlobalIndices result;
  const GlobalIndex center_index = layer.getGlobalVoxelIndex(center);
  const float radius_in_voxels = radius / layer.voxel_size;

  for (float x = -radius_in_voxels; x <= radius_in_voxels; x++) {
    for (float y = -radius_in_voxels; y <= radius_in_voxels; y++) {
      for (float z = -radius_in_voxels; z <= radius_in_voxels; z++) {
        Point point_voxel_space(x, y, z);

        // check if point is inside the spheres radius
        if (point_voxel_space.norm() <= radius_in_voxels) {
          GlobalIndex voxel_offset_index(std::floor(point_voxel_space.x()),
                                         std::floor(point_voxel_space.y()),
                                         std::floor(point_voxel_space.z()));
          result.push_back(center_index + voxel_offset_index);
        }
      }
    }
  }
  return result;
}

RoomEvaluator::Ptr RoomEvaluator::fromFile(const Config& config,
                                           const std::string& room_filepath,
                                           const std::string& tsdf_filepath) {
  const auto rooms = RoomGeometry::fromFile(room_filepath);

  auto tsdf = io::loadLayer<TsdfLayer>(tsdf_filepath);
  if (!tsdf) {
    LOG(ERROR) << "Failed to load TSDF from: " << tsdf_filepath;
    return nullptr;
  }

  return std::make_unique<RoomEvaluator>(config, rooms, tsdf);
}

}  // namespace hydra::eval
