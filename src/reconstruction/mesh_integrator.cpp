// The original implementation from https://github.com/personalrobotics/OpenChisel and
// subsequent modifications falls under the following license:
//
// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// Modifications are also subject to the following copyright and disclaimer:
//
// Copyright 2022 Massachusetts Institute of Technology.
// All Rights Reserved
//
// Research was sponsored by the United States Air Force Research Laboratory and
// the United States Air Force Artificial Intelligence Accelerator and was
// accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
// and conclusions contained in this document are those of the authors and should
// not be interpreted as representing the official policies, either expressed or
// implied, of the United States Air Force or the U.S. Government. The U.S.
// Government is authorized to reproduce and distribute reprints for Government
// purposes notwithstanding any copyright notation herein.
#include "hydra/reconstruction/mesh_integrator.h"

#include <glog/logging.h>
#include <voxblox/integrator/integrator_utils.h>
#include <voxblox/mesh/mesh.h>
#include <voxblox/utils/meshing_utils.h>

#include <iomanip>
#include <thread>

#include "hydra/reconstruction/marching_cubes.h"
#include "hydra/reconstruction/volumetric_map.h"
#include "hydra/reconstruction/voxblox_utilities.h"

namespace hydra {

using voxblox::Block;
using voxblox::BlockIndex;
using voxblox::BlockIndexList;
using voxblox::FloatingPoint;
using voxblox::IndexElement;
using voxblox::Layer;
using voxblox::Mesh;
using voxblox::MixedThreadSafeIndex;
using voxblox::Point;
using voxblox::ThreadSafeIndex;
using voxblox::TsdfVoxel;
using voxblox::VertexIndex;
using voxblox::VoxelIndex;
namespace vutils = voxblox::utils;

MeshIntegrator::MeshIntegrator(const MeshIntegratorConfig& config) : config_(config) {
  // clang-format off
  cube_index_offsets_ << 0, 1, 1, 0, 0, 1, 1, 0,
                         0, 0, 1, 1, 0, 0, 1, 1,
                         0, 0, 0, 0, 1, 1, 1, 1;
  // clang-format on
}

void MeshIntegrator::allocateBlocks(const BlockIndexList& blocks,
                                    VolumetricMap& map) const {
  auto& mesh_layer = map.getMeshLayer();
  auto vertex_layer = map.getOccupancyLayer();
  for (const BlockIndex& block_index : blocks) {
    auto mesh = mesh_layer.allocateBlock(block_index, map.hasSemantics());
    mesh->clear();
    auto mesh_labels = mesh_layer.getSemanticBlock(block_index);
    if (mesh_labels) {
      mesh_labels->clear();
    }

    if (vertex_layer) {
      // reset layer tracking vertex occupancy in voxels
      auto vertex_block = vertex_layer->allocateBlockPtrByIndex(block_index);
      // we need to reset these so that marching cubes can assign them correctly
      for (size_t idx = 0u; idx < vertex_block->num_voxels(); ++idx) {
        vertex_block->getVoxelByLinearIndex(idx).on_surface = false;
      }
    }
  }
}

void MeshIntegrator::showUpdateInfo(const VolumetricMap& map,
                                    const BlockIndexList& blocks,
                                    int verbosity) const {
  if (!VLOG_IS_ON(verbosity)) {
    return;
  }

  const auto& mesh_layer = map.getMeshLayer();
  std::stringstream ss;
  ss << "Updated blocks:";
  for (const auto& idx : blocks) {
    const auto& block = mesh_layer.getMeshBlock(idx);
    ss << "  - " << std::setw(4) << std::setfill(' ') << block->vertices.size()
       << " vertices @ " << showIndex(idx);
  }

  VLOG(10) << ss.str();
}

void MeshIntegrator::generateMesh(VolumetricMap& map,
                                  bool only_mesh_updated_blocks,
                                  bool clear_updated_flag) const {
  // TODO(nathan) think about this more
  cube_coord_offsets_ = cube_index_offsets_.cast<FloatingPoint>() * map.voxel_size();

  const auto& tsdf = map.getTsdfLayer();
  BlockIndexList blocks;
  if (only_mesh_updated_blocks) {
    tsdf.getAllUpdatedBlocks(voxblox::Update::kMesh, &blocks);
  } else {
    tsdf.getAllAllocatedBlocks(&blocks);
  }

  allocateBlocks(blocks, map);

  // interior then exterior, but order shouldn't matter too much...
  launchThreads(blocks, true, map);
  launchThreads(blocks, false, map);
  showUpdateInfo(map, blocks, 10);

  for (const auto& block_idx : blocks) {
    map.getMeshLayer().getMeshBlock(block_idx)->updated = true;
  }

  if (!clear_updated_flag) {
    return;
  }

  for (const auto& block_idx : blocks) {
    map.getTsdfLayer().getBlockPtrByIndex(block_idx)->updated().reset(
        voxblox::Update::kMesh);
  }
}

void MeshIntegrator::launchThreads(const BlockIndexList& blocks,
                                   bool interior_pass,
                                   VolumetricMap& map) const {
  std::unique_ptr<ThreadSafeIndex> id_queue(new MixedThreadSafeIndex(blocks.size()));

  std::list<std::thread> threads;
  for (int i = 0; i < config_.integrator_threads; ++i) {
    if (interior_pass) {
      threads.emplace_back(
          &MeshIntegrator::processInterior, this, blocks, &map, id_queue.get());
    } else {
      threads.emplace_back(
          &MeshIntegrator::processExterior, this, blocks, &map, id_queue.get());
    }
  }

  for (std::thread& thread : threads) {
    thread.join();
  }
}

void MeshIntegrator::processInterior(const BlockIndexList& blocks,
                                     VolumetricMap* map,
                                     ThreadSafeIndex* index_getter) const {
  DCHECK(map != nullptr);
  DCHECK(index_getter != nullptr);

  size_t list_idx;
  while (index_getter->getNextIndex(&list_idx)) {
    const auto& block_index = blocks.at(list_idx);
    VLOG(10) << "Extracting interior for block: " << showIndex(block_index);

    VoxelIndex v_idx;
    const int limit = map->voxels_per_side() - 1;
    for (v_idx.x() = 0; v_idx.x() < limit; ++v_idx.x()) {
      for (v_idx.y() = 0; v_idx.y() < limit; ++v_idx.y()) {
        for (v_idx.z() = 0; v_idx.z() < limit; ++v_idx.z()) {
          meshBlockInterior(block_index, v_idx, *map);
        }
      }
    }
  }
}

void MeshIntegrator::processExterior(const BlockIndexList& blocks,
                                     VolumetricMap* map,
                                     ThreadSafeIndex* index_getter) const {
  DCHECK(map != nullptr);
  DCHECK(index_getter != nullptr);

  size_t list_idx;
  while (index_getter->getNextIndex(&list_idx)) {
    const auto& block_index = blocks.at(list_idx);
    VLOG(10) << "Extracting exterior for block: " << showIndex(block_index);
    const auto vps = map->voxels_per_side();
    VoxelIndex v_idx;

    // Max X plane
    // takes care of edge (x_max, y_max, z), takes care of edge (x_max, y, z_max).
    v_idx.x() = vps - 1;
    for (v_idx.z() = 0; v_idx.z() < vps; v_idx.z()++) {
      for (v_idx.y() = 0; v_idx.y() < vps; v_idx.y()++) {
        meshBlockExterior(block_index, v_idx, *map);
      }
    }

    // Max Y plane.
    // takes care of edge (x, y_max, z_max) without corner (x_max, y_max, z_max).
    v_idx.y() = vps - 1;
    for (v_idx.z() = 0; v_idx.z() < vps; v_idx.z()++) {
      for (v_idx.x() = 0; v_idx.x() < vps - 1; v_idx.x()++) {
        meshBlockExterior(block_index, v_idx, *map);
      }
    }

    // Max Z plane.
    v_idx.z() = vps - 1;
    for (v_idx.y() = 0; v_idx.y() < vps - 1; v_idx.y()++) {
      for (v_idx.x() = 0; v_idx.x() < vps - 1; v_idx.x()++) {
        meshBlockExterior(block_index, v_idx, *map);
      }
    }

    // TODO(nathan) push this earlier
    // mesh->updated = true;
  }
}

template <typename Voxel>
voxblox::Block<Voxel>* maybeGetBlockPtr(Layer<Voxel>* layer, const BlockIndex& index) {
  if (!layer) {
    return nullptr;
  }

  return layer->getBlockPtrByIndex(index).get();
}

void MeshIntegrator::meshBlockInterior(const BlockIndex& block_index,
                                       const VoxelIndex& index,
                                       VolumetricMap& map) const {
  VLOG(15) << "[mesh] processing interior voxel: " << index.transpose();
  auto mesh = map.getMeshLayer().getMeshBlock(block_index);
  auto block = map.getTsdfLayer().getBlockPtrByIndex(block_index);
  if (!block || !mesh) {
    LOG(ERROR) << "Invalid block index: " << block_index.transpose();
    return;
  }

  const auto coords = block->computeCoordinatesFromVoxelIndex(index);
  auto vertex_block = maybeGetBlockPtr(map.getOccupancyLayer(), block_index);
  const auto semantics = maybeGetBlockPtr(map.getSemanticLayer(), block_index);
  std::vector<uint32_t>* mesh_labels = nullptr;
  if (semantics) {
    mesh_labels = map.getMeshLayer().getSemanticBlock(block_index);
  }

  MarchingCubes::SdfPoints points;
  for (int i = 0; i < 8; ++i) {
    auto& point = points[i];
    VoxelIndex corner_index = index + cube_index_offsets_.col(i);
    const auto& voxel = block->getVoxelByVoxelIndex(corner_index);
    if (!vutils::getSdfIfValid(voxel, config_.min_weight, &point.distance)) {
      return;
    }

    point.weight = voxel.weight;
    point.pos = coords + cube_coord_offsets_.col(i);
    point.color = voxel.color;
    if (semantics) {
      const auto& semantic_voxel = semantics->getVoxelByVoxelIndex(corner_index);
      if (!semantic_voxel.empty) {
        point.label = semantic_voxel.semantic_label;
      }
    }

    if (vertex_block) {
      point.vertex_voxel = &vertex_block->getVoxelByVoxelIndex(corner_index);
    }
  }

  ::hydra::MarchingCubes::meshCube(block_index, points, *mesh, mesh_labels);
}

BlockIndex getNeighborIndex(const BlockIndex& block_idx,
                            int voxels_per_side,
                            VoxelIndex& corner_index) {
  BlockIndex block_offset = BlockIndex::Zero();
  for (unsigned int j = 0u; j < 3u; j++) {
    if (corner_index(j) < 0) {
      block_offset(j) = -1;
      corner_index(j) = corner_index(j) + voxels_per_side;
    } else if (corner_index(j) >= static_cast<IndexElement>(voxels_per_side)) {
      block_offset(j) = 1;
      corner_index(j) = corner_index(j) - voxels_per_side;
    }
  }

  return block_idx + block_offset;
}

void MeshIntegrator::meshBlockExterior(const BlockIndex& block_index,
                                       const VoxelIndex& index,
                                       VolumetricMap& map) const {
  VLOG(15) << "[mesh] processing exterior voxel: " << index.transpose();
  auto mesh = map.getMeshLayer().getMeshBlock(block_index);
  auto block = map.getTsdfLayer().getBlockPtrByIndex(block_index);
  if (!block || !mesh) {
    LOG(ERROR) << "Invalid block index: " << block_index.transpose();
    return;
  }

  const auto coords = block->computeCoordinatesFromVoxelIndex(index);
  auto vertex_block = maybeGetBlockPtr(map.getOccupancyLayer(), block_index);
  const auto semantics = maybeGetBlockPtr(map.getSemanticLayer(), block_index);
  std::vector<uint32_t>* mesh_labels = nullptr;
  if (semantics) {
    mesh_labels = map.getMeshLayer().getSemanticBlock(block_index);
  }

  MarchingCubes::SdfPoints points;
  for (int i = 0; i < 8; ++i) {
    auto& point = points[i];
    VoxelIndex c_idx = index + cube_index_offsets_.col(i);

    const TsdfVoxel* voxel;
    const SemanticVoxel* semantic_voxel = nullptr;
    if (block->isValidVoxelIndex(c_idx)) {
      voxel = &block->getVoxelByVoxelIndex(c_idx);
      if (vertex_block) {
        point.vertex_voxel = &vertex_block->getVoxelByVoxelIndex(c_idx);
      }
      if (semantics) {
        semantic_voxel = &semantics->getVoxelByVoxelIndex(c_idx);
      }
    } else {
      // we have to access a different block
      const auto n_idx = getNeighborIndex(block_index, map.voxels_per_side(), c_idx);
      const auto n_block = map.getTsdfLayer().getBlockPtrByIndex(n_idx);
      if (!n_block) {
        return;
      }

      CHECK(n_block->isValidVoxelIndex(c_idx));
      voxel = &n_block->getVoxelByVoxelIndex(c_idx);
      if (semantics) {
        const auto& s_block = map.getSemanticLayer()->getBlockByIndex(n_idx);
        semantic_voxel = &s_block.getVoxelByVoxelIndex(c_idx);
      }

      // // we can't ensure that neighboring blocks stay in sync with the current mesh
      // // easily, so we don't track nearest surfaces to neighboring blocks for now
      // auto neighbor_vertex_block = vertex_layer_->getBlockPtrByIndex(n_idx);
      // voxels[i] = &neighbor_vertex_block->getVoxelByVoxelIndex(c_idx);
    }

    if (!vutils::getSdfIfValid(*voxel, config_.min_weight, &point.distance)) {
      return;
    }

    point.weight = voxel->weight;
    point.pos = coords + cube_coord_offsets_.col(i);
    point.color = voxel->color;
    if (semantic_voxel && !semantic_voxel->empty) {
      point.label = semantic_voxel->semantic_label;
    }
  }

  ::hydra::MarchingCubes::meshCube(block_index, points, *mesh, mesh_labels);
}

}  // namespace hydra
