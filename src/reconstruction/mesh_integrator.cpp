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

#include <iomanip>
#include <list>
#include <thread>

#include "hydra/common/common.h"
#include "hydra/reconstruction/marching_cubes.h"
#include "hydra/reconstruction/volumetric_map.h"

namespace hydra {

MeshIntegrator::MeshIntegrator(const MeshIntegratorConfig& config)
    : config(config::checkValid(config)) {}

void MeshIntegrator::allocateBlocks(const BlockIndices& blocks,
                                    VolumetricMap& map,
                                    OccupancyLayer* occupancy) const {
  auto& mesh_layer = map.getMeshLayer();
  for (const BlockIndex& block_index : blocks) {
    auto& mesh = mesh_layer.allocateBlock(block_index, map.hasSemantics());
    mesh.clear();

    if (!occupancy) {
      continue;
    }

    // reset layer tracking vertex occupancy in voxels so that marching cubes can assign
    // them correctly
    for (auto& vertex_voxel : occupancy->allocateBlock(block_index)) {
      vertex_voxel.on_surface = false;
    }
  }
}

void MeshIntegrator::showUpdateInfo(const VolumetricMap& map,
                                    const BlockIndices& blocks,
                                    int verbosity) const {
  if (!VLOG_IS_ON(verbosity)) {
    return;
  }

  const auto& mesh_layer = map.getMeshLayer();
  std::stringstream ss;
  ss << "Updated blocks:";
  for (const auto& idx : blocks) {
    const auto& block = mesh_layer.getBlock(idx);
    ss << "  - " << std::setw(4) << std::setfill(' ') << block.points.size()
       << " vertices @ " << showIndex(idx);
  }

  VLOG(10) << ss.str();
}

void MeshIntegrator::generateMesh(VolumetricMap& map,
                                  bool only_mesh_updated_blocks,
                                  bool clear_updated_flag,
                                  OccupancyLayer* occupancy) const {
  // TODO(nathan) think about this more
  cube_coord_offsets_ = cube_index_offsets_.cast<float>() * map.config.voxel_size;
  const auto& tsdf = map.getTsdfLayer();
  const BlockIndices blocks =
      only_mesh_updated_blocks ? tsdf.blockIndicesWithCondition(TsdfBlock::meshUpdated)
                               : tsdf.allocatedBlockIndices();

  allocateBlocks(blocks, map, occupancy);

  // interior then exterior, but order shouldn't matter too much...
  launchThreads(blocks, true, map, occupancy);
  launchThreads(blocks, false, map, occupancy);
  showUpdateInfo(map, blocks, 5);

  for (const auto& block_idx : blocks) {
    map.getMeshLayer().getBlock(block_idx).updated = true;
  }

  if (!clear_updated_flag) {
    return;
  }

  for (const auto& block_idx : blocks) {
    tsdf.getBlock(block_idx).mesh_updated = false;
  }
}

void MeshIntegrator::launchThreads(const BlockIndices& blocks,
                                   bool interior_pass,
                                   VolumetricMap& map,
                                   OccupancyLayer* occupancy) const {
  BlockIndexGetter index_getter(blocks);
  std::list<std::thread> threads;
  for (int i = 0; i < config.integrator_threads; ++i) {
    if (interior_pass) {
      threads.emplace_back(
          &MeshIntegrator::processInterior, this, &map, &index_getter, occupancy);
    } else {
      threads.emplace_back(
          &MeshIntegrator::processExterior, this, &map, &index_getter, occupancy);
    }
  }

  for (std::thread& thread : threads) {
    thread.join();
  }
}

void MeshIntegrator::processInterior(VolumetricMap* map,
                                     BlockIndexGetter* index_getter,
                                     OccupancyLayer* occupancy) const {
  BlockIndex block_index;

  while (index_getter->getNextIndex(block_index)) {
    VLOG(10) << "Extracting interior for block: " << showIndex(block_index);

    VoxelIndex v_idx;
    const int limit = map->config.voxels_per_side - 1;
    for (v_idx.x() = 0; v_idx.x() < limit; ++v_idx.x()) {
      for (v_idx.y() = 0; v_idx.y() < limit; ++v_idx.y()) {
        for (v_idx.z() = 0; v_idx.z() < limit; ++v_idx.z()) {
          meshBlockInterior(block_index, v_idx, *map, occupancy);
        }
      }
    }
  }
}

void MeshIntegrator::processExterior(VolumetricMap* map,
                                     BlockIndexGetter* index_getter,
                                     OccupancyLayer* occupancy) const {
  BlockIndex block_index;
  while (index_getter->getNextIndex(block_index)) {
    VLOG(10) << "Extracting exterior for block: " << showIndex(block_index);
    const auto vps = map->config.voxels_per_side;
    VoxelIndex v_idx;

    // Max X plane
    // takes care of edge (x_max, y_max, z), takes care of edge (x_max, y, z_max).
    v_idx.x() = vps - 1;
    for (v_idx.z() = 0; v_idx.z() < vps; v_idx.z()++) {
      for (v_idx.y() = 0; v_idx.y() < vps; v_idx.y()++) {
        meshBlockExterior(block_index, v_idx, *map, occupancy);
      }
    }

    // Max Y plane.
    // takes care of edge (x, y_max, z_max) without corner (x_max, y_max, z_max).
    v_idx.y() = vps - 1;
    for (v_idx.z() = 0; v_idx.z() < vps; v_idx.z()++) {
      for (v_idx.x() = 0; v_idx.x() < vps - 1; v_idx.x()++) {
        meshBlockExterior(block_index, v_idx, *map, occupancy);
      }
    }

    // Max Z plane.
    v_idx.z() = vps - 1;
    for (v_idx.y() = 0; v_idx.y() < vps - 1; v_idx.y()++) {
      for (v_idx.x() = 0; v_idx.x() < vps - 1; v_idx.x()++) {
        meshBlockExterior(block_index, v_idx, *map, occupancy);
      }
    }

    // TODO(nathan) push this earlier
    // mesh->updated = true;
  }
}

template <typename Block>
Block* maybeGetBlockPtr(spatial_hash::BlockLayer<Block>* layer,
                        const BlockIndex& index) {
  if (!layer) {
    return nullptr;
  }
  return layer->getBlockPtr(index).get();
}

void MeshIntegrator::meshBlockInterior(const BlockIndex& block_index,
                                       const VoxelIndex& index,
                                       VolumetricMap& map,
                                       OccupancyLayer* occupancy) const {
  VLOG(15) << "[mesh] processing interior voxel: " << index.transpose();
  auto mesh = map.getMeshLayer().getBlockPtr(block_index);
  auto block = map.getTsdfLayer().getBlockPtr(block_index);
  if (!block || !mesh) {
    LOG(ERROR) << "Invalid block index: " << block_index.transpose();
    return;
  }

  const auto coords = block->getVoxelPosition(index);
  auto vertex_block = maybeGetBlockPtr(occupancy, block_index);
  const auto semantics = maybeGetBlockPtr(map.getSemanticLayer(), block_index);

  MarchingCubes::SdfPoints points;
  for (int i = 0; i < 8; ++i) {
    auto& point = points[i];
    VoxelIndex corner_index = index + cube_index_offsets_.col(i);
    const auto& voxel = block->getVoxel(corner_index);
    if (voxel.weight < config.min_weight) {
      return;
    }

    point.weight = voxel.weight;
    point.distance = voxel.distance;
    point.pos = coords + cube_coord_offsets_.col(i);
    point.color = voxel.color;
    if (semantics) {
      const auto& semantic_voxel = semantics->getVoxel(corner_index);
      if (!semantic_voxel.empty) {
        point.label = semantic_voxel.semantic_label;
      }
    }

    if (vertex_block) {
      point.vertex_voxel = &vertex_block->getVoxel(corner_index);
    }
  }

  ::hydra::MarchingCubes::meshCube(block_index, points, *mesh);
}

BlockIndex MeshIntegrator::getNeighborIndex(const BlockIndex& block_idx,
                                            int voxels_per_side,
                                            VoxelIndex& corner_index) {
  BlockIndex block_offset = BlockIndex::Zero();
  for (unsigned int j = 0u; j < 3u; j++) {
    if (corner_index(j) < 0) {
      block_offset(j) = -1;
      corner_index(j) = corner_index(j) + voxels_per_side;
    } else if (corner_index(j) >= static_cast<BlockIndex::Scalar>(voxels_per_side)) {
      block_offset(j) = 1;
      corner_index(j) = corner_index(j) - voxels_per_side;
    }
  }

  return block_idx + block_offset;
}

void MeshIntegrator::meshBlockExterior(const BlockIndex& block_index,
                                       const VoxelIndex& index,
                                       VolumetricMap& map,
                                       OccupancyLayer* occupancy) const {
  VLOG(15) << "[mesh] processing exterior voxel: " << index.transpose();
  auto mesh = map.getMeshLayer().getBlockPtr(block_index);
  auto block = map.getTsdfLayer().getBlockPtr(block_index);
  if (!block || !mesh) {
    LOG(ERROR) << "Invalid block index: " << block_index.transpose();
    return;
  }

  const auto coords = block->getVoxelPosition(index);
  auto vertex_block = maybeGetBlockPtr(occupancy, block_index);
  const auto semantics = maybeGetBlockPtr(map.getSemanticLayer(), block_index);

  MarchingCubes::SdfPoints points;
  for (int i = 0; i < 8; ++i) {
    auto& point = points[i];
    VoxelIndex c_idx = index + cube_index_offsets_.col(i);

    const TsdfVoxel* voxel;
    const SemanticVoxel* semantic_voxel = nullptr;
    if (block->isValidVoxelIndex(c_idx)) {
      voxel = &block->getVoxel(c_idx);
      if (vertex_block) {
        point.vertex_voxel = &vertex_block->getVoxel(c_idx);
      }
      if (semantics) {
        semantic_voxel = &semantics->getVoxel(c_idx);
      }
    } else {
      // we have to access a different block
      const auto n_idx =
          getNeighborIndex(block_index, map.config.voxels_per_side, c_idx);
      const auto n_block = map.getTsdfLayer().getBlockPtr(n_idx);
      if (!n_block) {
        return;
      }

      CHECK(n_block->isValidVoxelIndex(c_idx));
      voxel = &n_block->getVoxel(c_idx);
      if (semantics) {
        const auto& s_block = map.getSemanticLayer()->getBlock(n_idx);
        semantic_voxel = &s_block.getVoxel(c_idx);
      }

      // // we can't ensure that neighboring blocks stay in sync with the current mesh
      // // easily, so we don't track nearest surfaces to neighboring blocks for now
      // auto neighbor_vertex_block = vertex_layer_->getBlockPtr(n_idx);
      // voxels[i] = &neighbor_vertex_block->getVoxel(c_idx);
    }

    if (voxel->weight < config.min_weight) {
      return;
    }
    point.distance = voxel->distance;
    point.weight = voxel->weight;
    point.pos = coords + cube_coord_offsets_.col(i);
    point.color = voxel->color;
    if (semantic_voxel && !semantic_voxel->empty) {
      point.label = semantic_voxel->semantic_label;
    }
  }

  ::hydra::MarchingCubes::meshCube(block_index, points, *mesh);
}

const Eigen::Matrix<int, 3, 8> MeshIntegrator::cube_index_offsets_ = [] {
  Eigen::Matrix<int, 3, 8> offsets;
  // clang-format off
  offsets << 0, 1, 1, 0, 0, 1, 1, 0,
             0, 0, 1, 1, 0, 0, 1, 1,
             0, 0, 0, 0, 1, 1, 1, 1;
  // clang-format on
  return offsets;
}();

}  // namespace hydra
