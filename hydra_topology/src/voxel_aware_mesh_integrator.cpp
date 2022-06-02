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
// all
// copies or substantial portions of the Software.
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
#include "hydra_topology/voxel_aware_mesh_integrator.h"
#include "hydra_topology/voxel_aware_marching_cubes.h"

#include <glog/logging.h>

#include <voxblox/utils/meshing_utils.h>

namespace hydra {
namespace topology {

using voxblox::IndexElement;
using voxblox::MeshIntegratorConfig;
using voxblox::MixedThreadSafeIndex;
using voxblox::Point;
namespace vutils = voxblox::utils;

using TsdfLayer = Layer<TsdfVoxel>;
using TsdfBlock = Block<TsdfVoxel>;
using GvdLayer = Layer<GvdVoxel>;
using GvdBlock = Block<GvdVoxel>;

VoxelAwareMeshIntegrator::VoxelAwareMeshIntegrator(const MeshIntegratorConfig& config,
                                                   TsdfLayer* sdf_layer,
                                                   GvdLayer* gvd_layer,
                                                   MeshLayer* mesh_layer)
    : MeshIntegrator<TsdfVoxel>(config, sdf_layer, mesh_layer), gvd_layer_(gvd_layer) {
  DCHECK(gvd_layer != nullptr);
  cube_coord_offsets_ = cube_index_offsets_.cast<FloatingPoint>() * voxel_size_;
}

void VoxelAwareMeshIntegrator::launchThreads(const BlockIndexList& blocks,
                                             bool interior_pass) {
  std::unique_ptr<ThreadSafeIndex> index_getter(
      new MixedThreadSafeIndex(blocks.size()));

  std::list<std::thread> threads;
  for (size_t i = 0; i < config_.integrator_threads; ++i) {
    if (interior_pass) {
      threads.emplace_back(
          &VoxelAwareMeshIntegrator::processInterior, this, blocks, index_getter.get());
    } else {
      threads.emplace_back(
          &VoxelAwareMeshIntegrator::processExterior, this, blocks, index_getter.get());
    }
  }

  for (std::thread& thread : threads) {
    thread.join();
  }
}

void VoxelAwareMeshIntegrator::generateMesh(bool only_mesh_updated_blocks,
                                            bool clear_updated_flag) {
  CHECK(!clear_updated_flag || (sdf_layer_mutable_ != nullptr))
      << "integrator isn't mutable";

  BlockIndexList blocks;
  if (only_mesh_updated_blocks) {
    sdf_layer_const_->getAllUpdatedBlocks(voxblox::Update::kMesh, &blocks);
  } else {
    sdf_layer_const_->getAllAllocatedBlocks(&blocks);
  }

  for (const BlockIndex& block_index : blocks) {
    mesh_layer_->allocateMeshPtrByIndex(block_index);
  }

  launchThreads(blocks, true);
  launchThreads(blocks, false);

  if (clear_updated_flag) {
    for (const auto& block_idx : blocks) {
      auto block = sdf_layer_mutable_->getBlockPtrByIndex(block_idx);
      block->updated().reset(voxblox::Update::kMesh);
    }
  }
}

void VoxelAwareMeshIntegrator::processInterior(const BlockIndexList& blocks,
                                               ThreadSafeIndex* index_getter) {
  DCHECK(index_getter != nullptr);

  size_t list_idx;
  while (index_getter->getNextIndex(&list_idx)) {
    updateBlockInterior(blocks[list_idx]);
  }
}

void VoxelAwareMeshIntegrator::processExterior(const BlockIndexList& blocks,
                                               ThreadSafeIndex* index_getter) {
  DCHECK(index_getter != nullptr);

  size_t list_idx;
  while (index_getter->getNextIndex(&list_idx)) {
    updateBlockExterior(blocks[list_idx]);
  }
}

void VoxelAwareMeshIntegrator::updateBlockInterior(const BlockIndex& block_index) {
  auto mesh = mesh_layer_->getMeshPtrByIndex(block_index);
  mesh->clear();
  auto block = sdf_layer_const_->getBlockPtrByIndex(block_index);
  DCHECK(block) << "invalid SDF block for mesh";

  IndexElement vps = block->voxels_per_side();
  VertexIndex next_mesh_index = 0;

  VoxelIndex voxel_index;
  for (voxel_index.x() = 0; voxel_index.x() < vps - 1; ++voxel_index.x()) {
    for (voxel_index.y() = 0; voxel_index.y() < vps - 1; ++voxel_index.y()) {
      for (voxel_index.z() = 0; voxel_index.z() < vps - 1; ++voxel_index.z()) {
        Point coords = block->computeCoordinatesFromVoxelIndex(voxel_index);
        extractMeshInsideBlock(
            *block, voxel_index, coords, &next_mesh_index, mesh.get());
      }
    }
  }
}

void VoxelAwareMeshIntegrator::updateBlockExterior(const BlockIndex& block_index) {
  auto mesh = mesh_layer_->getMeshPtrByIndex(block_index);
  auto block = sdf_layer_const_->getBlockPtrByIndex(block_index);

  IndexElement vps = block->voxels_per_side();
  VertexIndex next_mesh_index = mesh->size();

  VoxelIndex voxel_index;

  // Max X plane
  // takes care of edge (x_max, y_max, z), takes care of edge (x_max, y, z_max).
  voxel_index.x() = vps - 1;
  for (voxel_index.z() = 0; voxel_index.z() < vps; voxel_index.z()++) {
    for (voxel_index.y() = 0; voxel_index.y() < vps; voxel_index.y()++) {
      Point coords = block->computeCoordinatesFromVoxelIndex(voxel_index);
      extractMeshOnBorder(*block, voxel_index, coords, &next_mesh_index, mesh.get());
    }
  }

  // Max Y plane.
  // takes care of edge (x, y_max, z_max) without corner (x_max, y_max, z_max).
  voxel_index.y() = vps - 1;
  for (voxel_index.z() = 0; voxel_index.z() < vps; voxel_index.z()++) {
    for (voxel_index.x() = 0; voxel_index.x() < vps - 1; voxel_index.x()++) {
      Point coords = block->computeCoordinatesFromVoxelIndex(voxel_index);
      extractMeshOnBorder(*block, voxel_index, coords, &next_mesh_index, mesh.get());
    }
  }

  // Max Z plane.
  voxel_index.z() = vps - 1;
  for (voxel_index.y() = 0; voxel_index.y() < vps - 1; voxel_index.y()++) {
    for (voxel_index.x() = 0; voxel_index.x() < vps - 1; voxel_index.x()++) {
      Point coords = block->computeCoordinatesFromVoxelIndex(voxel_index);
      extractMeshOnBorder(*block, voxel_index, coords, &next_mesh_index, mesh.get());
    }
  }

  if (config_.use_color) {
    updateMeshColor(*block, mesh.get());
  }

  mesh->updated = true;
}

void VoxelAwareMeshIntegrator::extractMeshInsideBlock(const TsdfBlock& block,
                                                      const VoxelIndex& index,
                                                      const Point& coords,
                                                      VertexIndex* next_mesh_index,
                                                      Mesh* mesh) {
  DCHECK(next_mesh_index != nullptr);
  DCHECK(mesh != nullptr);

  const BlockIndex block_index = block.block_index();
  GvdBlock::Ptr gvd_block = gvd_layer_->getBlockPtrByIndex(block_index);
  DCHECK(gvd_block != nullptr);

  PointMatrix corner_coords;
  SdfMatrix corner_sdf;
  std::vector<GvdVoxel*> gvd_voxels(8, nullptr);
  bool all_neighbors_observed = true;

  for (int i = 0; i < 8; ++i) {
    VoxelIndex corner_index = index + cube_index_offsets_.col(i);
    const TsdfVoxel& voxel = block.getVoxelByVoxelIndex(corner_index);

    if (!vutils::getSdfIfValid(voxel, config_.min_weight, &(corner_sdf(i)))) {
      all_neighbors_observed = false;
      break;
    }

    corner_coords.col(i) = coords + cube_coord_offsets_.col(i);
    gvd_voxels[i] = &gvd_block->getVoxelByVoxelIndex(corner_index);
  }

  if (all_neighbors_observed) {
    std::vector<bool> voxels_in_block(8, true);
    VoxelAwareMarchingCubes::meshCube(block_index,
                                      corner_coords,
                                      corner_sdf,
                                      next_mesh_index,
                                      mesh,
                                      gvd_voxels,
                                      voxels_in_block);
  }
}

void VoxelAwareMeshIntegrator::extractMeshOnBorder(const TsdfBlock& block,
                                                   const VoxelIndex& index,
                                                   const Point& coords,
                                                   VertexIndex* next_mesh_index,
                                                   Mesh* mesh) {
  DCHECK(next_mesh_index != nullptr);
  DCHECK(mesh != nullptr);

  const BlockIndex block_index = block.block_index();
  GvdBlock::Ptr gvd_block = gvd_layer_->getBlockPtrByIndex(block_index);
  DCHECK(gvd_block != nullptr);

  PointMatrix corner_coords;
  SdfMatrix corner_sdf;
  std::vector<GvdVoxel*> gvd_voxels(8, nullptr);

  bool all_neighbors_observed = true;

  std::vector<bool> voxels_in_block(8);
  for (int i = 0; i < 8; ++i) {
    VoxelIndex corner_index = index + cube_index_offsets_.col(i);

    if (block.isValidVoxelIndex(corner_index)) {
      voxels_in_block[i] = true;
      const TsdfVoxel& voxel = block.getVoxelByVoxelIndex(corner_index);

      if (!vutils::getSdfIfValid(voxel, config_.min_weight, &(corner_sdf(i)))) {
        all_neighbors_observed = false;
        break;
      }

      corner_coords.col(i) = coords + cube_coord_offsets_.col(i);
      gvd_voxels[i] = &gvd_block->getVoxelByVoxelIndex(corner_index);
    } else {
      voxels_in_block[i] = false;
      // We have to access a different block.
      BlockIndex block_offset = BlockIndex::Zero();

      for (unsigned int j = 0u; j < 3u; j++) {
        if (corner_index(j) < 0) {
          block_offset(j) = -1;
          corner_index(j) = corner_index(j) + voxels_per_side_;
        } else if (corner_index(j) >= static_cast<IndexElement>(voxels_per_side_)) {
          block_offset(j) = 1;
          corner_index(j) = corner_index(j) - voxels_per_side_;
        }
      }

      BlockIndex neighbor_index = block.block_index() + block_offset;

      if (sdf_layer_const_->hasBlock(neighbor_index)) {
        const Block<TsdfVoxel>& neighbor_block =
            sdf_layer_const_->getBlockByIndex(neighbor_index);

        CHECK(neighbor_block.isValidVoxelIndex(corner_index));
        const TsdfVoxel& voxel = neighbor_block.getVoxelByVoxelIndex(corner_index);
        GvdBlock::Ptr neighbor_gvd_block =
            gvd_layer_->getBlockPtrByIndex(neighbor_index);
        gvd_voxels[i] = &neighbor_gvd_block->getVoxelByVoxelIndex(corner_index);

        if (!vutils::getSdfIfValid(voxel, config_.min_weight, &(corner_sdf(i)))) {
          all_neighbors_observed = false;
          break;
        }

        corner_coords.col(i) = coords + cube_coord_offsets_.col(i);
      } else {
        all_neighbors_observed = false;
        break;
      }
    }
  }

  if (all_neighbors_observed) {
    VoxelAwareMarchingCubes::meshCube(block_index,
                                      corner_coords,
                                      corner_sdf,
                                      next_mesh_index,
                                      mesh,
                                      gvd_voxels,
                                      voxels_in_block);
  }
}

}  // namespace topology
}  // namespace hydra
