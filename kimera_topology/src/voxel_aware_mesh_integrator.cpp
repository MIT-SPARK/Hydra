// TODO(nathan) double check open-chisel license
// TODO(nathan) figure out voxblox license
#include "kimera_topology/voxel_aware_mesh_integrator.h"
#include "kimera_topology/voxel_aware_marching_cubes.h"

#include <glog/logging.h>

#include <voxblox/utils/meshing_utils.h>

namespace kimera {
namespace topology {

using voxblox::IndexElement;
using voxblox::MeshIntegratorConfig;
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

void VoxelAwareMeshIntegrator::extractMeshInsideBlock(const TsdfBlock& block,
                                                      const VoxelIndex& index,
                                                      const Point& coords,
                                                      VertexIndex* next_mesh_index,
                                                      Mesh* mesh) {
  DCHECK(next_mesh_index != nullptr);
  DCHECK(mesh != nullptr);

  // TODO(nathan) avoid multiple calls (though overhead isn't too bad)
  GvdBlock::Ptr gvd_block = gvd_layer_->getBlockPtrByIndex(block.block_index());
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
    VoxelAwareMarchingCubes::meshCube(
        corner_coords, corner_sdf, next_mesh_index, mesh, gvd_voxels, voxels_in_block);
  }
}

void VoxelAwareMeshIntegrator::extractMeshOnBorder(const TsdfBlock& block,
                                                   const VoxelIndex& index,
                                                   const Point& coords,
                                                   VertexIndex* next_mesh_index,
                                                   Mesh* mesh) {
  DCHECK(next_mesh_index != nullptr);
  DCHECK(mesh != nullptr);

  // TODO(nathan) avoid multiple calls (though overhead isn't too bad)
  GvdBlock::Ptr gvd_block = gvd_layer_->getBlockPtrByIndex(block.block_index());
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
        GvdBlock::Ptr neighbor_gvd_block = gvd_layer_->getBlockPtrByIndex(neighbor_index);
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
    VoxelAwareMarchingCubes::meshCube(
        corner_coords, corner_sdf, next_mesh_index, mesh, gvd_voxels, voxels_in_block);
  }
}

}  // namespace topology
}  // namespace kimera
