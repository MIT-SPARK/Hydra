// TODO(nathan) double check open-chisel license
// TODO(nathan) figure out voxblox license
#pragma once
#include "kimera_topology/voxblox_types.h"
#include "kimera_topology/gvd_voxel.h"

#include <voxblox/mesh/mesh_integrator.h>

namespace kimera {
namespace topology {

class VoxelAwareMeshIntegrator : public voxblox::MeshIntegrator<TsdfVoxel> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VoxelAwareMeshIntegrator(const voxblox::MeshIntegratorConfig& config,
                           Layer<TsdfVoxel>* sdf_layer,
                           Layer<GvdVoxel>* gvd_layer,
                           MeshLayer* mesh_layer);

  virtual ~VoxelAwareMeshIntegrator() = default;

  virtual void extractMeshInsideBlock(const Block<TsdfVoxel>& block,
                              const VoxelIndex& index,
                              const voxblox::Point& coords,
                              VertexIndex* next_mesh_index,
                              Mesh* mesh) override;

  virtual void extractMeshOnBorder(const Block<TsdfVoxel>& block,
                           const VoxelIndex& index,
                           const voxblox::Point& coords,
                           VertexIndex* next_mesh_index,
                           Mesh* mesh) override;

 protected:
  Layer<GvdVoxel>* gvd_layer_;

  Eigen::Matrix<FloatingPoint, 3, 8> cube_coord_offsets_;
};

}  // namespace topology
}  // namespace kimera
