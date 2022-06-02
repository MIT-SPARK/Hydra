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
#include "hydra_topology_test/test_fixtures.h"

#include <hydra_topology/gvd_utilities.h>
#include <voxblox/core/voxel.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/simulation/simulation_world.h>

using namespace voxblox;

namespace hydra {
namespace topology {
namespace test_helpers {

void EsdfTestFixture::SetUp() { setupWorld(); }

void EsdfTestFixture::updateTsdfIntegrator(TsdfIntegratorBase& integrator,
                                           size_t index) {
  Transformation world_T_camera = getPose(index);
  VLOG(10) << "world_T_camera: " << std::endl << world_T_camera;

  Eigen::Vector2i resolution;
  resolution << depth_camera_width, depth_camera_height;

  Colors colors;
  Pointcloud camera_pointcloud;
  world.getPointcloudFromTransform(world_T_camera,
                                   resolution,
                                   depth_camera_fov,
                                   depth_camera_max_distance,
                                   &camera_pointcloud,
                                   &colors);

  VLOG(10) << "camera pointcloud: " << camera_pointcloud.size() << " points";

  Pointcloud world_pointcloud;
  transformPointcloud(world_T_camera.inverse(), camera_pointcloud, &world_pointcloud);
  VLOG(10) << "world pointcloud: " << world_pointcloud.size() << " points";

  integrator.integratePointCloud(world_T_camera, world_pointcloud, colors);
}

Point EsdfTestFixture::getCenter() const { return Point(0.0, 0.0, 2.0); }

void EsdfTestFixture::setupWorld() {
  world.setBounds(Point(-5.0, -5.0, -1.0), Point(5.0, 5.0, 6.0));
  world.addObject(std::make_unique<Cylinder>(getCenter(), 2.0, 4.0, Color::Red()));
  world.addGroundLevel(0.0);
}

Transformation EsdfTestFixture::getPose(size_t index) const {
  double angle =
      2.0 * M_PI * (static_cast<double>(index) / static_cast<double>(num_angles));
  Point position(
      pose_radius * std::sin(angle), pose_radius * std::cos(angle), pose_height);

  Point direction = getCenter() - position;
  double yaw = std::atan2(direction.y(), direction.x());

  Quaternion rotation =
      Quaternion(Eigen::AngleAxis<float>(yaw, Point::UnitZ()) *
                 Eigen::AngleAxis<float>(camera_pitch, Point::UnitY()));

  return Transformation(rotation, position);
}

GvdTestFixture::GvdTestFixture() : EsdfTestFixture() {
  depth_camera_max_distance = 3.0;
  depth_camera_fov = M_PI / 7.0;
  pose_radius = 2.0;
  pose_height = 1.5;
  num_angles = 50;
  num_poses = 2;
  camera_pitch = 0.6;
  depth_camera_width = 240;
  depth_camera_height = 320;
}

Transformation GvdTestFixture::getPose(size_t) const {
  return EsdfTestFixture::getPose(0);
}

void GvdTestFixture::setupWorld() {
  world.setBounds(Point(-5.0, -5.0, -1.0), Point(5.0, 5.0, 6.0));
  world.addObject(
      std::make_unique<Cube>(Point(0.0, 0.0, 0.0), Point(1.0, 1.0, 2.0), Color::Red()));
  world.addGroundLevel(0.0);
}

void SingleBlockTestFixture::setTsdfVoxel(int x,
                                          int y,
                                          int z,
                                          float distance,
                                          float weight) {
  CHECK_LT(x, voxels_per_side);
  CHECK_LT(y, voxels_per_side);
  CHECK_LT(z, voxels_per_side);

  VoxelIndex v_index;
  v_index << x, y, z;

  auto& voxel = tsdf_block->getVoxelByVoxelIndex(v_index);
  voxel.distance = distance;
  voxel.weight = weight;
}

const GvdVoxel& SingleBlockTestFixture::getGvdVoxel(int x, int y, int z) {
  CHECK_LT(x, voxels_per_side);
  CHECK_LT(y, voxels_per_side);
  CHECK_LT(z, voxels_per_side);

  VoxelIndex v_index;
  v_index << x, y, z;

  return gvd_block->getVoxelByVoxelIndex(v_index);
}

void SingleBlockTestFixture::SetUp() {
  gvd_config.min_distance_m = truncation_distance;
  gvd_config.max_distance_m = 10.0;

  tsdf_layer.reset(new Layer<TsdfVoxel>(voxel_size, voxels_per_side));
  gvd_layer.reset(new Layer<GvdVoxel>(voxel_size, voxels_per_side));
  mesh_layer.reset(new MeshLayer(voxel_size * voxels_per_side));

  BlockIndex block_index = BlockIndex::Zero();
  tsdf_block = tsdf_layer->allocateBlockPtrByIndex(block_index);
  gvd_block = gvd_layer->allocateBlockPtrByIndex(block_index);
  tsdf_block->updated().set();

  for (int x = 0; x < voxels_per_side; ++x) {
    for (int y = 0; y < voxels_per_side; ++y) {
      for (int z = 0; z < voxels_per_side; ++z) {
        const bool is_edge = (x == 0) || (y == 0) || (z == 0);
        setTsdfVoxel(x, y, z, is_edge ? 0.0 : truncation_distance);
      }
    }
  }
}

void SingleBlockExtractionTestFixture::SetUp() {
  SingleBlockTestFixture::SetUp();
  setBlockState();

  gvd_integrator.reset(
      new GvdIntegrator(gvd_config, tsdf_layer.get(), gvd_layer, mesh_layer));
  gvd_integrator->updateFromTsdfLayer(true);
}

void SingleBlockExtractionTestFixture::setBlockState() {}

void LargeSingleBlockTestFixture::SetUp() {
  voxels_per_side = 8;
  SingleBlockTestFixture::SetUp();
}

void TestFixture2d::setTsdfVoxel(int x, int y, float distance, float weight) {
  CHECK_LT(x, voxels_per_side);
  CHECK_LT(y, voxels_per_side);

  VoxelIndex v_index;
  v_index << x, y, 0;

  auto& voxel = tsdf_block->getVoxelByVoxelIndex(v_index);
  voxel.distance = distance;
  voxel.weight = weight;

  auto& gvd_voxel = gvd_block->getVoxelByVoxelIndex(v_index);
  if (distance == 0.0) {
    setGvdSurfaceVoxel(gvd_voxel);
    gvd_voxel.block_vertex_index = 0;
    gvd_voxel.mesh_block[0] = 0;
    gvd_voxel.mesh_block[1] = 0;
    gvd_voxel.mesh_block[2] = 0;
  } else {
    gvd_voxel.on_surface = false;
  }
}

const GvdVoxel& TestFixture2d::getGvdVoxel(int x, int y) {
  CHECK_LT(x, voxels_per_side);
  CHECK_LT(y, voxels_per_side);

  VoxelIndex v_index;
  v_index << x, y, 0;

  return gvd_block->getVoxelByVoxelIndex(v_index);
}

void TestFixture2d::SetUp() {
  tsdf_layer.reset(new Layer<TsdfVoxel>(voxel_size, voxels_per_side));
  gvd_layer.reset(new Layer<GvdVoxel>(voxel_size, voxels_per_side));
  mesh_layer.reset(new MeshLayer(voxel_size * voxels_per_side));

  BlockIndex block_index = BlockIndex::Zero();
  tsdf_block = tsdf_layer->allocateBlockPtrByIndex(block_index);
  gvd_block = gvd_layer->allocateBlockPtrByIndex(block_index);
  tsdf_block->updated().set();

  for (int x = 0; x < voxels_per_side; ++x) {
    for (int y = 0; y < voxels_per_side; ++y) {
      for (int z = 1; z < voxels_per_side; ++z) {
        VoxelIndex v_index;
        v_index << x, y, 0;

        auto& voxel = tsdf_block->getVoxelByVoxelIndex(v_index);
        voxel.weight = 0.0;
      }
    }
  }
}

}  // namespace test_helpers
}  // namespace topology
}  // namespace hydra
