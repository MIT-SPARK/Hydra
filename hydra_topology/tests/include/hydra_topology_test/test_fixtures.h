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
#pragma once
#include <gtest/gtest.h>
#include <hydra_topology/gvd_integrator.h>
#include <hydra_topology/voxblox_types.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/simulation/simulation_world.h>

namespace hydra {
namespace topology {
namespace test_helpers {

// loosely based on the test fixture in test_sdf_integrators.cc in voxblox
class EsdfTestFixture : public ::testing::Test {
 public:
  EsdfTestFixture() = default;
  virtual ~EsdfTestFixture() = default;

  voxblox::SimulationWorld world;

  size_t num_angles = 20;
  size_t num_poses = 1;
  size_t pose_radius = 6.0;
  size_t pose_height = 2.0;
  double camera_pitch = 0.1;

  int depth_camera_width = 320;
  int depth_camera_height = 240;
  double depth_camera_fov = 2.61799;
  double depth_camera_max_distance = 10.0;

  virtual void SetUp() override;

  void updateTsdfIntegrator(voxblox::TsdfIntegratorBase& integrator, size_t index);

  virtual voxblox::Point getCenter() const;

  virtual void setupWorld();

  virtual voxblox::Transformation getPose(size_t index) const;
};

class GvdTestFixture : public EsdfTestFixture {
 public:
  GvdTestFixture();
  virtual ~GvdTestFixture() = default;

  virtual void setupWorld() override;
  virtual voxblox::Transformation getPose(size_t index) const override;
};

class SingleBlockTestFixture : public ::testing::Test {
 public:
  SingleBlockTestFixture() = default;
  virtual ~SingleBlockTestFixture() = default;

  virtual void SetUp() override;

  void setTsdfVoxel(int x, int y, int z, float distance, float weight = 0.1f);

  const GvdVoxel& getGvdVoxel(int x, int y, int z);

  float voxel_size = 0.1f;
  int voxels_per_side = 4;
  double truncation_distance = 0.1;

  Layer<TsdfVoxel>::Ptr tsdf_layer;
  Layer<GvdVoxel>::Ptr gvd_layer;
  MeshLayer::Ptr mesh_layer;

  GvdIntegratorConfig gvd_config;
  voxblox::Block<voxblox::TsdfVoxel>::Ptr tsdf_block;
  voxblox::Block<GvdVoxel>::Ptr gvd_block;
};

class LargeSingleBlockTestFixture : public SingleBlockTestFixture {
 public:
  LargeSingleBlockTestFixture() = default;
  virtual ~LargeSingleBlockTestFixture() = default;

  virtual void SetUp() override;
};

class SingleBlockExtractionTestFixture : public SingleBlockTestFixture {
 public:
  SingleBlockExtractionTestFixture() = default;
  virtual ~SingleBlockExtractionTestFixture() = default;

  virtual void SetUp() override;

  std::unique_ptr<GvdIntegrator> gvd_integrator;

 protected:
  virtual void setBlockState();
};

class TestFixture2d : public ::testing::Test {
 public:
  TestFixture2d() = default;
  virtual ~TestFixture2d() = default;

  virtual void SetUp() override;

  void setTsdfVoxel(int x, int y, float distance, float weight = 0.1f);

  const GvdVoxel& getGvdVoxel(int x, int y);

  float voxel_size = 1.0f;
  int voxels_per_side = 8;
  double truncation_distance = 0.1;

  Layer<TsdfVoxel>::Ptr tsdf_layer;
  Layer<GvdVoxel>::Ptr gvd_layer;
  MeshLayer::Ptr mesh_layer;

  GvdIntegratorConfig gvd_config;
  voxblox::Block<voxblox::TsdfVoxel>::Ptr tsdf_block;
  voxblox::Block<GvdVoxel>::Ptr gvd_block;
};

}  // namespace test_helpers
}  // namespace topology
}  // namespace hydra
