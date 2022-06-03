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
#include <gtest/gtest.h>

#include <hydra_topology/gvd_integrator.h>

namespace hydra {
namespace topology {

class GvdTestIntegrator : public GvdIntegrator {
 public:
  GvdTestIntegrator(const GvdIntegratorConfig& config,
                    Layer<TsdfVoxel>* tsdf_layer,
                    const Layer<GvdVoxel>::Ptr& gvd_layer,
                    const MeshLayer::Ptr& mesh_layer)
      : GvdIntegrator(config, tsdf_layer, gvd_layer, mesh_layer) {
    update_stats_.clear();
  }

  ~GvdTestIntegrator() = default;

  using GvdIntegrator::lower_;
  using GvdIntegrator::pushToQueue;
  using GvdIntegrator::PushType;
  using GvdIntegrator::raise_;
  using GvdIntegrator::update_stats_;
  using GvdIntegrator::updateObservedGvdVoxel;
};

struct EsdfHelpers : ::testing::Test {
  EsdfHelpers() {
    const double voxel_size = 0.1;
    const size_t voxels_per_side = 16;
    tsdf_layer.reset(new Layer<TsdfVoxel>(voxel_size, voxels_per_side));
    gvd_layer.reset(new Layer<GvdVoxel>(voxel_size, voxels_per_side));
    mesh_layer.reset(new MeshLayer(voxel_size * voxels_per_side));
    GvdIntegratorConfig config;
    integrator.reset(
        new GvdTestIntegrator(config, tsdf_layer.get(), gvd_layer, mesh_layer));
  }

  void reset(const GvdIntegratorConfig& config) {
    integrator.reset(
        new GvdTestIntegrator(config, tsdf_layer.get(), gvd_layer, mesh_layer));
  }

  std::unique_ptr<GvdTestIntegrator> integrator;
  Layer<TsdfVoxel>::Ptr tsdf_layer;
  Layer<GvdVoxel>::Ptr gvd_layer;
  MeshLayer::Ptr mesh_layer;
};

TEST_F(EsdfHelpers, TestPushQueueNew) {
  GvdVoxel voxel;
  voxel.in_queue = false;
  voxel.distance = 0.1;

  integrator->pushToQueue(GlobalIndex::Zero(), voxel, GvdTestIntegrator::PushType::NEW);
  EXPECT_EQ(1u, integrator->update_stats_.number_new_voxels);
  EXPECT_EQ(0u, integrator->update_stats_.number_lowered_voxels);
  EXPECT_EQ(0u, integrator->update_stats_.number_raised_voxels);
  EXPECT_FALSE(integrator->lower_.empty());
  EXPECT_TRUE(integrator->raise_.empty());
  EXPECT_TRUE(voxel.in_queue);
}

TEST_F(EsdfHelpers, TestPushQueueLower) {
  GvdVoxel voxel;
  voxel.in_queue = false;
  voxel.distance = 0.1;

  integrator->pushToQueue(
      GlobalIndex::Zero(), voxel, GvdTestIntegrator::PushType::LOWER);
  EXPECT_EQ(0u, integrator->update_stats_.number_new_voxels);
  EXPECT_EQ(1u, integrator->update_stats_.number_lowered_voxels);
  EXPECT_EQ(0u, integrator->update_stats_.number_raised_voxels);
  EXPECT_FALSE(integrator->lower_.empty());
  EXPECT_TRUE(integrator->raise_.empty());
  EXPECT_TRUE(voxel.in_queue);
}

TEST_F(EsdfHelpers, TestPushQueueRaise) {
  GvdVoxel voxel;
  voxel.in_queue = false;
  voxel.distance = 0.1;

  integrator->pushToQueue(
      GlobalIndex::Zero(), voxel, GvdTestIntegrator::PushType::RAISE);
  EXPECT_EQ(0u, integrator->update_stats_.number_new_voxels);
  EXPECT_EQ(0u, integrator->update_stats_.number_lowered_voxels);
  EXPECT_EQ(1u, integrator->update_stats_.number_raised_voxels);
  EXPECT_TRUE(integrator->lower_.empty());
  EXPECT_FALSE(integrator->raise_.empty());
  EXPECT_FALSE(voxel.in_queue);
}

TEST_F(EsdfHelpers, TestPushQueueBoth) {
  GvdVoxel voxel;
  voxel.in_queue = false;
  voxel.distance = 0.1;

  integrator->pushToQueue(
      GlobalIndex::Zero(), voxel, GvdTestIntegrator::PushType::BOTH);
  EXPECT_EQ(0u, integrator->update_stats_.number_new_voxels);
  EXPECT_EQ(0u, integrator->update_stats_.number_lowered_voxels);
  EXPECT_EQ(1u, integrator->update_stats_.number_raised_voxels);
  EXPECT_FALSE(integrator->lower_.empty());
  EXPECT_FALSE(integrator->raise_.empty());
  EXPECT_TRUE(voxel.in_queue);
}

TEST_F(EsdfHelpers, TestObservedVoxel) {
  // make sure we know the truncation and hysteresis constants
  GvdIntegratorConfig config;
  config.min_distance_m = 0.2;
  config.min_diff_m = 1.0e-3;
  config.min_weight = 1.0e-6;
  reset(config);

  {  // test case 1: both negative and fixed, outside min diff
    TsdfVoxel tsdf_voxel;
    tsdf_voxel.weight = 1.0;
    tsdf_voxel.distance = -0.0963441;

    GvdVoxel gvd_voxel;
    gvd_voxel.distance = -0.1;
    gvd_voxel.in_queue = false;
    gvd_voxel.has_parent = true;  // avoid raising voxel

    integrator->updateObservedGvdVoxel(tsdf_voxel, GlobalIndex::Zero(), gvd_voxel);
    EXPECT_EQ(0u, integrator->update_stats_.number_new_voxels);
    EXPECT_EQ(1u, integrator->update_stats_.number_lowered_voxels);
    EXPECT_EQ(0u, integrator->update_stats_.number_raised_voxels);
    EXPECT_FALSE(integrator->lower_.empty());
    EXPECT_TRUE(integrator->raise_.empty());
    EXPECT_TRUE(gvd_voxel.in_queue);
  }
}

}  // namespace topology
}  // namespace hydra
