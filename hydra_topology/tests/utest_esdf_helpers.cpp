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
    integrator.reset(new GvdTestIntegrator(config, tsdf_layer.get(), gvd_layer, mesh_layer));
  }

  void reset(const GvdIntegratorConfig& config) {
    integrator.reset(new GvdTestIntegrator(config, tsdf_layer.get(), gvd_layer, mesh_layer));
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
