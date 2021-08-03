#include "kimera_topology_test/layer_utils.h"
#include "kimera_topology_test/test_fixtures.h"

#include <gtest/gtest.h>

#include <kimera_topology/gvd_integrator.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/utils/evaluation_utils.h>

namespace kimera {
namespace topology {

using namespace voxblox;

using test_helpers::compareLayers;
using test_helpers::EsdfTestFixture;
using test_helpers::esdfVoxelsSame;
using test_helpers::LayerComparisonResult;

#define COPY_FIELD(dest, src, field) dest.field = src.field;

inline GvdIntegratorConfig gvdConfigFromEsdfConfig(const EsdfIntegrator::Config& esdf) {
  GvdIntegratorConfig config;
  COPY_FIELD(config, esdf, max_distance_m);
  COPY_FIELD(config, esdf, min_distance_m);
  COPY_FIELD(config, esdf, min_diff_m);
  COPY_FIELD(config, esdf, min_weight);
  COPY_FIELD(config, esdf, num_buckets);
  COPY_FIELD(config, esdf, multi_queue);
  return config;
}

#undef COPY_FIELD

TEST_F(EsdfTestFixture, TestEsdfSame) {
  const float voxel_size = 0.25f;
  const int voxels_per_side = 16;

  TsdfIntegratorBase::Config tsdf_config;
  Layer<TsdfVoxel>::Ptr tsdf_layer(new Layer<TsdfVoxel>(voxel_size, voxels_per_side));
  FastTsdfIntegrator tsdf_integrator(tsdf_config, tsdf_layer.get());

  EsdfIntegrator::Config esdf_config;
  esdf_config.min_distance_m = tsdf_config.default_truncation_distance;
  esdf_config.max_distance_m = 10.0;
  esdf_config.full_euclidean_distance = true;
  esdf_config.default_distance_m = esdf_config.max_distance_m;

  Layer<EsdfVoxel> original_layer(voxel_size, voxels_per_side);
  EsdfIntegrator original_integrator(esdf_config, tsdf_layer.get(), &original_layer);

  GvdIntegratorConfig gvd_config = gvdConfigFromEsdfConfig(esdf_config);
  Layer<GvdVoxel>::Ptr gvd_layer(new Layer<GvdVoxel>(voxel_size, voxels_per_side));
  MeshLayer::Ptr mesh_layer(new MeshLayer(voxel_size * voxels_per_side));
  GvdIntegrator gvd_integrator(gvd_config, tsdf_layer.get(), gvd_layer, mesh_layer);

  for (size_t i = 0; i < num_poses; ++i) {
    updateTsdfIntegrator(tsdf_integrator, i);

    // we need to keep the updated flags for the second integrator
    gvd_integrator.updateFromTsdfLayer(false);
    original_integrator.updateFromTsdfLayer(true);

    LayerComparisonResult result =
        compareLayers(*gvd_layer, original_layer, &test_helpers::gvdEsdfVoxelsSame);
    EXPECT_EQ(0u, result.num_missing_lhs);
    EXPECT_EQ(0u, result.num_missing_rhs);
    EXPECT_EQ(0u, result.num_lhs_seen_rhs_unseen);
    EXPECT_EQ(0u, result.num_rhs_seen_lhs_unseen);

    EXPECT_EQ(original_layer.getNumberOfAllocatedBlocks(),
              gvd_layer->getNumberOfAllocatedBlocks());
  }

  Layer<EsdfVoxel> simulation_esdf(voxel_size, voxels_per_side);
  world.generateSdfFromWorld(esdf_config.max_distance_m, &simulation_esdf);

  LayerComparisonResult voxblox_results =
      compareLayers(original_layer, simulation_esdf, &test_helpers::esdfVoxelsSame);
  VLOG(3) << "Voxblox " << voxblox_results;

  LayerComparisonResult gvd_results =
      compareLayers(*gvd_layer, simulation_esdf, &test_helpers::gvdEsdfVoxelsSame);
  VLOG(3) << "GVD " << gvd_results;

  EXPECT_LT(gvd_results.rmse, voxblox_results.rmse);
}

}  // namespace topology
}  // namespace kimera
