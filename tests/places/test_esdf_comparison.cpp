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
#include <hydra/places/gvd_integrator.h>
#include <hydra/reconstruction/combo_integrator.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/utils/evaluation_utils.h>

#include "hydra_test/gvd_layer_utils.h"
#include "hydra_test/place_fixtures.h"

namespace hydra {
namespace places {

using namespace voxblox;

using test::compareLayers;
using test::EsdfTestFixture;
using test::esdfVoxelsSame;
using test::LayerComparisonResult;

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

  MeshIntegratorConfig mesh_config;
  mesh_config.integrator_threads = 1;
  GvdIntegratorConfig gvd_config = gvdConfigFromEsdfConfig(esdf_config);
  Layer<GvdVoxel>::Ptr gvd_layer(new Layer<GvdVoxel>(voxel_size, voxels_per_side));
  MeshLayer::Ptr mesh_layer(new MeshLayer(voxel_size * voxels_per_side));
  ComboIntegrator gvd_integrator(
      gvd_config, tsdf_layer.get(), gvd_layer, mesh_layer, &mesh_config);

  for (size_t i = 0; i < num_poses; ++i) {
    updateTsdfIntegrator(tsdf_integrator, i);

    // we need to keep the updated flags for the second integrator
    gvd_integrator.update(0, false);
    original_integrator.updateFromTsdfLayer(true);

    LayerComparisonResult result =
        compareLayers(*gvd_layer, original_layer, &test::gvdEsdfVoxelsSame);
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
      compareLayers(original_layer, simulation_esdf, &test::esdfVoxelsSame);
  VLOG(3) << "Voxblox " << voxblox_results;

  LayerComparisonResult gvd_results =
      compareLayers(*gvd_layer, simulation_esdf, &test::gvdEsdfVoxelsSame);
  VLOG(3) << "GVD " << gvd_results;

  EXPECT_LT(gvd_results.rmse, voxblox_results.rmse);
}

}  // namespace places
}  // namespace hydra
