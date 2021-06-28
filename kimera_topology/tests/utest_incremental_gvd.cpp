#include "kimera_topology_test/test_fixtures.h"

#include <gtest/gtest.h>

#include <kimera_topology/gvd_integrator.h>

namespace kimera {
namespace topology {

using test_helpers::SingleBlockTestFixture;
using voxblox::VoxelIndex;

TEST_F(SingleBlockTestFixture, DISABLED_JoinCorrect) {
  GvdIntegrator gvd_integrator(gvd_config, tsdf_layer, gvd_layer);
  gvd_config.sma_angle = M_PI / 4.0;
  for (int x = 0; x < voxels_per_side; ++x) {
    for (int y = 0; y < voxels_per_side; ++y) {
      for (int z = 0; z < voxels_per_side; ++z) {
        const bool is_edge = (x == 0) || (y == 0);
        setTsdfVoxel(x, y, z, is_edge ? 0.0 : truncation_distance);
      }
    }
  }

  gvd_integrator.updateFromTsdfLayer(true);

  for (int x = 0; x < voxels_per_side; ++x) {
    for (int y = 0; y < voxels_per_side; ++y) {
      for (int z = 0; z < voxels_per_side; ++z) {
        const auto& voxel = getGvdVoxel(x, y, z);

        double expected_distance = std::min(x, y) * truncation_distance;

        EXPECT_NEAR(expected_distance, voxel.distance, 1.0e-6);
        EXPECT_TRUE(!voxel.is_voronoi || !voxel.fixed);
        EXPECT_TRUE(voxel.has_parent == !voxel.fixed);

        if (!voxel.has_parent) {
          continue;
        }

        // diagonal plane should be voronoi
        EXPECT_EQ(x == y && x >= 2, voxel.is_voronoi)
            << voxel << " @ (" << x << ", " << y << ", " << z << ")";
      }
    }
  }
}

TEST_F(SingleBlockTestFixture, DISABLED_CornerCorrect) {
  GvdIntegrator gvd_integrator(gvd_config, tsdf_layer, gvd_layer);
  gvd_integrator.updateFromTsdfLayer(true);

  for (int x = 0; x < voxels_per_side; ++x) {
    for (int y = 0; y < voxels_per_side; ++y) {
      for (int z = 0; z < voxels_per_side; ++z) {
        const auto& voxel = getGvdVoxel(x, y, z);

        double expected_distance = std::min(x, std::min(y, z)) * truncation_distance;

        EXPECT_NEAR(expected_distance, voxel.distance, 1.0e-6);
        EXPECT_TRUE(!voxel.is_voronoi || !voxel.fixed);
        EXPECT_TRUE(voxel.has_parent == !voxel.fixed);

        if (!voxel.has_parent) {
          continue;
        }

        // upper 2x2 should all be voronoi
        EXPECT_EQ(x >= 2 && y >= 2 && z >= 2, voxel.is_voronoi)
            << voxel << " @ (" << x << ", " << y << ", " << z << ")";
      }
    }
  }
}

}  // namespace topology
}  // namespace kimera
