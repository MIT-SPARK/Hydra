#include <gtest/gtest.h>

#include <kimera_topology/voxel_aware_marching_cubes.h>
#include <set>

namespace kimera {
namespace topology {

static constexpr float TEST_TOLERANCE = 1.0e-6f;

TEST(VoxelAwareMarchingCubes, EdgeInterpolationBasic) {
  PointMatrix vertex_coordinates = PointMatrix::Zero();
  SdfMatrix sdf_values;
  std::vector<GvdVoxel*> gvd_voxels(8, nullptr);
  // add zero-crossings at just the bottom right? corner
  sdf_values << -1.0, 1.0, 10.0, 2.0, 3.0, 10.0, 10.0, 10.0;

  // make vertices for everything that should be used for interpolation
  vertex_coordinates.col(0) << -1.0, -1.0, -1.0;
  vertex_coordinates.col(1) << 1.0, 1.0, 1.0;
  vertex_coordinates.col(3) << 1.0, 1.0, 1.0;
  vertex_coordinates.col(4) << 1.0, 1.0, 1.0;

  EdgeIndexMatrix edge_coords = EdgeIndexMatrix::Zero();
  interpolateEdges(vertex_coordinates, sdf_values, edge_coords, gvd_voxels);

  std::set<int> valid_edges{0, 3, 8};
  for (int i = 0; i < edge_coords.cols(); ++i) {
    if (valid_edges.count(i)) {
      continue;
    }

    EXPECT_EQ(0.0f, edge_coords.col(i).norm())
        << "i: " << edge_coords.col(i).transpose();
  }

  Eigen::Vector3f expected_edge0;
  expected_edge0 << 0.0f, 0.0f, 0.0f;
  EXPECT_NEAR(0.0f, (expected_edge0 - edge_coords.col(0)).norm(), TEST_TOLERANCE)
      << "0: " << edge_coords.col(0).transpose();

  Eigen::Vector3f expected_edge3;
  expected_edge3 << -1.0f / 3.0f, -1.0f / 3.0f, -1.0f / 3.0f;
  EXPECT_NEAR(0.0f, (expected_edge3 - edge_coords.col(3)).norm(), TEST_TOLERANCE)
      << "3: " << edge_coords.col(3).transpose();

  Eigen::Vector3f expected_edge8;
  expected_edge8 << -1.0f / 2.0f, -1.0f / 2.0f, -1.0f / 2.0f;
  EXPECT_NEAR(0.0f, (expected_edge8 - edge_coords.col(8)).norm(), TEST_TOLERANCE)
      << "8: " << edge_coords.col(8).transpose();
}

TEST(VoxelAwareMarchingCubes, EdgeInterpolationWithVoxels) {
  PointMatrix vertex_coordinates = PointMatrix::Zero();
  SdfMatrix sdf_values;

  GvdVoxel actual_voxels[8];
  std::vector<GvdVoxel*> gvd_voxels(8, nullptr);
  for (size_t i = 0; i < gvd_voxels.size(); ++i) {
    gvd_voxels[i] = &(actual_voxels[i]);
    gvd_voxels[i]->distance = 10.0;
  }

  // add zero-crossings at just the bottom right? corner
  sdf_values << -1.0, 1.0, 10.0, 2.0, 3.0, 10.0, 10.0, 10.0;

  // make vertices for everything that should be used for interpolation
  vertex_coordinates.col(0) << -1.0, -1.0, -1.0;
  vertex_coordinates.col(1) << 1.0, 1.0, 1.0;
  vertex_coordinates.col(3) << 1.0, 1.0, 1.0;
  vertex_coordinates.col(4) << 1.0, 1.0, 1.0;

  EdgeIndexMatrix edge_coords = EdgeIndexMatrix::Zero();
  interpolateEdges(vertex_coordinates, sdf_values, edge_coords, gvd_voxels);

  std::set<int> valid_edges{0, 3, 8};
  for (int i = 0; i < edge_coords.cols(); ++i) {
    if (valid_edges.count(i)) {
      continue;
    }

    EXPECT_EQ(0.0f, edge_coords.col(i).norm())
        << "i: " << edge_coords.col(i).transpose();
  }

  Eigen::Vector3f expected_edge0;
  expected_edge0 << 0.0f, 0.0f, 0.0f;
  EXPECT_NEAR(0.0f, (expected_edge0 - edge_coords.col(0)).norm(), TEST_TOLERANCE)
      << "0: " << edge_coords.col(0).transpose();

  Eigen::Vector3f expected_edge3;
  expected_edge3 << -1.0f / 3.0f, -1.0f / 3.0f, -1.0f / 3.0f;
  EXPECT_NEAR(0.0f, (expected_edge3 - edge_coords.col(3)).norm(), TEST_TOLERANCE)
      << "3: " << edge_coords.col(3).transpose();

  Eigen::Vector3f expected_edge8;
  expected_edge8 << -1.0f / 2.0f, -1.0f / 2.0f, -1.0f / 2.0f;
  EXPECT_NEAR(0.0f, (expected_edge8 - edge_coords.col(8)).norm(), TEST_TOLERANCE)
      << "8: " << edge_coords.col(8).transpose();

  std::set<size_t> expected_surface_vertices{0, 1};
  for (size_t i = 0; i < 8; ++i) {
    EXPECT_FALSE(actual_voxels[i].has_parent);
    if (expected_surface_vertices.count(i)) {
      EXPECT_TRUE(gvd_voxels[i]->on_surface);
    } else {
      EXPECT_FALSE(gvd_voxels[i]->on_surface);
    }
  }
}

}  // namespace topology
}  // namespace kimera
