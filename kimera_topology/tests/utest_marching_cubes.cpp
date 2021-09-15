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
  std::vector<uint8_t> edge_status;
  interpolateEdges(
      vertex_coordinates, sdf_values, edge_coords, edge_status, gvd_voxels);

  std::set<int> valid_edges{0, 3, 8};
  for (int i = 0; i < edge_coords.cols(); ++i) {
    EXPECT_EQ(0u, edge_status[i]);
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
  std::vector<uint8_t> edge_status;
  interpolateEdges(
      vertex_coordinates, sdf_values, edge_coords, edge_status, gvd_voxels);

  std::set<int> valid_edges{0, 3, 8};
  for (int i = 0; i < edge_coords.cols(); ++i) {
    if (valid_edges.count(i)) {
      continue;
    }

    EXPECT_EQ(0u, edge_status[i]);
    EXPECT_EQ(0.0f, edge_coords.col(i).norm())
        << "i: " << edge_coords.col(i).transpose();
  }

  // see kEdgeIndexPairs for edge index to voxel index mapping
  EXPECT_EQ(3u, edge_status[0]);  // both voxels are valid
  EXPECT_EQ(2u, edge_status[3]);  // the second voxel is valid
  EXPECT_EQ(1u, edge_status[8]);  // the first voxel is valid

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

TEST(VoxelAwareMarchingCubes, CubeMeshingNearestVertexIndexCorrect) {
  voxblox::Mesh mesh;
  PointMatrix vertex_coordinates = PointMatrix::Zero();
  SdfMatrix sdf_values;
  voxblox::VertexIndex next_index = 0;

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

  std::vector<bool> voxels_in_block(8, true);
  VoxelAwareMarchingCubes::meshCube(
      vertex_coordinates, sdf_values, &next_index, &mesh, gvd_voxels, voxels_in_block);
  EXPECT_EQ(3u, next_index);

  EXPECT_TRUE(actual_voxels[0].on_surface);
  EXPECT_TRUE(actual_voxels[1].on_surface);
  // the last vertex overwrites the index for 0, and is the only valid vertex for 1
  EXPECT_EQ(2u, actual_voxels[0].block_vertex_index);
  EXPECT_EQ(2u, actual_voxels[1].block_vertex_index);
}

TEST(VoxelAwareMarchingCubes, CubeMeshingOutsideBlockCorrect) {
  voxblox::Mesh mesh;
  PointMatrix vertex_coordinates = PointMatrix::Zero();
  SdfMatrix sdf_values;
  voxblox::VertexIndex next_index = 0;

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

  std::vector<bool> voxels_in_block(8, true);
  voxels_in_block[1] = false;
  VoxelAwareMarchingCubes::meshCube(
      vertex_coordinates, sdf_values, &next_index, &mesh, gvd_voxels, voxels_in_block);
  EXPECT_EQ(3u, next_index);

  EXPECT_TRUE(actual_voxels[0].on_surface);
  EXPECT_FALSE(actual_voxels[1].on_surface);
  EXPECT_EQ(2u, actual_voxels[0].block_vertex_index);
}

}  // namespace topology
}  // namespace kimera
