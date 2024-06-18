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
#include <hydra/reconstruction/marching_cubes.h>
#include <hydra/reconstruction/mesh_integrator.h>

#include <set>

namespace hydra {

static constexpr float TEST_TOLERANCE = 1.0e-6f;

using PointMatrix = Eigen::Matrix<float, 3, 8>;
using SdfMatrix = std::array<float, 8>;

void fillPointsFromMatrices(const PointMatrix& pos,
                            const SdfMatrix& sdf,
                            MarchingCubes::SdfPoints& points) {
  for (size_t i = 0; i < 8; ++i) {
    points[i].pos = pos.col(i);
    points[i].distance = sdf[i];
    points[i].weight = 1.0;
  }
}

TEST(MarchingCubes, EdgeInterpolation) {
  // add zero-crossings at just the bottom right? corner
  SdfMatrix sdf_values{-1.0, 1.0, 10.0, 2.0, 3.0, 10.0, 10.0, 10.0};

  // make vertices for everything that should be used for interpolation
  PointMatrix vertex_coordinates = PointMatrix::Zero();
  vertex_coordinates.col(0) << -1.0, -1.0, -1.0;
  vertex_coordinates.col(1) << 1.0, 1.0, 1.0;
  vertex_coordinates.col(3) << 1.0, 1.0, 1.0;
  vertex_coordinates.col(4) << 1.0, 1.0, 1.0;

  MarchingCubes::SdfPoints sdf_points;
  fillPointsFromMatrices(vertex_coordinates, sdf_values, sdf_points);
  MarchingCubes::EdgePoints edge_coords;
  for (size_t i = 0; i < edge_coords.size(); ++i) {
    edge_coords[i].pos.setZero();
  }

  MarchingCubes::EdgeStatus edge_status;
  MarchingCubes::interpolateEdges(sdf_points, edge_coords, edge_status);

  std::set<int> valid_edges{0, 3, 8};
  for (size_t i = 0; i < edge_coords.size(); ++i) {
    if (valid_edges.count(i)) {
      continue;
    }

    EXPECT_EQ(0u, edge_status[i]);
    EXPECT_EQ(0.0f, edge_coords[i].pos.norm())
        << "i: " << edge_coords[i].pos.transpose();
  }

  // see kEdgeIndexPairs for edge index to voxel index mapping
  EXPECT_EQ(3u, edge_status[0]);  // both voxels are valid
  EXPECT_EQ(2u, edge_status[3]);  // the second voxel is valid
  EXPECT_EQ(1u, edge_status[8]);  // the first voxel is valid

  Eigen::Vector3f expected_edge0;
  expected_edge0 << 0.0f, 0.0f, 0.0f;
  const auto& result0 = edge_coords[0].pos;
  EXPECT_NEAR(0.0f, (expected_edge0 - result0).norm(), TEST_TOLERANCE)
      << "0: " << result0.transpose();

  Eigen::Vector3f expected_edge3;
  expected_edge3 << -1.0f / 3.0f, -1.0f / 3.0f, -1.0f / 3.0f;
  const auto& result3 = edge_coords[3].pos;
  EXPECT_NEAR(0.0f, (expected_edge3 - result3).norm(), TEST_TOLERANCE)
      << "3: " << result3.transpose();

  Eigen::Vector3f expected_edge8;
  expected_edge8 << -1.0f / 2.0f, -1.0f / 2.0f, -1.0f / 2.0f;
  const auto& result8 = edge_coords[8].pos;
  EXPECT_NEAR(0.0f, (expected_edge8 - result8).norm(), TEST_TOLERANCE)
      << "8: " << result8.transpose();
}

TEST(MarchingCubes, CubeMeshingNearestVertexIndexCorrect) {
  // add zero-crossings at just the bottom right? corner
  SdfMatrix sdf_values{-1.0, 1.0, 10.0, 2.0, 3.0, 10.0, 10.0, 10.0};

  // make vertices for everything that should be used for interpolation
  PointMatrix vertex_coordinates = PointMatrix::Zero();
  vertex_coordinates.col(0) << -1.0, -1.0, -1.0;
  vertex_coordinates.col(1) << 1.0, 1.0, 1.0;
  vertex_coordinates.col(3) << 1.0, 1.0, 1.0;
  vertex_coordinates.col(4) << 1.0, 1.0, 1.0;

  MarchingCubes::SdfPoints sdf_points;
  fillPointsFromMatrices(vertex_coordinates, sdf_values, sdf_points);
  OccupancyVoxel actual_voxels[8];
  for (size_t i = 0; i < 8; ++i) {
    sdf_points[i].vertex_voxel = &(actual_voxels[i]);
  }

  Mesh mesh;
  BlockIndex block = BlockIndex::Zero();
  MarchingCubes::meshCube(block, sdf_points, mesh);
  EXPECT_EQ(3u, mesh.numVertices());

  EXPECT_TRUE(actual_voxels[0].on_surface);
  EXPECT_TRUE(actual_voxels[1].on_surface);
  // the last vertex overwrites the index for 0, and is the only valid vertex for 1
  EXPECT_EQ(2u, actual_voxels[0].block_vertex_index);
  EXPECT_EQ(2u, actual_voxels[1].block_vertex_index);
}

}  // namespace hydra
