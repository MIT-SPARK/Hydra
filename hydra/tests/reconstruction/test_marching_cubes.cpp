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
#include <hydra/reconstruction/volumetric_map.h>

#include <set>

namespace hydra {

static constexpr float TEST_TOLERANCE = 1.0e-6f;

using spark_dsg::Mesh;
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

  MarchingCubes::interpolateEdges(sdf_points, edge_coords);

  std::set<int> valid_edges{0, 3, 8};
  for (size_t i = 0; i < edge_coords.size(); ++i) {
    if (valid_edges.count(i)) {
      continue;
    }

    EXPECT_EQ(0.0f, edge_coords[i].pos.norm())
        << "i: " << edge_coords[i].pos.transpose();
  }

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

  Mesh mesh;
  MarchingCubes::meshCube(sdf_points, mesh, VoxelIndex::Zero());
  EXPECT_EQ(3u, mesh.numVertices());
}

// Test that added face counts are correct
TEST(MarchingCubes, FaceCounts) {
  PointMatrix positions;
  positions << 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1;

  Mesh mesh;
  std::set<size_t> counts;
  for (size_t config = 0; config < 256; ++config) {
    SCOPED_TRACE(config);
    SdfMatrix distances;
    for (size_t corner = 0; corner < distances.size(); ++corner) {
      distances[corner] = config & (1u << corner) ? -1.0f : 1.0f;
    }

    MarchingCubes::SdfPoints points;
    fillPointsFromMatrices(positions, distances, points);
    const auto previous_faces = mesh.numFaces();
    const auto added = MarchingCubes::meshCube(points, mesh, VoxelIndex::Zero());
    EXPECT_EQ(added, mesh.numFaces() - previous_faces);
    counts.insert(added);
  }

  EXPECT_EQ(counts, (std::set<size_t>{0, 1, 2, 3, 4, 5}));
}

TEST(MarchingCubes, IndexedFacesPreserveAllConfigurations) {
  PointMatrix positions;
  positions << 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1;
  // Include tiny triangles, coincident intersections at zero, and the midpoint
  // fallback for small SDF differences.
  for (const auto& magnitudes : {std::pair{1.0f, 1.0f},
                                 std::pair{1.0e-8f, 1.0f},
                                 std::pair{1.0f, 0.0f},
                                 std::pair{1.0e-8f, 1.0e-8f}}) {
    for (size_t config = 0; config < 256; ++config) {
      SCOPED_TRACE(config);
      SCOPED_TRACE(::testing::PrintToString(magnitudes));
      SdfMatrix distances;
      for (size_t corner = 0; corner < distances.size(); ++corner) {
        distances[corner] =
            config & (1u << corner) ? -magnitudes.first : magnitudes.second;
      }
      MarchingCubes::SdfPoints points;
      fillPointsFromMatrices(positions, distances, points);
      Mesh original;
      Mesh indexed;
      MarchingCubes::EdgeCache cache(1);
      const auto expected =
          MarchingCubes::meshCube(points, original, VoxelIndex::Zero());
      EXPECT_EQ(MarchingCubes::meshCube(points, indexed, VoxelIndex::Zero(), &cache),
                expected);
      ASSERT_EQ(indexed.numFaces(), original.numFaces());
      for (size_t i = 0; i < indexed.numFaces(); ++i) {
        const auto& face = indexed.faces[i];
        EXPECT_NE(face[0], face[1]);
        EXPECT_NE(face[0], face[2]);
        EXPECT_NE(face[1], face[2]);
        for (size_t j = 0; j < 3; ++j) {
          EXPECT_TRUE(
              indexed.pos(face[j]).isApprox(original.pos(original.faces[i][j])));
        }
      }
      std::set<int> edges;
      size_t entries = 0;
      while (MarchingCubes::kTriangleTable[config][entries] != -1) {
        edges.insert(MarchingCubes::kTriangleTable[config][entries]);
        ++entries;
      }
      EXPECT_EQ(indexed.numFaces(), entries / 3);
      EXPECT_EQ(indexed.numVertices(), edges.size());
    }
  }
}

TEST(MeshIntegrator, SharesInteriorAndBoundaryEdgesWithoutDroppingFaces) {
  for (int axis = 0; axis < 3; ++axis) {
    SCOPED_TRACE(axis);
    VolumetricMap::Config map_config;
    map_config.voxel_size = 1.0f;
    map_config.voxels_per_side = 2;
    map_config.with_semantics = true;
    map_config.with_tracking = true;
    VolumetricMap map(map_config);
    for (int x = -1; x <= 0; ++x) {
      for (int y = -1; y <= 0; ++y) {
        for (int z = -1; z <= 0; ++z) {
          const BlockIndex index(x, y, z);
          map.allocateBlock(index);
          auto block = map.getBlock(index);
          for (size_t i = 0; i < block.tsdf->numVoxels(); ++i) {
            auto& voxel = block.tsdf->getVoxel(i);
            voxel.distance = block.tsdf->getVoxelPosition(i)[axis] + 1.0f;
            voxel.weight = 1.0f;
            voxel.color = voxel.distance < 0 ? spark_dsg::Color(0, 0, 0)
                                             : spark_dsg::Color(100, 100, 100);
            auto& semantic = block.semantic->getVoxel(i);
            semantic.empty = false;
            semantic.semantic_label = voxel.distance < 0 ? 1 : 2;
            auto& tracking = block.tracking->getVoxel(i);
            tracking.first_observed = voxel.distance < 0 ? 10 : 20;
            tracking.last_observed = voxel.distance < 0 ? 30 : 40;
          }
        }
      }
    }
    MeshIntegrator::Config config;
    config.integrator_threads = 2;
    MeshIntegrator integrator(config);
    const BlockIndex index(-1, -1, -1);
    integrator.generateMesh(map, false, false);
    const auto indexed = map.getMeshLayer().getBlock(index);
    ASSERT_EQ(indexed.numVertices(), 9u);
    ASSERT_EQ(indexed.numFaces(), 8u);
    ASSERT_EQ(indexed.face_voxels.size(), 8u);
    for (size_t i = 0; i < indexed.numVertices(); ++i) {
      EXPECT_EQ(indexed.labels[i], 2u);  // Equal weights: positive-axis endpoint wins.
      EXPECT_EQ(indexed.colors[i], spark_dsg::Color(50, 50, 50));
      EXPECT_EQ(indexed.first_seen_stamps[i], 10u);
      EXPECT_EQ(indexed.stamps[i], 40u);
    }

    // Reconstruct the same cells without a cache to compare winding, attributes,
    // and face provenance independently of the vertex numbering.
    auto& original = map.getMeshLayer().getBlock(index);
    original.clear();
    integrator.meshBlockInterior(index, VoxelIndex::Zero(), map);
    for (int x = 0; x < 2; ++x) {
      for (int y = 0; y < 2; ++y) {
        for (int z = 0; z < 2; ++z) {
          if (x || y || z) {
            integrator.meshBlockExterior(index, VoxelIndex(x, y, z), map);
          }
        }
      }
    }
    ASSERT_EQ(original.numFaces(), indexed.numFaces());
    for (size_t i = 0; i < indexed.numFaces(); ++i) {
      size_t matches = 0;
      for (size_t j = 0; j < original.numFaces(); ++j) {
        if (indexed.face_voxels[i] != original.face_voxels[j]) {
          continue;
        }
        bool same = true;
        for (size_t k = 0; k < 3; ++k) {
          same &= indexed.pos(indexed.faces[i][k])
                      .isApprox(original.pos(original.faces[j][k]));
        }
        matches += same;
      }
      EXPECT_EQ(matches, 1u);
    }
    integrator.generateMesh(map, false, false);
    EXPECT_EQ(original.faces, indexed.faces);
    EXPECT_EQ(original.numVertices(), indexed.numVertices());
  }
}

}  // namespace hydra
