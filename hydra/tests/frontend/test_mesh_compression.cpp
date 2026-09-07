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

#include <limits>

#include "hydra/frontend/mesh_compression.h"
#include "hydra/reconstruction/mesh_integrator.h"
#include "hydra/utils/pgmo_mesh_traits.h"

namespace hydra {
namespace {

VolumetricMap makeMap() {
  VolumetricMap::Config config;
  config.voxel_size = 0.1f;
  config.voxels_per_side = 16;
  return VolumetricMap(config);
}

MeshBlock& triangle(VolumetricMap& map,
                    const GlobalIndex& cell = GlobalIndex(0, 0, 0)) {
  auto& mesh = map.getMeshLayer().allocateBlock(BlockIndex(0, 0, 0));
  mesh.resizeVertices(3);
  mesh.points = {{0.05f, 0.05f, 0.08f}, {0.15f, 0.05f, 0.08f}, {0.05f, 0.15f, 0.08f}};
  mesh.faces = {{0, 1, 2}};
  mesh.face_cells = {cell};
  return mesh;
}

void observeCell(VolumetricMap& map,
                 const GlobalIndex& cell,
                 float distance,
                 float weight = 1.0f) {
  for (int corner = 0; corner < 8; ++corner) {
    const GlobalIndex offset(corner & 1, (corner >> 1) & 1, (corner >> 2) & 1);
    const GlobalIndex index = cell + offset;
    auto& voxel = map.getTsdfLayer().allocateVoxel(index);
    voxel.distance = distance;
    voxel.weight = weight;
  }
}

void checkMesh(const spark_dsg::Mesh& mesh) {
  for (const auto& face : mesh.faces) {
    for (const auto index : face) {
      EXPECT_LT(index, mesh.numVertices());
    }

    EXPECT_NE(face[0], face[1]);
    EXPECT_NE(face[1], face[2]);
    EXPECT_NE(face[0], face[2]);
  }
}

class CellCompressionTest : public testing::Test {
 protected:
  void update(const VolumetricMap& map,
              const MeshCompression::ArchivePredicate& archive = {}) {
    delta = compression.update(map, ++stamp, archive);
    delta->updateMesh(mesh, offsets);
    checkMesh(mesh);
  }

  MeshCompression compression{0.005};
  spark_dsg::Mesh mesh;
  kimera_pgmo::MeshOffsetInfo offsets;
  kimera_pgmo::MeshDelta::Ptr delta;
  uint64_t stamp = 0;
};

}  // namespace

TEST_F(CellCompressionTest, PartialReobservationPreservesUnknownGeometry) {
  auto map = makeMap();
  triangle(map);
  update(map);
  auto partial = makeMap();
  partial.getMeshLayer().allocateBlock(BlockIndex(0, 0, 0));
  auto& voxel = partial.getTsdfLayer().allocateVoxel(GlobalIndex(0, 0, 0));
  voxel.distance = 0.3f;
  voxel.weight = 1.0f;
  update(partial);
  EXPECT_EQ(mesh.numFaces(), 1u);
  EXPECT_EQ(mesh.numVertices(), 3u);
  EXPECT_EQ(delta->info.prev_to_curr->size(), 3u);
}

TEST_F(CellCompressionTest, ClearingRequiresAllEightObservedFreeCorners) {
  auto map = makeMap();
  triangle(map);
  update(map);
  auto cleared = makeMap();
  observeCell(cleared, GlobalIndex(0, 0, 0), 0.3f);
  auto& voxel = cleared.getTsdfLayer().getVoxel(GlobalIndex(1, 1, 1));
  for (const auto weight : {0.0f, std::numeric_limits<float>::quiet_NaN()}) {
    voxel.weight = weight;
    update(cleared);
    EXPECT_EQ(mesh.numFaces(), 1u);
  }

  voxel.weight = 1.0f;
  for (const auto distance : {-0.1f, 0.0f, std::numeric_limits<float>::quiet_NaN()}) {
    voxel.distance = distance;
    update(cleared);
    EXPECT_EQ(mesh.numFaces(), 1u);
  }

  voxel.distance = 0.3f;
  update(cleared);
  EXPECT_EQ(mesh.numFaces(), 0u);
  EXPECT_EQ(mesh.numVertices(), 0u);
  EXPECT_TRUE(delta->info.prev_to_curr->empty());
  update(map);
  EXPECT_EQ(mesh.numFaces(), 1u);
}

TEST_F(CellCompressionTest, ReplacesMovingGroundSurfaceWithoutAccumulatingTriangles) {
  auto map = makeMap();
  auto& block = triangle(map);
  for (size_t i = 0; i < 10; ++i) {
    for (auto& point : block.points) {
      point.z() += 0.006f;
    }

    update(map);
    ASSERT_EQ(mesh.numFaces(), 1u);
    ASSERT_EQ(mesh.numVertices(), 3u);
    EXPECT_EQ(mesh.points, block.points);
  }
}

TEST_F(CellCompressionTest, ReobservingVerticesDoesNotReplaceAnotherCellsFace) {
  auto map = makeMap();
  triangle(map);
  update(map);
  auto other = makeMap();
  auto& block = triangle(other, GlobalIndex(1, 0, 0));
  block.resizeVertices(4);
  block.points[3] = {0.15f, 0.15f, 0.08f};
  block.faces = {{0, 1, 3}, {0, 3, 2}};
  block.face_cells.assign(2, GlobalIndex(1, 0, 0));
  update(other);
  EXPECT_EQ(mesh.numFaces(), 3u);
  EXPECT_EQ(mesh.numVertices(), 4u);
}

TEST_F(CellCompressionTest, SharedVerticesSurviveClearingOneSourceCell) {
  auto map = makeMap();
  auto& block = triangle(map);
  block.resizeVertices(4);
  block.points[3] = {0.15f, 0.15f, 0.08f};
  block.faces.push_back({1, 3, 2});
  block.face_cells.push_back(GlobalIndex(1, 0, 0));
  update(map);
  auto partial = makeMap();
  observeCell(partial, GlobalIndex(0, 0, 0), 0.3f);
  update(partial);
  EXPECT_EQ(mesh.numFaces(), 1u);
  EXPECT_EQ(mesh.numVertices(), 3u);
}

TEST_F(CellCompressionTest, DuplicateFacesRetainBothSourceCells) {
  auto map = makeMap();
  auto& block = triangle(map);
  block.faces.push_back({2, 1, 0});
  block.face_cells.push_back(GlobalIndex(1, 0, 0));
  update(map);
  ASSERT_EQ(mesh.numFaces(), 1u);
  auto partial = makeMap();
  observeCell(partial, GlobalIndex(0, 0, 0), 0.3f);
  update(partial);
  EXPECT_EQ(mesh.numFaces(), 1u);
  EXPECT_EQ(mesh.numVertices(), 3u);
  update(makeMap());
  EXPECT_EQ(mesh.numFaces(), 1u);
}

TEST_F(CellCompressionTest, MissingNeighborBlockDoesNotClearBoundaryCell) {
  const GlobalIndex cell(15, 0, 0);
  auto map = makeMap();
  triangle(map, cell);
  update(map);
  auto partial = makeMap();
  observeCell(partial, cell, 0.3f);
  partial.getTsdfLayer().removeBlock(BlockIndex(1, 0, 0));
  update(partial);
  EXPECT_EQ(mesh.numFaces(), 1u);
}

TEST_F(CellCompressionTest, FrozenEndpointsPreserveArchivedFacesDuringReplacement) {
  auto map = makeMap();
  auto& block = triangle(map);
  block.resizeVertices(5);
  block.points = {{-0.25f, 0.05f, 0.05f},
                  {-0.25f, 0.25f, 0.05f},
                  {-0.045f, 0.05f, 0.05f},
                  {0.25f, 0.05f, 0.05f},
                  {0.25f, 0.25f, 0.05f}};
  block.faces.push_back({2, 3, 4});
  block.face_cells.push_back(GlobalIndex(1, 0, 0));
  update(map);
  update(makeMap(), [](const auto& vertex) { return vertex.pos.x() < 0.0f; });
  ASSERT_EQ(offsets.archived_vertices, 2u);
  ASSERT_EQ(mesh.numFaces(), 2u);
  const auto face = mesh.faces[0];
  const auto endpoint = mesh.points[face[2]];
  auto replacement = makeMap();
  auto& mesh_block = triangle(replacement, GlobalIndex(1, 0, 0));
  mesh_block.points = {
      {-0.044f, 0.05f, 0.05f}, {0.25f, 0.05f, 0.05f}, {0.25f, 0.25f, 0.05f}};
  for (size_t i = 0; i < 3; ++i) {
    update(replacement);
    ASSERT_EQ(mesh.numFaces(), 2u);
    EXPECT_EQ(mesh.faces[0], face);
    EXPECT_EQ(mesh.points[face[2]], endpoint);
  }

  auto cleared = makeMap();
  observeCell(cleared, GlobalIndex(1, 0, 0), 0.3f);
  update(cleared);
  EXPECT_EQ(mesh.numFaces(), 1u);
  EXPECT_EQ(mesh.points[face[2]], endpoint);
}

TEST_F(CellCompressionTest, ReobservationCannotClearArchivedGeometry) {
  auto map = makeMap();
  triangle(map);
  update(map);
  update(makeMap(), [](const auto&) { return true; });
  auto cleared = makeMap();
  observeCell(cleared, GlobalIndex(0, 0, 0), 0.3f);
  update(cleared);
  EXPECT_EQ(mesh.numVertices(), 3u);
  EXPECT_EQ(mesh.numFaces(), 1u);
}

TEST_F(CellCompressionTest, UpdatedVerticesOutsideWindowRemainActive) {
  auto map = makeMap();
  triangle(map);
  for (size_t i = 0; i < 3; ++i) {
    update(map, [](const auto&) { return true; });
    EXPECT_EQ(delta->getNumArchivedVertices(), 0u);
  }

  update(makeMap(), [](const auto&) { return true; });
  EXPECT_EQ(delta->getNumArchivedVertices(), 3u);
  EXPECT_EQ(mesh.numFaces(), 1u);
}

TEST_F(CellCompressionTest, PreservesAttributesAndFirstObservation) {
  auto map = makeMap();
  auto& block = triangle(map);
  block.colors[0] = spark_dsg::Color(10, 20, 30);
  update(map);
  block.colors[0] = spark_dsg::Color(40, 50, 60);
  update(map);
  EXPECT_EQ(delta->getVertex(0).traits.first_seen_stamp, 1u);
  EXPECT_EQ(delta->getVertex(0).traits.color[0], 40u);
}

TEST_F(CellCompressionTest, EmptyUpdates) {
  update(makeMap());
  EXPECT_EQ(mesh.numVertices(), 0u);
  EXPECT_EQ(mesh.numFaces(), 0u);
}

TEST(MeshCompression, MarchingCubesProvenanceSurvivesCopiesAndRemeshing) {
  auto map = makeMap();
  const GlobalIndex cell(-1, 0, 0);
  observeCell(map, cell, 0.3f);
  for (int y = 0; y < 2; ++y) {
    for (int z = 0; z < 2; ++z) {
      map.getTsdfLayer().getVoxel(GlobalIndex(0, y, z)).distance = -0.9f;
    }
  }

  MeshIntegrator::Config config;
  config.integrator_threads = 1;
  MeshIntegrator integrator(config);
  integrator.generateMesh(map, false, false);
  const auto& block = map.getMeshLayer().getBlock(BlockIndex(-1, 0, 0));
  ASSERT_EQ(block.numFaces(), 2u);
  ASSERT_EQ(block.face_cells.size(), 2u);
  EXPECT_EQ(block.face_cells[0], cell);
  for (const auto& tsdf_block : map.getTsdfLayer()) {
    tsdf_block.updated = true;
  }

  const auto copied = map.cloneUpdated();
  EXPECT_EQ(copied->getMeshLayer().getBlock(BlockIndex(-1, 0, 0)).face_cells,
            block.face_cells);
  MeshCompression compression(0.005);
  EXPECT_EQ(compression.update(map, 1)->getNumFaces(), 2u);
  observeCell(map, cell, 0.3f);
  integrator.generateMesh(map, false, false);
  EXPECT_TRUE(block.face_cells.empty());
  EXPECT_EQ(compression.update(map, 2)->getNumFaces(), 0u);
}

TEST(MeshCompression, ClearingAndReplacementAblations) {
  MeshCompression::Config config;
  config.clear_free_space = false;
  config.replace_reobserved_cells = false;
  MeshCompression compression(config);
  auto map = makeMap();
  auto& block = triangle(map);
  compression.update(map, 1);
  for (auto& point : block.points) {
    point.z() += 0.05f;
  }

  EXPECT_EQ(compression.update(map, 2)->getNumFaces(), 2u);
  auto cleared = makeMap();
  observeCell(cleared, GlobalIndex(0, 0, 0), 0.3f);
  EXPECT_EQ(compression.update(cleared, 3)->getNumFaces(), 2u);
}

TEST(MeshCompression, ConfiguredCornerWeightAndClearance) {
  MeshCompression::Config config;
  config.min_weight = 0.5f;
  config.min_clearance_m = 0.2;
  MeshCompression compression(config);
  auto map = makeMap();
  triangle(map);
  compression.update(map, 1);
  auto cleared = makeMap();
  observeCell(cleared, GlobalIndex(0, 0, 0), 0.3f, 0.25f);
  EXPECT_EQ(compression.update(cleared, 2)->getNumFaces(), 1u);
  observeCell(cleared, GlobalIndex(0, 0, 0), 0.15f);
  EXPECT_EQ(compression.update(cleared, 3)->getNumFaces(), 1u);
  observeCell(cleared, GlobalIndex(0, 0, 0), 0.3f);
  EXPECT_EQ(compression.update(cleared, 4)->getNumFaces(), 0u);
}

TEST(MeshCompression, MissingProvenanceRejectsInputBeforeMutation) {
  MeshCompression compression(0.005);
  auto map = makeMap();
  auto& block = triangle(map);
  compression.update(map, 1);
  block.face_cells.clear();
  EXPECT_THROW(compression.update(map, 2), std::invalid_argument);
  EXPECT_EQ(compression.update(makeMap(), 3)->getNumFaces(), 1u);
}

}  // namespace hydra
