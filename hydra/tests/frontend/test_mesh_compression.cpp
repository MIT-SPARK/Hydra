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
#include "hydra/utils/pgmo_mesh_traits.h"  // IWYU pragma: keep

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
  mesh.face_voxels = {cell};
  return mesh;
}

void observeVoxel(VolumetricMap& map,
                  const GlobalIndex& cell,
                  float distance,
                  float weight = 1.0f) {
  for (size_t corner = 0; corner < 8; ++corner) {
    const GlobalIndex index = cell + MeshCompression::cube_offsets[corner];
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

class MeshCompressionTest : public testing::Test {
 protected:
  void update(const VolumetricMap& map,
              const MeshCompression::ArchivePredicate& archive = {}) {
    ++stamp;
    delta = compression.update(map, stamp, archive);
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

// An empty initial update produces an empty mesh.
TEST_F(MeshCompressionTest, EmptyUpdates) {
  update(makeMap());
  EXPECT_EQ(mesh.numVertices(), 0u);
  EXPECT_EQ(mesh.numFaces(), 0u);
}

// Redundant faces and vertices get combined
TEST_F(MeshCompressionTest, CompressionCorrect) {
  auto map = makeMap();
  triangle(map);
  update(map);

  auto other = makeMap();
  auto& block = triangle(other, GlobalIndex(1, 0, 0));
  block.resizeVertices(4);
  block.points[3] = {0.15f, 0.15f, 0.08f};
  block.faces = {{0, 1, 3}, {0, 3, 2}};
  block.face_voxels.assign(2, GlobalIndex(1, 0, 0));
  update(other);
  EXPECT_EQ(mesh.numFaces(), 3u);
  EXPECT_EQ(mesh.numVertices(), 4u);
}

// Repeated observations of the same face still only produce one face
TEST_F(MeshCompressionTest, MovingSurface) {
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

// Traits update correctly under reobservations
TEST_F(MeshCompressionTest, VertexAttributes) {
  auto map = makeMap();
  auto& block = triangle(map);
  block.colors[0] = spark_dsg::Color(10, 20, 30);
  update(map);

  block.colors[0] = spark_dsg::Color(40, 50, 60);
  update(map);
  EXPECT_EQ(delta->getVertex(0).traits.first_seen_stamp, 1u);
  EXPECT_EQ(delta->getVertex(0).traits.color[0], 40u);
}

// Incomplete free-space evidence must preserve existing geometry.
TEST_F(MeshCompressionTest, PartialObservation) {
  // add default face to compression
  auto map = makeMap();
  triangle(map);
  update(map);

  // mark a single voxel as freespace
  auto partial = makeMap();
  partial.getMeshLayer().allocateBlock(BlockIndex(0, 0, 0));
  auto& voxel = partial.getTsdfLayer().allocateVoxel(GlobalIndex(0, 0, 0));
  voxel.distance = 0.3f;
  voxel.weight = 1.0f;
  update(partial);

  // mesh should still contain default face
  EXPECT_EQ(mesh.numFaces(), 1u);
  EXPECT_EQ(mesh.numVertices(), 3u);
  EXPECT_EQ(delta->info.prev_to_curr->size(), 3u);
}

// A missing neighboring block leaves boundary geometry unknown.
TEST_F(MeshCompressionTest, MissingNeighbor) {
  // add face at edge of block
  const GlobalIndex cell(15, 0, 0);
  auto map = makeMap();
  triangle(map, cell);
  update(map);

  // drop voxels in cube in neighboring block
  auto partial = makeMap();
  observeVoxel(partial, cell, 0.3f);
  partial.getTsdfLayer().removeBlock(BlockIndex(1, 0, 0));
  update(partial);
  EXPECT_EQ(mesh.numFaces(), 1u);
}

// Configuration influences clearance checks
TEST(MeshCompression, ClearanceThresholds) {
  MeshCompression::Config config;
  config.min_weight = 0.5f;
  config.min_clearance_m = 0.2;
  MeshCompression compression(config);

  auto map = makeMap();
  triangle(map);
  compression.update(map, 1);

  // weight below min weight
  auto cleared = makeMap();
  observeVoxel(cleared, GlobalIndex(0, 0, 0), 0.3f, 0.25f);
  EXPECT_EQ(compression.update(cleared, 2)->getNumFaces(), 1u);

  // distance below min clearance
  observeVoxel(cleared, GlobalIndex(0, 0, 0), 0.15f);
  EXPECT_EQ(compression.update(cleared, 3)->getNumFaces(), 1u);

  // weight and distance enough to clear face
  observeVoxel(cleared, GlobalIndex(0, 0, 0), 0.3f);
  EXPECT_EQ(compression.update(cleared, 4)->getNumFaces(), 0u);
}

// Clearing requires eight finite, observed corners with positive clearance.
TEST_F(MeshCompressionTest, FreeCorners) {
  // set up initial face
  auto map = makeMap();
  triangle(map);
  update(map);

  // make volumetric map that clears face
  auto cleared = makeMap();
  observeVoxel(cleared, GlobalIndex(0, 0, 0), 0.3f);
  auto& voxel = cleared.getTsdfLayer().getVoxel(GlobalIndex(1, 1, 1));

  // no weight invalidates freespace block
  voxel.weight = 0.0f;
  update(cleared);
  EXPECT_EQ(mesh.numFaces(), 1u);

  // invalid weight also invalidates freespace block
  voxel.weight = std::numeric_limits<float>::quiet_NaN();
  update(cleared);
  EXPECT_EQ(mesh.numFaces(), 1u);

  voxel.weight = 1.0f;

  // surface distance invalidates freespace block
  voxel.distance = -0.1;
  update(cleared);
  EXPECT_EQ(mesh.numFaces(), 1u);

  // invalid distance invalidates freespace block
  voxel.distance = std::numeric_limits<float>::quiet_NaN();
  update(cleared);
  EXPECT_EQ(mesh.numFaces(), 1u);

  // finalizing freespace clears the face
  voxel.distance = 0.3f;
  update(cleared);
  EXPECT_EQ(mesh.numFaces(), 0u);
  EXPECT_EQ(mesh.numVertices(), 0u);
  EXPECT_TRUE(delta->info.prev_to_curr->empty());

  // reobserving initial map restores face
  update(map);
  EXPECT_EQ(mesh.numFaces(), 1u);
}

// Clearing one voxel retains vertices referenced by another voxel.
TEST_F(MeshCompressionTest, SharedVertices) {
  // add default face and independent face
  auto map = makeMap();
  auto& block = triangle(map);
  block.resizeVertices(4);
  block.points[3] = {0.15f, 0.15f, 0.08f};
  block.faces.push_back({1, 3, 2});
  block.face_voxels.push_back(GlobalIndex(1, 0, 0));
  update(map);

  // add observation that clears default face
  auto partial = makeMap();
  observeVoxel(partial, GlobalIndex(0, 0, 0), 0.3f);
  update(partial);
  EXPECT_EQ(mesh.numFaces(), 1u);
  EXPECT_EQ(mesh.numVertices(), 3u);
}

// Faces get deduplicated correctly
TEST_F(MeshCompressionTest, DuplicateFaces) {
  auto map = makeMap();
  auto& block = triangle(map);
  // permutation of default face with same winding order
  block.faces.push_back({2, 0, 1});
  block.face_voxels.push_back(GlobalIndex(1, 0, 0));
  update(map);
  EXPECT_EQ(mesh.numFaces(), 1u);

  auto partial = makeMap();
  observeVoxel(partial, GlobalIndex(0, 0, 0), 0.3f);
  update(partial);
  EXPECT_EQ(mesh.numFaces(), 1u);
  EXPECT_EQ(mesh.numVertices(), 3u);

  update(makeMap());
  EXPECT_EQ(mesh.numFaces(), 1u);
}

// Free-space observations cannot delete archived geometry.
TEST_F(MeshCompressionTest, ArchivedGeometry) {
  // add default face
  auto map = makeMap();
  triangle(map);
  update(map);

  // archive face
  update(makeMap(), [](const auto&) { return true; });

  // update with valid freespace where face was archived
  auto cleared = makeMap();
  observeVoxel(cleared, GlobalIndex(0, 0, 0), 0.3f);
  update(cleared);
  EXPECT_EQ(mesh.numVertices(), 3u);
  EXPECT_EQ(mesh.numFaces(), 1u);
}

// Reobservations of faces delay archival even outside the window.
TEST_F(MeshCompressionTest, ObservedOutsideWindow) {
  auto map = makeMap();
  triangle(map);
  for (size_t i = 0; i < 3; ++i) {
    // vertices aren't archived until face isn't directly observed
    update(map, [](const auto&) { return true; });
    EXPECT_EQ(delta->getNumArchivedVertices(), 0u);
  }

  update(makeMap(), [](const auto&) { return true; });
  EXPECT_EQ(delta->getNumArchivedVertices(), 3u);
  EXPECT_EQ(mesh.numFaces(), 1u);
}

// Missing face voxel indices throws exception
TEST_F(MeshCompressionTest, FaceIndexMismatch) {
  // default update works
  auto map = makeMap();
  auto& block = triangle(map);
  update(map);

  // missing voxels throws exception
  block.face_voxels.clear();
  EXPECT_THROW(compression.update(map, 2), std::invalid_argument);
}

// Replacement and clearing must preserve endpoints of archived faces.
TEST_F(MeshCompressionTest, FrozenEndpoints) {
  auto map = makeMap();
  auto& block = triangle(map);
  block.resizeVertices(5);
  block.points = {{-0.25f, 0.05f, 0.05f},
                  {-0.25f, 0.25f, 0.05f},
                  {-0.045f, 0.05f, 0.05f},
                  {0.25f, 0.05f, 0.05f},
                  {0.25f, 0.25f, 0.05f}};
  block.faces.push_back({2, 3, 4});
  block.face_voxels.push_back(GlobalIndex(1, 0, 0));
  update(map);
  update(makeMap(), [](const auto& vertex) { return vertex.pos.x() < 0.0f; });
  EXPECT_EQ(offsets.archived_vertices, 2u);
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
    const auto& correspondence = compression.correspondence();
    const auto active = correspondence.find(mesh_block.points.front());
    ASSERT_TRUE(active);
    EXPECT_NE(offsets.toGlobalVertex(*active), face[2]);
    if (face[2] >= offsets.prev_archived_vertices) {
      const auto& candidates =
          correspondence.retained.at(correspondence.grid.toIndex(endpoint));
      EXPECT_NE(
          std::find(
              candidates.begin(), candidates.end(), offsets.toLocalVertex(face[2])),
          candidates.end());
    }
  }

  auto cleared = makeMap();
  observeVoxel(cleared, GlobalIndex(1, 0, 0), 0.3f);
  update(cleared);
  EXPECT_EQ(mesh.numFaces(), 1u);
  EXPECT_EQ(mesh.points[face[2]], endpoint);
}

// Check that compression works input from mesh integrator
TEST(MeshCompression, FaceProvenance) {
  auto map = makeMap();
  const GlobalIndex cell(-1, 0, 0);
  observeVoxel(map, cell, 0.3f);
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
  EXPECT_EQ(block.numFaces(), 2u);
  EXPECT_EQ(block.face_voxels.size(), 2u);
  EXPECT_EQ(block.face_voxels[0], cell);
  for (const auto& tsdf_block : map.getTsdfLayer()) {
    tsdf_block.updated = true;
  }

  const auto copied = map.cloneUpdated();
  EXPECT_EQ(copied->getMeshLayer().getBlock(BlockIndex(-1, 0, 0)).face_voxels,
            block.face_voxels);

  MeshCompression compression(0.005);
  EXPECT_EQ(compression.update(map, 1)->getNumFaces(), 2u);
  observeVoxel(map, cell, 0.3f);
  integrator.generateMesh(map, false, false);
  EXPECT_TRUE(block.face_voxels.empty());
  EXPECT_EQ(compression.update(map, 2)->getNumFaces(), 0u);
}

}  // namespace hydra
