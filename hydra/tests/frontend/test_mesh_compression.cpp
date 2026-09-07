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
#include "hydra/utils/pgmo_mesh_traits.h"

namespace hydra {
namespace {

VolumetricMap makeMap() {
  VolumetricMap::Config config;
  config.voxel_size = 0.1f;
  config.voxels_per_side = 16;
  return VolumetricMap(config);
}

MeshBlock& triangle(VolumetricMap& map) {
  map.allocateBlock(BlockIndex::Zero());
  auto& mesh = map.getMeshLayer().allocateBlock(BlockIndex(0, 0, 0));
  mesh.resizeVertices(3);
  mesh.points = {{0.05f, 0.05f, 0.05f}, {0.25f, 0.05f, 0.05f}, {0.05f, 0.25f, 0.05f}};
  mesh.faces = {{0, 1, 2}};
  return mesh;
}

void observe(VolumetricMap& map,
             const Eigen::Vector3f& pos,
             float distance,
             float weight = 1.0f) {
  auto& voxel = map.getTsdfLayer().allocateVoxel(pos);
  voxel.distance = distance;
  voxel.weight = weight;
}

void checkMesh(const spark_dsg::Mesh& mesh) {
  for (const auto& face : mesh.faces) {
    for (const auto idx : face) {
      EXPECT_LT(idx, mesh.numVertices());
    }

    EXPECT_NE(face[0], face[1]);
    EXPECT_NE(face[1], face[2]);
    EXPECT_NE(face[2], face[0]);
  }
}
class MeshCompressionBoundaryTest : public testing::Test {
 protected:
  void SetUp() override {
    auto map = makeMap();
    auto& block = triangle(map);
    block.resizeVertices(5);
    block.points = {{-0.25f, 0.05f, 0.05f},
                    {-0.25f, 0.25f, 0.05f},
                    {-0.045f, 0.05f, 0.05f},
                    {0.25f, 0.05f, 0.05f},
                    {0.25f, 0.25f, 0.05f}};
    block.faces.push_back({2, 3, 4});
    compression.update(map, 1)->updateMesh(mesh, offsets);
    compression
        .update(makeMap(), 2, [](const auto& vertex) { return vertex.pos.x() < 0.0f; })
        ->updateMesh(mesh, offsets);
    ASSERT_EQ(mesh.numFaces(), 2u);
    ASSERT_EQ(offsets.archived_vertices, 2u);
  }

  VolumetricMap reobservedTriangle() const {
    auto map = makeMap();
    auto& block = triangle(map);
    // The replacement moves within the frozen endpoint's compression cell.
    block.points = {
        {-0.044f, 0.05f, 0.05f}, {0.25f, 0.05f, 0.05f}, {0.25f, 0.25f, 0.05f}};
    return map;
  }

  MeshCompression compression{0.01};
  spark_dsg::Mesh mesh;
  kimera_pgmo::MeshOffsetInfo offsets;
};

}  // namespace

TEST(MeshCompression, PartialReobservationPreservesUnknownGeometry) {
  auto map = makeMap();
  triangle(map);
  MeshCompression compression(0.01);
  spark_dsg::Mesh mesh;
  kimera_pgmo::MeshOffsetInfo offsets;
  compression.update(map, 1)->updateMesh(mesh, offsets);

  // Same block has been reallocated after leaving reconstruction's window.
  // Only one voxel is observed, and its new mesh is empty.
  auto partial = makeMap();
  partial.allocateBlock(BlockIndex::Zero());
  partial.getMeshLayer().allocateBlock(BlockIndex(0, 0, 0));
  observe(partial, {0.05f, 0.05f, 0.05f}, 0.0f);
  auto delta = compression.update(partial, 2);
  EXPECT_EQ(delta->getNumArchivedVertices(), 0u);
  ASSERT_EQ(delta->info.prev_to_curr->size(), 3u);
  delta->updateMesh(mesh, offsets);
  EXPECT_EQ(mesh.numVertices(), 3u);
  EXPECT_EQ(mesh.numFaces(), 1u);
  checkMesh(mesh);
}

TEST(MeshCompression, OnlyObservedFreeSpaceDeletesVerticesAndIncidentFaces) {
  auto map = makeMap();
  const auto points = triangle(map).points;
  MeshCompression compression(0.01);
  spark_dsg::Mesh mesh;
  kimera_pgmo::MeshOffsetInfo offsets;
  compression.update(map, 1)->updateMesh(mesh, offsets);
  auto partial = makeMap();
  observe(partial, points[0], 0.3f);
  observe(partial, points[1], 0.3f, 0.0f);
  observe(partial, points[2], -0.3f);
  auto delta = compression.update(partial, 2);
  EXPECT_EQ(delta->info.prev_to_curr->count(0), 0u);
  EXPECT_EQ(delta->info.prev_to_curr->size(), 2u);
  delta->updateMesh(mesh, offsets);
  EXPECT_EQ(mesh.numVertices(), 2u);
  EXPECT_EQ(mesh.numFaces(), 0u);
}

TEST(MeshCompression, MissingBlocksAndNearSurfaceSamplesAreNotCleared) {
  auto map = makeMap();
  const auto points = triangle(map).points;
  MeshCompression compression(0.01);
  compression.update(map, 1);
  auto empty = makeMap();
  EXPECT_EQ(compression.update(empty, 2)->getNumVertices(), 3u);
  observe(empty, points[0], 0.04f);
  observe(empty, points[1], std::numeric_limits<float>::quiet_NaN());
  observe(empty, points[2], 0.3f, std::numeric_limits<float>::quiet_NaN());
  auto delta = compression.update(empty, 3);
  EXPECT_EQ(delta->getNumVertices(), 3u);
  EXPECT_EQ(delta->getNumFaces(), 1u);
}

TEST(MeshCompression, CompressesAcrossBlocksAndRemovesDuplicateDegenerateFaces) {
  auto map = makeMap();
  auto& first = triangle(map);
  first.faces.push_back({2, 1, 0});
  first.faces.push_back({0, 0, 1});
  auto& second = map.getMeshLayer().allocateBlock(BlockIndex(1, 0, 0));
  second.resizeVertices(3);
  second.points = first.points;
  second.points[0].x() += 0.001f;
  second.faces = {{0, 1, 2}};
  MeshCompression compression(0.01);
  auto delta = compression.update(map, 1);
  EXPECT_EQ(delta->getNumVertices(), 3u);
  EXPECT_EQ(delta->getNumFaces(), 1u);
  delta = compression.update(map, 2);
  EXPECT_EQ(delta->getNumVertices(), 3u);
  EXPECT_EQ(delta->getNumFaces(), 1u);
}

TEST(MeshCompression, PreservesTraitsAndFirstObservation) {
  auto map = makeMap();
  auto& block = triangle(map);
  block.colors[0] = spark_dsg::Color(10, 20, 30);
  MeshCompression compression(0.01);
  auto delta = compression.update(map, 10);
  EXPECT_EQ(delta->getVertex(0).traits.first_seen_stamp, 10u);
  block.colors[0] = spark_dsg::Color(40, 50, 60);
  delta = compression.update(map, 20);
  EXPECT_EQ(delta->timestamp_ns, 20u);
  EXPECT_EQ(delta->getVertex(0).traits.stamp, 20u);
  EXPECT_EQ(delta->getVertex(0).traits.first_seen_stamp, 10u);
  EXPECT_EQ(delta->getVertex(0).traits.color[0], 40u);
}

TEST(MeshCompression, PreservesInputLabelsAndTimestamps) {
  auto map = makeMap();
  auto& block = map.getMeshLayer().allocateBlock(BlockIndex(0, 0, 0), true, true);
  block.resizeVertices(1);
  block.points[0] = {0.05f, 0.05f, 0.05f};
  block.labels[0] = 7;
  block.stamps[0] = 10;
  block.first_seen_stamps[0] = 5;
  MeshCompression compression(0.01);
  compression.update(map, 20);
  block.labels[0] = 8;
  block.stamps[0] = 30;
  block.first_seen_stamps[0] = 25;
  const auto delta = compression.update(map, 40);
  const auto& traits = delta->getVertex(0).traits;
  EXPECT_TRUE(traits.properties.has_label);
  EXPECT_TRUE(traits.properties.has_stamp);
  EXPECT_TRUE(traits.properties.has_first_seen_stamp);
  EXPECT_EQ(traits.label, 8u);
  EXPECT_EQ(traits.stamp, 30u);
  EXPECT_EQ(traits.first_seen_stamp, 5u);
}

TEST(MeshCompression, ArchivesAtVertexLevelWithinOneBlock) {
  auto map = makeMap();
  auto& block = triangle(map);
  block.resizeVertices(6);
  block.points[3] = {1.05f, 0.05f, 0.05f};
  block.points[4] = {1.25f, 0.05f, 0.05f};
  block.points[5] = {1.05f, 0.25f, 0.05f};
  block.faces.push_back({3, 4, 5});
  MeshCompression compression(0.01);
  spark_dsg::Mesh mesh;
  kimera_pgmo::MeshOffsetInfo offsets;
  compression.update(map, 1)->updateMesh(mesh, offsets);
  const auto outside = [](const auto& vertex) { return vertex.pos.x() > 1.0f; };
  auto empty = makeMap();
  auto delta = compression.update(empty, 2, outside);
  EXPECT_EQ(delta->getNumArchivedVertices(), 3u);
  EXPECT_EQ(delta->getNumArchivedFaces(), 1u);
  EXPECT_EQ(delta->getNumActiveVertices(), 3u);
  delta->updateMesh(mesh, offsets);
  EXPECT_EQ(offsets.archived_vertices, 3u);
  EXPECT_EQ(mesh.numFaces(), 2u);
  delta = compression.update(empty, 3, outside);
  EXPECT_EQ(delta->getNumArchivedVertices(), 0u);
  delta->updateMesh(mesh, offsets);
  EXPECT_EQ(mesh.numVertices(), 6u);
  checkMesh(mesh);
}

TEST(MeshCompression, BoundaryFacesRemainValidThroughClearingAndArchival) {
  auto map = makeMap();
  auto& block = triangle(map);
  block.resizeVertices(5);
  block.points[0] = {-0.25f, 0.05f, 0.05f};
  block.points[1] = {-0.25f, 0.25f, 0.05f};
  block.points[2] = {-0.05f, 0.05f, 0.05f};
  block.points[3] = {0.25f, 0.05f, 0.05f};
  block.points[4] = {0.25f, 0.25f, 0.05f};
  block.faces.push_back({2, 3, 4});
  const auto points = block.points;
  MeshCompression compression(0.01);
  spark_dsg::Mesh mesh;
  kimera_pgmo::MeshOffsetInfo offsets;
  compression.update(map, 1)->updateMesh(mesh, offsets);
  auto empty = makeMap();
  auto delta = compression.update(
      empty, 2, [](const auto& vertex) { return vertex.pos.x() < 0.0f; });
  EXPECT_EQ(delta->getNumArchivedVertices(), 2u);
  EXPECT_EQ(delta->getNumArchivedFaces(), 1u);
  delta->updateMesh(mesh, offsets);
  EXPECT_EQ(mesh.numFaces(), 2u);
  checkMesh(mesh);

  // Vertex 2 supports the immutable face and must survive. Clear the mutable
  // face through vertex 3, then finish archiving the frozen boundary endpoint.
  observe(empty, points[2], 0.3f);
  observe(empty, points[3], 0.3f);
  delta = compression.update(empty, 3);
  EXPECT_EQ(delta->getNumArchivedVertices(), 1u);
  delta->updateMesh(mesh, offsets);
  EXPECT_EQ(mesh.numFaces(), 1u);
  EXPECT_EQ(mesh.numVertices(), 4u);
  EXPECT_EQ(offsets.archived_faces, 1u);
  checkMesh(mesh);
  compression.update(makeMap(), 4, [](const auto&) { return true; })
      ->updateMesh(mesh, offsets);
  checkMesh(mesh);
  EXPECT_EQ(mesh.numVertices(), 4u);
}

TEST_F(MeshCompressionBoundaryTest, ReplacesActiveTriangleWithFrozenEndpoint) {
  const auto archived_face = mesh.faces[0];
  const auto archived_endpoint = mesh.points[archived_face[2]];
  auto map = reobservedTriangle();
  for (uint64_t stamp = 3; stamp <= 5; ++stamp) {
    const auto delta = compression.update(map, stamp);
    EXPECT_EQ(delta->getNumActiveFaces(), 1u);
    delta->updateMesh(mesh, offsets);
    ASSERT_EQ(mesh.numFaces(), 2u);
    EXPECT_EQ(mesh.faces[0], archived_face);
    EXPECT_EQ(mesh.points[archived_face[2]], archived_endpoint);
    EXPECT_EQ(mesh.numVertices(), 6u);
    EXPECT_EQ(offsets.archived_vertices, 3u);
    checkMesh(mesh);
  }

  auto cleared = makeMap();
  observe(cleared, {-0.044f, 0.05f, 0.05f}, 0.3f);
  compression.update(cleared, 6)->updateMesh(mesh, offsets);
  ASSERT_EQ(mesh.numFaces(), 1u);
  EXPECT_EQ(mesh.faces[0], archived_face);
  EXPECT_EQ(mesh.points[archived_face[2]], archived_endpoint);
  checkMesh(mesh);
}

TEST_F(MeshCompressionBoundaryTest, PartialReobservationRetainsBoundaryTriangle) {
  auto map = reobservedTriangle();
  auto& block = map.getMeshLayer().getBlock(BlockIndex(0, 0, 0));
  block.resizeVertices(1);
  block.faces.clear();
  compression.update(map, 3)->updateMesh(mesh, offsets);
  EXPECT_EQ(mesh.numFaces(), 2u);
  EXPECT_EQ(offsets.archived_vertices, 2u);
  checkMesh(mesh);

  // Mutable endpoints in the cell map are not new observations by themselves.
  compression.update(makeMap(), 4)->updateMesh(mesh, offsets);
  EXPECT_EQ(mesh.numFaces(), 2u);
  EXPECT_EQ(offsets.archived_vertices, 2u);
  checkMesh(mesh);

  compression.update(reobservedTriangle(), 5)->updateMesh(mesh, offsets);
  EXPECT_EQ(mesh.numFaces(), 2u);
  EXPECT_EQ(offsets.archived_vertices, 3u);
  checkMesh(mesh);
}

TEST(MeshCompression, ReobservationCannotClearArchivedGeometry) {
  auto map = makeMap();
  const auto points = triangle(map).points;
  MeshCompression compression(0.01);
  spark_dsg::Mesh mesh;
  kimera_pgmo::MeshOffsetInfo offsets;
  compression.update(map, 1)->updateMesh(mesh, offsets);
  compression.update(makeMap(), 2, [](const auto&) { return true; })
      ->updateMesh(mesh, offsets);
  auto partial = makeMap();
  observe(partial, points[0], 0.3f);
  compression.update(partial, 3)->updateMesh(mesh, offsets);
  EXPECT_EQ(mesh.numVertices(), 3u);
  EXPECT_EQ(mesh.numFaces(), 1u);
  checkMesh(mesh);
}

TEST(MeshCompression, UpdatedVerticesOutsideWindowAreNotRepeatedlyArchived) {
  auto map = makeMap();
  triangle(map);
  MeshCompression compression(0.01);
  spark_dsg::Mesh mesh;
  kimera_pgmo::MeshOffsetInfo offsets;
  const auto outside = [](const auto&) { return true; };
  for (uint64_t stamp = 1; stamp <= 3; ++stamp) {
    auto delta = compression.update(map, stamp, outside);
    EXPECT_EQ(delta->getNumArchivedVertices(), 0u);
    delta->updateMesh(mesh, offsets);
    EXPECT_EQ(mesh.numVertices(), 3u);
    EXPECT_EQ(mesh.numFaces(), 1u);
  }

  auto delta = compression.update(makeMap(), 4, outside);
  EXPECT_EQ(delta->getNumArchivedVertices(), 3u);
  delta->updateMesh(mesh, offsets);
  EXPECT_EQ(mesh.numVertices(), 3u);
  checkMesh(mesh);
}

TEST(MeshCompression, ReobservedVerticesReplaceOldTopology) {
  auto map = makeMap();
  auto& block = triangle(map);
  block.resizeVertices(4);
  block.points[3] = {0.25f, 0.25f, 0.05f};
  block.faces.push_back({1, 3, 2});
  MeshCompression compression(0.01);
  spark_dsg::Mesh mesh;
  kimera_pgmo::MeshOffsetInfo offsets;
  compression.update(map, 1)->updateMesh(mesh, offsets);
  block.faces = {{0, 1, 3}, {0, 3, 2}};
  compression.update(map, 2)->updateMesh(mesh, offsets);
  ASSERT_EQ(mesh.numFaces(), 2u);
  EXPECT_EQ(mesh.faces, block.faces);
  checkMesh(mesh);
}

TEST(MeshCompression, ClearsEntireMeshThenAcceptsNewGeometry) {
  auto map = makeMap();
  const auto points = triangle(map).points;
  MeshCompression compression(0.01);
  spark_dsg::Mesh mesh;
  kimera_pgmo::MeshOffsetInfo offsets;
  compression.update(map, 1)->updateMesh(mesh, offsets);
  auto cleared = makeMap();
  for (const auto& pos : points) {
    observe(cleared, pos, 0.3f);
  }

  auto delta = compression.update(cleared, 2);
  EXPECT_TRUE(delta->info.prev_to_curr->empty());
  delta->updateMesh(mesh, offsets);
  EXPECT_EQ(mesh.numVertices(), 0u);
  EXPECT_EQ(mesh.numFaces(), 0u);
  delta = compression.update(map, 3);
  EXPECT_EQ(delta->info.prev_active_vertices, 0u);
  delta->updateMesh(mesh, offsets);
  EXPECT_EQ(mesh.numVertices(), 3u);
  EXPECT_EQ(mesh.numFaces(), 1u);
  checkMesh(mesh);
}

TEST(MeshCompression, ConfigurableClearanceAndMinimumWeight) {
  auto map = makeMap();
  const auto points = triangle(map).points;
  MeshCompression::Config config;
  config.min_weight = 0.5f;
  config.min_clearance_m = 0.2;
  MeshCompression compression(config);
  compression.update(map, 1);
  auto cleared = makeMap();
  observe(cleared, points[0], 0.3f, 0.25f);
  observe(cleared, points[1], 0.15f);
  observe(cleared, points[2], 0.3f);
  auto delta = compression.update(cleared, 2);
  EXPECT_EQ(delta->getNumVertices(), 2u);
  EXPECT_EQ(delta->info.prev_to_curr->count(0), 1u);
  EXPECT_EQ(delta->info.prev_to_curr->count(1), 1u);
  EXPECT_EQ(delta->info.prev_to_curr->count(2), 0u);
}

TEST(MeshCompression, EmptyUpdates) {
  MeshCompression compression(0.01);
  auto delta = compression.update(makeMap(), 1);
  EXPECT_EQ(delta->getNumVertices(), 0u);
  EXPECT_EQ(delta->getNumFaces(), 0u);
  EXPECT_TRUE(delta->info.prev_to_curr->empty());
}

}  // namespace hydra
