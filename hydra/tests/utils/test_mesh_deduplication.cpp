#include <gtest/gtest.h>
#include <hydra/utils/mesh_deduplication.h>

#include <limits>

namespace hydra {

TEST(MeshDeduplication, NeighborCellsLabelsAndEuclideanDistance) {
  MeshVertexDeduplicator dedup(1.0e-5);
  EXPECT_EQ(dedup.add({-1.0e-6f, 0, 0}, 1), 0u);
  EXPECT_EQ(dedup.add({1.0e-6f, 0, 0}, 1), 0u);
  EXPECT_EQ(dedup.add({1.0e-6f, 0, 0}, 2), 1u);
  EXPECT_EQ(dedup.add({1.0e-4f, 0, 0}, 1), 2u);

  MeshVertexDeduplicator diagonal(1.0e-5);
  EXPECT_EQ(diagonal.add({1.0e-6f, 1.0e-6f, 1.0e-6f}), 0u);
  EXPECT_EQ(diagonal.add({9.0e-6f, 9.0e-6f, 9.0e-6f}), 1u);
}

TEST(MeshDeduplication, RepresentativesDoNotDriftOrMergeTransitively) {
  MeshVertexDeduplicator dedup(1.0);
  EXPECT_EQ(dedup.add({0, 0, 0}), 0u);
  EXPECT_EQ(dedup.add({0.75f, 0, 0}), 0u);
  EXPECT_EQ(dedup.add({1.5f, 0, 0}), 1u);
  EXPECT_EQ(dedup.add({-1, 0, 0}), 0u);  // Inclusive tolerance.
}

TEST(MeshDeduplication, ExactModeAndInvalidInput) {
  MeshVertexDeduplicator dedup(0);
  EXPECT_EQ(dedup.add({0, 0, 0}), 0u);
  EXPECT_EQ(dedup.add({-0.0f, 0, 0}), 0u);
  EXPECT_EQ(dedup.add({1.0e-6f, 0, 0}), 1u);
  EXPECT_EQ(dedup.add({0, 0, 0}, 1), 2u);
  EXPECT_THROW(MeshVertexDeduplicator(-1), std::invalid_argument);
  EXPECT_THROW(MeshVertexDeduplicator(std::numeric_limits<double>::infinity()),
               std::invalid_argument);
  EXPECT_THROW(MeshVertexDeduplicator(std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
  EXPECT_THROW(dedup.add({std::numeric_limits<float>::quiet_NaN(), 0, 0}),
               std::invalid_argument);
  MeshVertexDeduplicator spatial(1.0e-5);
  EXPECT_THROW(spatial.add({1.0e30f, 0, 0}), std::out_of_range);
}

TEST(MeshDeduplication, CompactsAttributesAndFaces) {
  spark_dsg::Mesh mesh(true, true, true, true);
  mesh.resizeVertices(6);
  mesh.points = {
      {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1.0e-6f, 0, 0}, {1, 1, 0}, {0, 0, 0}};
  mesh.labels = {1, 1, 1, 1, 1, 2};
  mesh.stamps = {10, 10, 10, 20, 10, 10};
  mesh.first_seen_stamps = {5, 5, 5, 2, 5, 5};
  mesh.colors[0] = spark_dsg::Color(1, 2, 3);
  mesh.colors[3] = spark_dsg::Color(4, 5, 6);
  mesh.faces = {{0, 1, 2}, {3, 4, 2}, {0, 3, 1}, {5, 1, 2}};
  const auto original = mesh;
  deduplicateMesh(mesh, 0);
  EXPECT_EQ(mesh, original);
  deduplicateMesh(mesh, 1.0e-5);
  EXPECT_EQ(mesh.numVertices(), 5u);
  EXPECT_EQ(mesh.faces, (spark_dsg::Mesh::Faces{{0, 1, 2}, {0, 3, 2}, {4, 1, 2}}));
  EXPECT_EQ(mesh.stamps[0], 20u);
  EXPECT_EQ(mesh.first_seen_stamps[0], 2u);
  EXPECT_EQ(mesh.color(0), original.color(0));
  EXPECT_EQ(mesh.pos(0), original.pos(0));
  EXPECT_EQ(mesh.label(4), 2u);
}

TEST(MeshDeduplication, UnlabelledMeshesAndInvalidFaces) {
  spark_dsg::Mesh mesh(false, false, false, false);
  mesh.points = {{0, 0, 0}, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  mesh.faces = {{1, 2, 3}};
  deduplicateMesh(mesh, 1.0e-5);
  EXPECT_EQ(mesh.numVertices(), 3u);
  EXPECT_EQ(mesh.faces, (spark_dsg::Mesh::Faces{{0, 1, 2}}));
  EXPECT_TRUE(mesh.colors.empty());
  EXPECT_TRUE(mesh.labels.empty());
  EXPECT_TRUE(mesh.stamps.empty());
  mesh.faces.push_back({0, 1, 10});
  const auto original = mesh;
  EXPECT_THROW(deduplicateMesh(mesh, 1.0e-5), std::out_of_range);
  EXPECT_EQ(mesh, original);
}

}  // namespace hydra
