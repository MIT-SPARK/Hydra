#include <gtest/gtest.h>
#include <kimera_pgmo/compression/block_compression.h>

#include <set>

#include "hydra/utils/pgmo_mesh_interface.h"

namespace hydra {
namespace {

void fillIndexedMesh(spark_dsg::Mesh& mesh, size_t vertices, float offset = 0.0f) {
  mesh.resizeVertices(vertices);
  mesh.points = {
      {offset, 0, 0}, {offset + 1, 0, 0}, {offset, 1, 0}, {offset + 1, 1, 0}};
  mesh.faces = {{0, 2, 1}, {1, 2, 3}};
  if (vertices == 6) {
    mesh.points.push_back({offset + 2, 0, 0});
    mesh.points.push_back({offset + 2, 1, 0});
    mesh.faces.push_back({1, 3, 4});
    mesh.faces.push_back({4, 3, 5});
  }
}

void checkCompression(const kimera_pgmo::MeshInterface& mesh,
                      const MeshLayer& expected) {
  kimera_pgmo::BlockCompression compression(0.01);
  pcl::PointCloud<pcl::PointXYZRGBA> vertices;
  std::vector<pcl::Vertices> faces;
  std::vector<size_t> indices;
  kimera_pgmo::HashedIndexMapping remapping;
  const auto cloned = mesh.clone();
  ASSERT_NO_THROW(compression.compressAndIntegrate(
      *cloned, vertices, faces, indices, remapping, 1.0));

  size_t num_vertices = 0;
  std::set<std::array<size_t, 3>> expected_faces;
  for (const auto& block : expected) {
    num_vertices += block.numVertices();
    ASSERT_EQ(remapping.count(block.index), 1u);
    const auto& mapping = remapping.at(block.index);
    ASSERT_EQ(mapping.size(), block.numVertices());
    for (size_t i = 0; i < block.numVertices(); ++i) {
      ASSERT_EQ(mapping.count(i), 1u);
      const auto& point = vertices.at(mapping.at(i));
      EXPECT_FLOAT_EQ(point.x, block.pos(i).x());
      EXPECT_FLOAT_EQ(point.y, block.pos(i).y());
      EXPECT_FLOAT_EQ(point.z, block.pos(i).z());
    }
    for (const auto& face : block.faces) {
      expected_faces.insert(
          {mapping.at(face[0]), mapping.at(face[1]), mapping.at(face[2])});
    }
  }
  EXPECT_EQ(vertices.size(), num_vertices);
  EXPECT_EQ(indices.size(), num_vertices);
  ASSERT_EQ(faces.size(), expected_faces.size());
  std::set<std::array<size_t, 3>> actual_faces;
  for (const auto& face : faces) {
    ASSERT_EQ(face.vertices.size(), 3u);
    actual_faces.insert({static_cast<size_t>(face.vertices[0]),
                         static_cast<size_t>(face.vertices[1]),
                         static_cast<size_t>(face.vertices[2])});
  }
  EXPECT_EQ(actual_faces, expected_faces);
}

}  // namespace

TEST(PgmoMeshInterface, IndexedLayerCompression) {
  for (const size_t vertices : {4, 6}) {
    SCOPED_TRACE(vertices);
    MeshLayer layer(1.0f);
    fillIndexedMesh(layer.allocateBlock(BlockIndex(-1, 0, 0)), vertices);
    fillIndexedMesh(layer.allocateBlock(BlockIndex(1, 0, 0)), vertices, 10.0f);
    layer.allocateBlock(BlockIndex(0, 0, 0));
    checkCompression(PgmoMeshLayerInterface(layer), layer);
  }
}

TEST(PgmoMeshInterface, IndexedMeshCompression) {
  for (const size_t vertices : {4, 6}) {
    SCOPED_TRACE(vertices);
    MeshLayer layer(1.0f);
    auto& mesh = layer.allocateBlock(BlockIndex::Zero());
    fillIndexedMesh(mesh, vertices);
    checkCompression(PgmoMeshInterface(mesh), layer);
  }
}

TEST(PgmoMeshInterface, VerticesWithoutFacesDoNotCreateTriangles) {
  MeshLayer layer(1.0f);
  auto& mesh = layer.allocateBlock(BlockIndex::Zero());
  mesh.resizeVertices(3);
  mesh.points = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  const std::array<kimera_pgmo::MeshInterface::Ptr, 2> interfaces{
      std::make_shared<PgmoMeshLayerInterface>(layer),
      std::make_shared<PgmoMeshInterface>(mesh)};
  for (const auto& interface : interfaces) {
    kimera_pgmo::BlockCompression compression(0.01);
    pcl::PointCloud<pcl::PointXYZRGBA> vertices;
    std::vector<pcl::Vertices> faces;
    std::vector<size_t> indices;
    kimera_pgmo::HashedIndexMapping remapping;
    compression.compressAndIntegrate(
        *interface, vertices, faces, indices, remapping, 1.0);
    EXPECT_TRUE(vertices.empty());
    EXPECT_TRUE(faces.empty());
    EXPECT_TRUE(indices.empty());
  }
}

}  // namespace hydra
