#include <gtest/gtest.h>
#include <hydra_multi/operators/mesh_operator.h>
#include <kimera_pgmo/mesh_delta.h>
#include <kimera_pgmo/mesh_types.h>

#include "hydra_multi/common/types.h"

namespace hydra_multi {
namespace {

static const Eigen::IOFormat fmt(3, Eigen::DontAlignCols, ", ", "; ", "", "", "[", "]");

struct Vertex {
  float x;
  float y;
  float z;

  operator pcl::PointXYZRGBA() const {
    pcl::PointXYZRGBA point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  }
};

inline MeshData makeMesh(size_t num_vertices,
                         Eigen::Vector3f offset = Eigen::Vector3f::Zero(),
                         Eigen::Vector3f original_offset = Eigen::Vector3f::Zero()) {
  MeshData mesh_data;
  mesh_data.mesh.reset(new Mesh(true, true, false));
  mesh_data.original_vertices.reset(new pcl::PointCloud<pcl::PointXYZ>);
  mesh_data.vertex_stamps.reset(new Timestamps);
  mesh_data.mesh->resizeVertices(num_vertices);
  for (size_t i = 0; i < mesh_data.mesh->numVertices(); ++i) {
    Eigen::Vector3f pos;
    if (i % 3 == 0) {
      pos << static_cast<float>(i), static_cast<float>(i), 0.0;
    } else {
      pos << static_cast<float>(i), static_cast<float>(i), 1.0;
    }
    mesh_data.mesh->setPos(i, pos + offset);
    mesh_data.mesh->setTimestamp(i, i);
    auto original_pos = pos + original_offset;
    mesh_data.original_vertices->push_back(
        {original_pos(0), original_pos(1), original_pos(2)});
    mesh_data.vertex_stamps->push_back(i);
  }
  for (size_t i = 0; i < mesh_data.mesh->numVertices(); i += 3) {
    mesh_data.mesh->faces.push_back({i, i + 1, i + 2});
  }
  return mesh_data;
}

inline MeshDelta makeMeshDelta(size_t prev_active_vertices,
                               size_t prev_active_faces,
                               size_t num_vertices,
                               Eigen::Vector3f offset = Eigen::Vector3f::Zero()) {
  // seq, pre
  MeshDelta delta({0, prev_active_vertices, prev_active_faces});
  for (size_t i = 0; i < num_vertices; ++i) {
    Eigen::Vector3f pos;
    if (i % 3 == 0) {
      pos << static_cast<float>(prev_active_vertices + i),
          static_cast<float>(prev_active_vertices + i), 0.0;
    } else {
      pos << static_cast<float>(prev_active_vertices + i),
          static_cast<float>(prev_active_vertices + i), 1.0;
    }
    pos += offset;
    kimera_pgmo::traits::VertexTraits traits;
    traits.properties.has_stamp = true;
    traits.stamp = i + prev_active_vertices;
    delta.addVertex(pos, traits);
  }

  for (size_t i = 0; i < num_vertices; i += 3) {
    delta.addFace({i, i + 1, i + 2});
  }

  return delta;
}

}  // namespace

TEST(MeshOperatorTests, IncrementalAppend) {
  auto data = std::make_shared<MeshData>();
  MeshOperator mesh_operator(data);

  auto delta1 = makeMeshDelta(0, 0, 9);

  EXPECT_TRUE(mesh_operator.incrementalAppend(delta1));
  EXPECT_EQ(9, data->mesh->numVertices());
  Eigen::Vector3f vertex0_pos = Eigen::Vector3f::Zero();
  EXPECT_TRUE(data->mesh->pos(0).isApprox(vertex0_pos));

  // Update mesh
  auto mesh1_mod = makeMesh(9, {1, 2, 3});
  EXPECT_TRUE(mesh_operator(mesh1_mod, OperationType::UPDATE));
  // append transform should be updated

  auto delta2 = makeMeshDelta(0, 0, 3, Eigen::Vector3f(9.0, 9.0, 0.0));
  EXPECT_TRUE(mesh_operator.incrementalAppend(delta2));

  ASSERT_EQ(12, data->mesh->numVertices());
  EXPECT_EQ(4, data->mesh->numFaces());
  Eigen::Vector3f vertex11_pos;
  vertex11_pos << 12, 13, 4;
  EXPECT_TRUE(data->mesh->pos(11).isApprox(vertex11_pos))
      << "result: " << data->mesh->pos(11).format(fmt)
      << ", expected: " << vertex11_pos.format(fmt);
}

TEST(MeshOperatorTests, Merge) {
  auto data = std::make_shared<MeshData>();
  MeshOperator mesh_operator(data);

  auto delta1 = makeMeshDelta(0, 0, 9);

  EXPECT_TRUE(mesh_operator.incrementalAppend(delta1));
  EXPECT_EQ(9, data->mesh->numVertices());
  Eigen::Vector3f vertex0_pos = Eigen::Vector3f::Zero();
  EXPECT_TRUE(data->mesh->pos(0).isApprox(vertex0_pos));

  // Update mesh
  auto mesh1_mod = makeMesh(9, {1, 2, 3});
  EXPECT_TRUE(mesh_operator(mesh1_mod, OperationType::UPDATE));
  // append transform should be updated

  // Create new mesh to merge
  auto mesh2 = makeMesh(12);
  EXPECT_TRUE(mesh_operator(mesh2, OperationType::MERGE));

  EXPECT_EQ(12, data->mesh->numVertices());
  EXPECT_EQ(4, data->mesh->numFaces());
  Eigen::Vector3f vertex11_pos;
  vertex11_pos << 12, 13, 4;
  EXPECT_TRUE(data->mesh->pos(11).isApprox(vertex11_pos));
}

TEST(MeshOperatorTests, Update) {
  auto data = std::make_shared<MeshData>();
  MeshOperator mesh_operator(data);

  auto delta1 = makeMeshDelta(0, 0, 9);

  EXPECT_TRUE(mesh_operator.incrementalAppend(delta1));
  EXPECT_EQ(9, data->mesh->numVertices());
  Eigen::Vector3f vertex0_pos = Eigen::Vector3f::Zero();
  EXPECT_TRUE(data->mesh->pos(0).isApprox(vertex0_pos));

  // Update mesh
  auto mesh2 = makeMesh(9, {1, 2, 3});
  EXPECT_TRUE(mesh_operator(mesh2, OperationType::UPDATE));
  // append transform should be updated

  EXPECT_EQ(9, data->mesh->numVertices());
  EXPECT_EQ(3, data->mesh->numFaces());
  Eigen::Vector3f vertex8_pos;
  vertex8_pos << 9, 10, 4;
  EXPECT_TRUE(data->mesh->pos(8).isApprox(vertex8_pos));

  // Update mesh
  auto mesh3 = makeMesh(3);
  EXPECT_FALSE(mesh_operator(mesh3, OperationType::UPDATE));
}

TEST(MeshOperatorTests, Rebase) {
  auto data = std::make_shared<MeshData>();
  MeshOperator mesh_operator(data);

  auto delta1 = makeMeshDelta(0, 0, 12);

  EXPECT_TRUE(mesh_operator.incrementalAppend(delta1));
  EXPECT_EQ(12, data->mesh->numVertices());
  Eigen::Vector3f vertex0_pos;
  vertex0_pos << 0, 0, 0;
  EXPECT_TRUE(data->mesh->pos(0).isApprox(vertex0_pos));

  // Create new mesh to rebase
  auto mesh2 = makeMesh(9, {1, 2, 3});
  EXPECT_TRUE(mesh_operator(mesh2, OperationType::REBASE));

  EXPECT_EQ(12, data->mesh->numVertices());
  EXPECT_EQ(4, data->mesh->numFaces());
  Eigen::Vector3f vertex8_pos;
  vertex8_pos << 9, 10, 4;
  EXPECT_TRUE(data->mesh->pos(8).isApprox(vertex8_pos));

  // Try append on top
  auto delta2 = makeMeshDelta(0, 0, 3, Eigen::Vector3f(12.0, 12.0, 0.0));
  EXPECT_TRUE(mesh_operator.incrementalAppend(delta2));
  EXPECT_EQ(15, data->mesh->numVertices());
  EXPECT_EQ(5, data->mesh->numFaces());
  Eigen::Vector3f vertex14_pos;
  vertex14_pos << 15, 16, 4;
  EXPECT_TRUE(data->mesh->pos(14).isApprox(vertex14_pos));
}

}  // namespace hydra_multi
