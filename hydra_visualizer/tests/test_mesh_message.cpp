#include <gtest/gtest.h>

#include "hydra_visualizer/drawing.h"

namespace hydra::visualizer {

TEST(MeshMessage, PreservesMetadataForNativeColoring) {
  spark_dsg::Mesh mesh(true, true, true, true);
  mesh.resizeVertices(1);
  mesh.setPos(0, Eigen::Vector3f(1, 2, 3));
  mesh.setColor(0, spark_dsg::Color(10, 20, 30));
  mesh.setLabel(0, 7);
  mesh.setTimestamp(0, 123);
  mesh.setFirstSeenTimestamp(0, 100);
  const auto msg = makeMeshMsg(std_msgs::msg::Header(), mesh, "mesh", nullptr);
  ASSERT_EQ(msg.vertices.size(), 1u);
  const auto& vertex = msg.vertices.front();
  EXPECT_TRUE(vertex.has_color);
  EXPECT_TRUE(vertex.has_label);
  EXPECT_TRUE(vertex.has_stamp);
  EXPECT_TRUE(vertex.has_first_seen_stamp);
  EXPECT_EQ(vertex.label, 7u);
  EXPECT_EQ(vertex.stamp, 123u);
  EXPECT_EQ(vertex.first_seen_stamp, 100u);
  EXPECT_FLOAT_EQ(vertex.color.r, 10.0f / 255.0f);
}

TEST(MeshMessage, MissingMetadataStaysAbsent) {
  spark_dsg::Mesh mesh(false, false, false, false);
  mesh.resizeVertices(1);
  const auto msg = makeMeshMsg(std_msgs::msg::Header(), mesh, "mesh", nullptr);
  ASSERT_EQ(msg.vertices.size(), 1u);
  EXPECT_FALSE(msg.vertices.front().has_label);
  EXPECT_FALSE(msg.vertices.front().has_stamp);
  EXPECT_FALSE(msg.vertices.front().has_first_seen_stamp);
}

}  // namespace hydra::visualizer
