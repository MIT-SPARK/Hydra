#include <hydra_dsg_builder/minimum_spanning_tree.h>

#include <gtest/gtest.h>

namespace hydra {

TEST(MinimumSpanningTreeTests, TestSingleChain) {
  IsolatedSceneGraphLayer layer(1);
  layer.emplaceNode(0,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(0.0, 0.0, 0.0)));
  layer.emplaceNode(1,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(1.0, 0.0, 0.0)));
  layer.emplaceNode(2,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(3.0, 0.0, 0.0)));
  layer.emplaceNode(3,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(6.0, 0.0, 0.0)));
  layer.insertEdge(0, 1);
  layer.insertEdge(1, 2);
  layer.insertEdge(2, 3);

  MinimumSpanningTreeInfo info = getMinimumSpanningEdges(layer);
  EXPECT_TRUE(info.leaves.count(0));
  EXPECT_TRUE(info.leaves.count(3));
  EXPECT_EQ(2u, info.leaves.size());
  ASSERT_EQ(3u, info.edges.size());
  EXPECT_EQ(0u, info.edges[0].source);
  EXPECT_EQ(1u, info.edges[0].target);
  EXPECT_EQ(1u, info.edges[1].source);
  EXPECT_EQ(2u, info.edges[1].target);
  EXPECT_EQ(2u, info.edges[2].source);
  EXPECT_EQ(3u, info.edges[2].target);
}

TEST(MinimumSpanningTreeTests, TestCompleteGraph) {
  IsolatedSceneGraphLayer layer(1);
  layer.emplaceNode(0,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(0.0, 0.0, 0.0)));
  layer.emplaceNode(1,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(1.0, 0.0, 0.0)));
  layer.emplaceNode(2,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(3.0, 0.0, 0.0)));
  layer.emplaceNode(3,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(6.0, 0.0, 0.0)));
  layer.insertEdge(0, 1);
  layer.insertEdge(0, 2);
  layer.insertEdge(0, 3);
  layer.insertEdge(1, 2);
  layer.insertEdge(1, 3);
  layer.insertEdge(2, 3);

  MinimumSpanningTreeInfo info = getMinimumSpanningEdges(layer);
  EXPECT_TRUE(info.leaves.count(0));
  EXPECT_TRUE(info.leaves.count(3));
  EXPECT_EQ(2u, info.leaves.size());
  ASSERT_EQ(3u, info.edges.size());
  EXPECT_EQ(0u, info.edges[0].source);
  EXPECT_EQ(1u, info.edges[0].target);
  EXPECT_EQ(1u, info.edges[1].source);
  EXPECT_EQ(2u, info.edges[1].target);
  EXPECT_EQ(2u, info.edges[2].source);
  EXPECT_EQ(3u, info.edges[2].target);
}

}  // namespace hydra
