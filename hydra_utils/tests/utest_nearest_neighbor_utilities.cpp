#include <gtest/gtest.h>

#include <kimera_dsg/scene_graph.h>
#include <kimera_topology/nearest_neighbor_utilities.h>

namespace kimera {
namespace topology {

TEST(NearestNeighborUtilities, TestBasicDynamic) {
  SceneGraph::Ptr graph(new SceneGraph());
  ASSERT_TRUE(graph->hasLayer(2));
  for (size_t i = 0; i < 10; ++i) {
    graph->emplaceNode(
        2, i, std::make_unique<NodeAttributes>(Eigen::Vector3d(i, 0.0, 0.0)));
  }

  DynamicNearestNodeFinder finder(graph, 2);

  std::unordered_set<NodeId> first_nodes{2, 4, 6, 8};
  finder.addNodes(first_nodes);

  { // test "left" side
    NodeId result_node = 10; // node doesn't exist
    double result_distance = -1.0;
    finder.find(
        Eigen::Vector3d(0.0, 0.0, 0.0), 1, false, [&](NodeId node, double distance) {
          result_node = node;
          result_distance = distance;
        });

    EXPECT_EQ(2u, result_node);
    EXPECT_NEAR(2.0, result_distance, 1.0e-7);
  }

  { // test "right" side
    NodeId result_node = 10; // node doesn't exist
    double result_distance = -1.0;
    finder.find(
        Eigen::Vector3d(9.0, 0.0, 0.0), 1, false, [&](NodeId node, double distance) {
          result_node = node;
          result_distance = distance;
        });

    EXPECT_EQ(8u, result_node);
    EXPECT_NEAR(1.0, result_distance, 1.0e-7);
  }
}

TEST(NearestNeighborUtilities, TestAddRemove) {
  SceneGraph::Ptr graph(new SceneGraph());
  ASSERT_TRUE(graph->hasLayer(2));
  for (size_t i = 0; i < 10; ++i) {
    graph->emplaceNode(
        2, i, std::make_unique<NodeAttributes>(Eigen::Vector3d(i, 0.0, 0.0)));
  }

  DynamicNearestNodeFinder finder(graph, 2);

  std::unordered_set<NodeId> first_nodes{2, 4, 6, 8};
  finder.addNodes(first_nodes);

  { // test "left" side
    NodeId result_node = 10; // node doesn't exist
    double result_distance = -1.0;
    finder.find(
        Eigen::Vector3d(0.0, 0.0, 0.0), 1, false, [&](NodeId node, double distance) {
          result_node = node;
          result_distance = distance;
        });

    EXPECT_EQ(2u, result_node);
    EXPECT_NEAR(2.0, result_distance, 1.0e-7);
  }

  finder.removeNode(2);

  { // get a different result
    NodeId result_node = 10; // node doesn't exist
    double result_distance = -1.0;
    finder.find(
        Eigen::Vector3d(0.0, 0.0, 0.0), 1, false, [&](NodeId node, double distance) {
          result_node = node;
          result_distance = distance;
        });

    EXPECT_EQ(4u, result_node);
    EXPECT_NEAR(4.0, result_distance, 1.0e-7);
  }

  std::unordered_set<NodeId> second_nodes{2, 3};
  finder.addNodes(second_nodes);

  { // get the same result again
    NodeId result_node = 10; // node doesn't exist
    double result_distance = -1.0;
    finder.find(
        Eigen::Vector3d(0.0, 0.0, 0.0), 1, false, [&](NodeId node, double distance) {
          result_node = node;
          result_distance = distance;
        });

    EXPECT_EQ(2u, result_node);
    EXPECT_NEAR(2.0, result_distance, 1.0e-7);
  }
}

}  // namespace topology
}  // namespace kimera
