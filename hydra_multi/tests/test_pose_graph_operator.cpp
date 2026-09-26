#include <gtest/gtest.h>
#include <hydra_multi/operators/pose_graph_operator.h>

namespace hydra_multi {
using PoseGraph = pose_graph_tools::PoseGraph;
using PoseGraphNode = pose_graph_tools::PoseGraphNode;
using PoseGraphEdge = pose_graph_tools::PoseGraphEdge;

PoseGraph makePoseGraph(
    size_t num_nodes,
    Eigen::Matrix4d initial_transform = Eigen::Matrix4d::Identity(4, 4),
    Eigen::Vector3d trans_increment = Eigen::Vector3d::Ones(3)) {
  PoseGraph pose_graph;

  auto curr_node_transform = initial_transform;
  for (size_t node_id = 0; node_id < num_nodes; node_id++) {
    if (node_id > 0) {
      // Add edge
      PoseGraphEdge pg_edge;
      pg_edge.key_from = node_id - 1;
      pg_edge.key_to = node_id;

      Eigen::Matrix4d edge_tf = Eigen::Matrix4d::Identity(4, 4);
      edge_tf.block<3, 1>(0, 3) = trans_increment;

      curr_node_transform *= edge_tf;

      pg_edge.pose = Eigen::Affine3d(edge_tf);
      pg_edge.type = PoseGraphEdge::Type::ODOM;
      pose_graph.edges.push_back(pg_edge);
    }

    PoseGraphNode pg_node;
    pg_node.key = node_id;
    pg_node.pose = Eigen::Affine3d(curr_node_transform);
    pose_graph.nodes.push_back(pg_node);
  }

  return pose_graph;
}

TEST(PoseGraphOperatorTests, IncrementalAppend) {
  auto data = std::make_shared<PoseGraph>();
  PoseGraphOperator pose_graph_operator(data);

  Eigen::Matrix4d node_0_pose;
  node_0_pose << -1, 0, 0, 1, 0, -1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1;
  PoseGraph pg_1 = makePoseGraph(2, node_0_pose);
  EXPECT_TRUE(pose_graph_operator.incrementalAppend(pg_1));

  // To append
  PoseGraph pg_2;
  // Pose graph node
  PoseGraphNode pg_node_2;
  pg_node_2.robot_id = 0;
  pg_node_2.key = 2;
  pg_2.nodes.push_back(pg_node_2);
  // Pose graph edge
  PoseGraphEdge pg_edge_12;
  pg_edge_12.key_from = 1;
  pg_edge_12.key_to = 2;
  pg_edge_12.robot_from = 0;
  pg_edge_12.robot_to = 0;
  pg_edge_12.type = PoseGraphEdge::Type::ODOM;
  Eigen::Matrix4d edge_12_tf = Eigen::Matrix4d::Identity(4, 4);
  pg_edge_12.pose = Eigen::Affine3d(edge_12_tf);
  pg_2.edges.push_back(pg_edge_12);

  EXPECT_TRUE(pose_graph_operator.incrementalAppend(pg_2));

  EXPECT_EQ(3, data->nodes.size());
  EXPECT_EQ(2, data->edges.size());

  // Expect that pg_node is correctly appended on top of pg_1, and node 2's position
  // correctly updated
  Eigen::Matrix4d node_2_pose;
  node_2_pose << -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 2, 0, 0, 0, 1;
  EXPECT_TRUE(data->nodes[2].pose.matrix().isApprox(node_2_pose));

  // To append false
  PoseGraph pg_3;
  // Pose graph node
  PoseGraphNode pg_node_5;
  pg_node_5.robot_id = 0;
  pg_node_5.key = 5;
  pg_3.nodes.push_back(pg_node_5);
  // Pose graph edge
  PoseGraphEdge pg_edge_45;
  pg_edge_45.key_from = 4;
  pg_edge_45.key_to = 5;
  pg_edge_45.robot_from = 0;
  pg_edge_45.robot_to = 0;
  pg_edge_45.type = PoseGraphEdge::Type::ODOM;
  Eigen::Matrix4d edge_45_tf = Eigen::Matrix4d::Identity(4, 4);
  pg_edge_45.pose = Eigen::Affine3d(edge_45_tf);
  pg_3.edges.push_back(pg_edge_45);
  EXPECT_FALSE(pose_graph_operator.incrementalAppend(pg_3));
}

TEST(PoseGraphOperatorTests, Merge) {
  auto data = std::make_shared<PoseGraph>();
  PoseGraphOperator pose_graph_operator(data);

  Eigen::Matrix4d node_0_pose;
  node_0_pose << -1, 0, 0, 1, 0, -1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1;
  // Make target pose graph
  PoseGraph pg_1 = makePoseGraph(2, node_0_pose);
  pose_graph_operator.incrementalAppend(pg_1);

  // Make source pose graph (to merge)
  PoseGraph pg_2 = makePoseGraph(3);
  EXPECT_TRUE(pose_graph_operator(pg_2, OperationType::MERGE));

  EXPECT_EQ(3, data->nodes.size());
  EXPECT_EQ(2, data->edges.size());
  // Expect that pg_2 is correctly merged to pg_1 and node 2 is correctly updated
  Eigen::Matrix4d node_2_pose;
  node_2_pose << -1, 0, 0, -1, 0, -1, 0, -1, 0, 0, 1, 3, 0, 0, 0, 1;
  EXPECT_TRUE(data->nodes[2].pose.matrix().isApprox(node_2_pose));
}

TEST(PoseGraphOperatorTests, Update) {
  auto data = std::make_shared<PoseGraph>();
  PoseGraphOperator pose_graph_operator(data);

  // Make target pose graph
  PoseGraph pg_1 = makePoseGraph(2);
  pose_graph_operator.incrementalAppend(pg_1);

  // Make source pose graph (to merge)
  Eigen::Matrix4d node_0_pose;
  node_0_pose << -1, 0, 0, 1, 0, -1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1;
  PoseGraph pg_2 = makePoseGraph(4, node_0_pose);
  EXPECT_TRUE(pose_graph_operator(pg_2, OperationType::UPDATE));

  EXPECT_EQ(2, data->nodes.size());
  EXPECT_EQ(1, data->edges.size());
  // Expect that pg_1 is now the same as pg_2 (where they overlap)
  EXPECT_TRUE(data->nodes[0].pose.matrix().isApprox(node_0_pose));
  Eigen::Matrix4d node_1_pose;
  node_1_pose << -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 2, 0, 0, 0, 1;
  EXPECT_TRUE(data->nodes[1].pose.matrix().isApprox(node_1_pose));

  // Make source pose graph that is too small
  PoseGraph pg_3 = makePoseGraph(1);
  EXPECT_FALSE(pose_graph_operator(pg_3, OperationType::UPDATE));
}

TEST(PoseGraphOperatorTests, Rebase) {
  auto data = std::make_shared<PoseGraph>();
  PoseGraphOperator pose_graph_operator(data);

  // Make target pose graph
  PoseGraph pg_1 = makePoseGraph(4);
  pose_graph_operator.incrementalAppend(pg_1);

  // Make source pose graph (to merge)
  Eigen::Matrix4d node_0_pose;
  node_0_pose << -1, 0, 0, 1, 0, -1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1;
  PoseGraph pg_2 = makePoseGraph(2, node_0_pose);
  EXPECT_TRUE(pose_graph_operator(pg_2, OperationType::REBASE));

  EXPECT_EQ(4, data->nodes.size());
  EXPECT_EQ(3, data->edges.size());
  // Expect that pg_1 is now the same as pg_2 where they overlap
  EXPECT_TRUE(data->nodes[0].pose.matrix().isApprox(node_0_pose));
  Eigen::Matrix4d node_1_pose;
  node_1_pose << -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 2, 0, 0, 0, 1;
  EXPECT_TRUE(data->nodes[1].pose.matrix().isApprox(node_1_pose));

  // Expect that non-overlapping part of pg_1 is correctly updated
  Eigen::Matrix4d node_3_pose;
  node_3_pose << -1, 0, 0, -2, 0, -1, 0, -2, 0, 0, 1, 4, 0, 0, 0, 1;
  EXPECT_TRUE(data->nodes[3].pose.matrix().isApprox(node_3_pose));

  // Try append after rebase
  PoseGraph pg_3;
  // Pose graph node
  PoseGraphNode pg_node_4;
  pg_node_4.robot_id = 0;
  pg_node_4.key = 4;
  pg_3.nodes.push_back(pg_node_4);
  // Pose graph edge
  PoseGraphEdge pg_edge_34;
  pg_edge_34.key_from = 3;
  pg_edge_34.key_to = 4;
  pg_edge_34.robot_from = 0;
  pg_edge_34.robot_to = 0;
  pg_edge_34.type = PoseGraphEdge::Type::ODOM;
  Eigen::Matrix4d edge_34_tf = Eigen::Matrix4d::Identity(4, 4);
  pg_edge_34.pose = Eigen::Affine3d(edge_34_tf);
  pg_3.edges.push_back(pg_edge_34);

  EXPECT_TRUE(pose_graph_operator.incrementalAppend(pg_3));

  EXPECT_EQ(5, data->nodes.size());
  EXPECT_EQ(4, data->edges.size());

  // Expect that pg_node is correctly appended on top of pg_1, and node 2's position
  // correctly updated
  Eigen::Matrix4d node_4_pose;
  node_4_pose << -1, 0, 0, -2, 0, -1, 0, -2, 0, 0, 1, 4, 0, 0, 0, 1;
  EXPECT_TRUE(data->nodes[4].pose.matrix().isApprox(node_4_pose));
}
}  // namespace hydra_multi
