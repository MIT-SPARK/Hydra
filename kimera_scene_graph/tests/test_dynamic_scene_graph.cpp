#include "kimera_scene_graph/dynamic_scene_graph.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <std_srvs/Empty.h>

namespace kimera {

namespace fs = boost::filesystem;

// Keep node handles persistent.
class DynamicHumanNodeTestFixture : public ::testing::Test {
 public:
  DynamicHumanNodeTestFixture() {
    // TODO(nathan) should remove explicit tmp when we can use std::filesystem
    temporary_dir = fs::path("/tmp") / "humans_test_serialization";
    nh_private.setParam("serialization_dir", temporary_dir.string());
  }

  ~DynamicHumanNodeTestFixture() {
    if (fs::exists(temporary_dir)) {
      fs::remove_all(temporary_dir);
    }
  }

 protected:
  void SetUp() override {}
  void TearDown() override {}

 protected:
  ros::NodeHandle nh;
  ros::NodeHandle nh_private;
  fs::path temporary_dir;
};

/*
 * Test the isPoseDynamicallyFeasible method
 */
TEST_F(DynamicHumanNodeTestFixture, testIsPoseDynamicallyFeasible) {
  LOG(INFO) << "Running testIsPoseDynamicallyFeasible";
  kimera::DynamicSceneGraph d_graph(nh, nh_private);
  // Check far apart fails.
  kimera::DynamicHumanNode node1, node2;
  node1.attributes_.position_ = kimera::NodePosition(0, 0, 0);
  node2.attributes_.position_ = kimera::NodePosition(5, 5, 5);

  // pcl headers are in microseconds
  unsigned long inv_micro = pow(10, 6);
  node1.attributes_.timestamp_ = 0 * inv_micro;
  node2.attributes_.timestamp_ = 1 * inv_micro;
  EXPECT_FALSE(d_graph.isPoseDynamicallyFeasible(node2, node1));

  // Check with increased time, passes.
  node1.attributes_.timestamp_ = 1 * inv_micro;
  node2.attributes_.timestamp_ = 8 * inv_micro;
  EXPECT_TRUE(d_graph.isPoseDynamicallyFeasible(node2, node1));

  // Check very edge passes.
  node1.attributes_.position_ = kimera::NodePosition(5, 5, 0);
  node1.attributes_.timestamp_ = 0;
  node2.attributes_.timestamp_ =
      5 * inv_micro / d_graph.feasible_dyn_rate_ + 1000;
  EXPECT_TRUE(d_graph.isPoseDynamicallyFeasible(node2, node1));

  // Check that same point passes.
  node1.attributes_.position_ = kimera::NodePosition(5, 5, 5);
  EXPECT_TRUE(d_graph.isPoseDynamicallyFeasible(node2, node1));
}

/*
 * Test the isMeshDynamicallyFeasible method
 */
TEST_F(DynamicHumanNodeTestFixture, testIsMeshDynamicallyFeasible) {
  LOG(INFO) << "Running testIsMeshDynamicallyFeasible";
  kimera::DynamicSceneGraph d_graph(nh, nh_private);
  kimera::DynamicHumanNode node1, node2;
  std::vector<double> test_joints;
  for (size_t i = 0; i < kimera::NUM_JOINTS * 3; i++) {
    test_joints.push_back(i);
  }
  // Check same joints pass.
  node1.joints_ = Eigen::Map<kimera::JointMatrix>(&test_joints[0]);
  node2.joints_ = Eigen::Map<kimera::JointMatrix>(&test_joints[0]);

  // pcl headers are in microseconds
  unsigned long inv_micro = pow(10, 6);
  node1.attributes_.timestamp_ = 0 * inv_micro;
  node2.attributes_.timestamp_ = 1 * inv_micro;
  EXPECT_TRUE(d_graph.isMeshDynamicallyFeasible(node2, node1));

  // Check far away joints fail
  std::vector<double> break_joints;
  for (size_t i = 0; i < kimera::NUM_JOINTS * 3; i++) {
    break_joints.push_back(2 * i);
  }

  node2.joints_ = Eigen::Map<kimera::JointMatrix>(&break_joints[0]);
  EXPECT_FALSE(d_graph.isMeshDynamicallyFeasible(node2, node1));

  // Check just one far away joint causes failure
  std::vector<double> test_joints_copy(test_joints);
  size_t last_idx = test_joints.size() - 1;
  test_joints_copy[last_idx] = test_joints[last_idx] * 2;
  test_joints_copy[last_idx - 1] = test_joints[last_idx - 1] * 2;
  test_joints_copy[last_idx - 2] = test_joints[last_idx - 2] * 2;

  node2.joints_ = Eigen::Map<kimera::JointMatrix>(&test_joints_copy[0]);
  EXPECT_FALSE(d_graph.isMeshDynamicallyFeasible(node2, node1));

  // Check that all joints being slightly off passes.
  std::vector<double> off_test_joints;
  for (auto value : test_joints) {
    off_test_joints.push_back(value + 0.2);
  }
  node2.joints_ = Eigen::Map<kimera::JointMatrix>(&off_test_joints[0]);
  EXPECT_TRUE(d_graph.isMeshDynamicallyFeasible(node2, node1));
}

TEST_F(DynamicHumanNodeTestFixture, testIsShapeFeasible) {
  LOG(INFO) << "Running testIsShapeFeasible";
  kimera::DynamicSceneGraph d_graph(nh, nh_private);
  kimera::DynamicHumanNode node1, node2;

  std::vector<double> test_betas;
  for (size_t i = 0u; i < kimera::NUM_BETAS; i++) {
    test_betas.push_back(i);
  }

  // Identical betas pass the test
  node1.betas_ = test_betas;
  node2.betas_ = test_betas;
  EXPECT_TRUE(d_graph.isShapeFeasible(node2, node1));

  std::vector<double> break_betas;
  for (size_t i = 0u; i < kimera::NUM_BETAS; i++) {
    break_betas.push_back(i * 10);
  }

  // If all betas are far away the test fails
  node2.betas_ = break_betas;
  EXPECT_FALSE(d_graph.isShapeFeasible(node2, node1));
}

/*
 * Check that the SMPLList msg is converted properly.
 */
TEST_F(DynamicHumanNodeTestFixture, testSceneNodeFromSMPL) {
  LOG(INFO) << "Running testSceneNodeFromSMPL";
  kimera::DynamicSceneGraph d_graph(nh, nh_private);

  std::vector<double> test_joints;
  for (size_t i = 0; i < kimera::NUM_JOINTS * 3; i++) {
    test_joints.push_back(i);
  }
  graph_cmr_ros::SMPL smpl;
  smpl.header.stamp = ros::Time();
  smpl.centroid = {1, 2, 3};
  for (size_t i = 0; i < kimera::NUM_BETAS; i++) {
    smpl.betas.push_back(i);
  }
  smpl.orientation = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  smpl.joints = test_joints;

  kimera::DynamicHumanNode test_node;
  DynamicSceneGraph::DynamicHumanNodeFromSMPL(
      smpl, d_graph.human_color_, &test_node);
  auto& center = test_node.pose_.translation();
  EXPECT_EQ(smpl.centroid[0], center[0]);
  EXPECT_EQ(smpl.centroid[1], center[1]);
  EXPECT_EQ(smpl.centroid[2], center[2]);
}

/*
 * Check that if we send one human, we get the correct number of graphs and
 * connectivity.
 */
TEST_F(DynamicHumanNodeTestFixture, testGraphConnection) {
  LOG(INFO) << "Running testGraphConnection";
  kimera::DynamicSceneGraph d_graph(nh, nh_private);
  d_graph.feasible_dyn_rate_ = 2.0;
  d_graph.feasible_mesh_rate_ = 1.5;
  d_graph.time_cap_ = 10.0;

  boost::shared_ptr<graph_cmr_ros::SMPLList> msg(new graph_cmr_ros::SMPLList);
  std::vector<double> test_joints;
  for (size_t i = 0; i < kimera::NUM_JOINTS * 3; i++) {
    test_joints.push_back(i);
  }
  graph_cmr_ros::SMPL smpl;
  float base_time = 0.1;
  smpl.header.stamp = ros::Time(base_time);
  smpl.centroid = {0, 1, 1};
  for (size_t i = 0; i < kimera::NUM_BETAS; i++) {
    smpl.betas.push_back(i);
  }
  smpl.orientation = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  smpl.joints = test_joints;

  msg->human_meshes.push_back(smpl);

  // Append the same node several times over some number of timesteps, and
  // ensure everything stays the same.
  for (size_t i = 0; i < 2; i++) {
    d_graph.humanCallback(msg);

    // Ensure that there are the correct number of graphs.
    EXPECT_EQ(d_graph.human_db_.size(), 1u)
        << "STILL -- too many pose graphs -- iter: " << i;

    // Ensure that the optimized poses are all the same for one human.
    DynamicHumanNodeList opt_nodes;
    d_graph.getAllOptimizedHumanNodes(&opt_nodes);
    EXPECT_EQ(opt_nodes.size(), 1u)
        << "STILL -- too many humans -- iter: " << i;
    EXPECT_EQ(opt_nodes[0].size(), i + 1)
        << "STILL -- too many poses -- iter: " << i;
    EXPECT_EQ(opt_nodes[0].front().pose_.translation(),
              opt_nodes[0].back().pose_.translation())
        << "STILL -- poses aren't consistent -- iter: " << i;

    // Increment time to ensure that the checks pass.
    base_time += 2.0;
    msg->human_meshes[0].header.stamp = ros::Time(base_time);
  }

  // Append a slightly offset centroid node over several timesteps, ensure
  // things change accordingly.
  for (size_t i = 2; i < 4; i++) {
    msg->human_meshes[0].centroid[1] += 1.0;
    msg->human_meshes[0].centroid[2] += 1.0;
    base_time += 2.0;
    msg->human_meshes[0].header.stamp = ros::Time(base_time);
    d_graph.humanCallback(msg);

    // Ensure that there are the correct number of graphs.
    EXPECT_EQ(d_graph.human_db_.size(), 1)
        << "MOVING -- too many pose graphs -- iter: " << i;
    EXPECT_EQ(d_graph.human_db_[0].human_nodes_.size(), i + 1)
        << "MOVING -- too many poses -- iter: " << i;

    // Ensure that the unoptimized poses for the one human are moving properly.
    EXPECT_LT(
        (d_graph.human_db_[0].human_nodes_[i].pose_.translation().norm() -
         d_graph.human_db_[0].human_nodes_[i - 1].pose_.translation().norm()) -
            std::pow(2.0, 0.5),
        1e-6)  // sqrt(2)
        << "MOVING -- poses aren't consistent -- iter: " << i;
  }

  // Add a drastic jump to ensure that a new graph is added.
  for (size_t i = 4; i < 6; i++) {
    msg->human_meshes[0].centroid[1] += 12.0;
    msg->human_meshes[0].centroid[2] += 11.0;

    base_time += 1.0;
    msg->human_meshes[0].header.stamp = ros::Time(base_time);
    d_graph.humanCallback(msg);

    // Ensure that there are the correct number of graphs.
    EXPECT_EQ(d_graph.human_db_.size(), i - 2)
        << "MULTI -- too many pose graphs -- iter: " << i;
  }
}

/*
 * Verify round-trip serialization / deserialization for one human
 */
TEST_F(DynamicHumanNodeTestFixture, testHumansSerialization) {
  LOG(INFO) << "Running testGraphConnection";
  nh_private.setParam("visualize_joints", true);
  nh_private.setParam("visualize_pose_graphs", true);

  kimera::DynamicSceneGraph d_graph(nh, nh_private);
  d_graph.feasible_dyn_rate_ = 2.0;
  d_graph.feasible_mesh_rate_ = 1.5;
  d_graph.time_cap_ = 10.0;

  boost::shared_ptr<graph_cmr_ros::SMPLList> msg(new graph_cmr_ros::SMPLList);
  std::vector<double> test_joints;
  for (size_t i = 0; i < kimera::NUM_JOINTS * 3; i++) {
    test_joints.push_back(i);
  }
  graph_cmr_ros::SMPL smpl;
  float base_time = 0.1;
  smpl.header.stamp = ros::Time(base_time);
  smpl.centroid = {0, 1, 1};
  for (size_t i = 0; i < kimera::NUM_BETAS; i++) {
    smpl.betas.push_back(i);
  }
  smpl.orientation = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  smpl.joints = test_joints;

  msg->human_meshes.push_back(smpl);

  for (size_t i = 0; i < 2; i++) {
    d_graph.humanCallback(msg);

    base_time += 2.0;
    msg->human_meshes[0].header.stamp = ros::Time(base_time);
  }

  EXPECT_EQ(d_graph.human_db_.size(), 1u) << "too many pose graphs";
  std_srvs::Empty::Request serialization_req;
  std_srvs::Empty::Response serialization_res;
  EXPECT_TRUE(
      d_graph.serializeServiceCall(serialization_req, serialization_res));
  EXPECT_TRUE(fs::exists(temporary_dir / "humans_1.bag"));

  kimera::DynamicSceneGraph other_dgraph(nh, nh_private);
  d_graph.feasible_dyn_rate_ = 2.0;
  d_graph.feasible_mesh_rate_ = 1.5;
  d_graph.time_cap_ = 10.0;

  voxblox_msgs::FilePath::Request deserialization_req;
  deserialization_req.file_path = (temporary_dir / "humans_1.bag").string();
  voxblox_msgs::FilePath::Response deserialization_res;
  EXPECT_TRUE(other_dgraph.deserializeServiceCall(deserialization_req,
                                                  deserialization_res));
  // TODO(nathan) maybe add more thorough checks here
}

// TODO(marcus): add test that fails if you don't have the correct reference
// frame at world pose (pose0 must be identity)
// TODO(marcus): add similar tests that have consecutive detections and the pgo
// looks right.

}  // namespace kimera
