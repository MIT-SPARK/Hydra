#include "kimera_scene_graph/scene_graph_server.h"
#include <gtest/gtest.h>

// Keep node handles persistent.
class DynamicSceneNodeTest : public testing::Test
{
    public:
        DynamicSceneNodeTest(){}
        ~DynamicSceneNodeTest(){}
    
    protected:
        ros::NodeHandle nh;
        ros::NodeHandle nh_private;
};

/*
* Test the CheckDynamicFeasibility method
*/
TEST_F(DynamicSceneNodeTest, testCentroidCheck)
{
    LOG(WARNING) << "Running testCentroidCheck";
    kimera::DynamicSceneGraph d_graph(nh, nh_private);
    // Check far apart fails.
    kimera::DynamicSceneNode node1, node2;
    node1.attributes_.position_ = NodePosition(0, 0, 0);
    node2.attributes_.position_ = NodePosition(5, 5, 5);

    // pcl headers are in microseconds
    unsigned long inv_micro = pow(10, 6);
    node1.attributes_.timestamp_ = 0 * inv_micro;
    node2.attributes_.timestamp_ = 1 * inv_micro;
    EXPECT_FALSE(d_graph.checkDynamicFeasibility(node2, node1));
    
    // Check with increased time, passes.
    node1.attributes_.timestamp_ = 1 * inv_micro;
    node2.attributes_.timestamp_ = 8 * inv_micro;
    EXPECT_TRUE(d_graph.checkDynamicFeasibility(node2, node1));

    // Check very edge passes.
    node1.attributes_.position_ = NodePosition(5, 5, 0);
    node1.attributes_.timestamp_ = 0;
    node2.attributes_.timestamp_ = 5 * inv_micro / d_graph.FEASIBLE_DYN_RATE_ + 1000;
    EXPECT_TRUE(d_graph.checkDynamicFeasibility(node2, node1));

    // Check that same point passes.
    node1.attributes_.position_ = NodePosition(5, 5, 5);
    EXPECT_TRUE(d_graph.checkDynamicFeasibility(node2, node1));
}

/*
* Test the CheckMeshFeasibility method
*/
TEST_F(DynamicSceneNodeTest, testCheckMeshFeasibility)
{
    kimera::DynamicSceneGraph d_graph(nh, nh_private);
    kimera::DynamicSceneNode node1, node2;
    std::vector<double> test_joints;
    for(size_t i = 0; i < kimera::NUM_JOINTS * 3; i++){
        test_joints.push_back(i);
    }
    // Check same joints pass.
    node1.joints_ = Eigen::Map<kimera::JointMatrix>(&test_joints[0]);
    node2.joints_ = Eigen::Map<kimera::JointMatrix>(&test_joints[0]);
    
    // pcl headers are in microseconds
    unsigned long inv_micro = pow(10, 6);
    node1.attributes_.timestamp_ = 0 * inv_micro;
    node2.attributes_.timestamp_ = 1 * inv_micro;
    EXPECT_TRUE(d_graph.checkMeshFeasibility(node2, node1));

    // Check far away joints fail
    std::vector<double> break_joints;
    for (size_t i = 0; i < kimera::NUM_JOINTS * 3; i++){
        break_joints.push_back(2 * i);
    }

    node2.joints_ = Eigen::Map<kimera::JointMatrix>(&break_joints[0]);
    EXPECT_FALSE(d_graph.checkMeshFeasibility(node2, node1));

    // Check just one far away joint causes failure
    std::vector<double> test_joints_copy(test_joints);
    size_t last_idx = test_joints.size() - 1;
    test_joints_copy[last_idx] = test_joints[last_idx] * 2;
    test_joints_copy[last_idx - 1] = test_joints[last_idx - 1] * 2;
    test_joints_copy[last_idx - 2] = test_joints[last_idx - 2] * 2;

    node2.joints_ = Eigen::Map<kimera::JointMatrix>(&test_joints_copy[0]);
    EXPECT_FALSE(d_graph.checkMeshFeasibility(node2, node1));

    // Check that all joints being slightly off passes.
    std::vector<double> off_test_joints;
    for(auto value : test_joints){
        off_test_joints.push_back(value + 0.2);
    }
    node2.joints_ = Eigen::Map<kimera::JointMatrix>(&off_test_joints[0]);
    EXPECT_TRUE(d_graph.checkMeshFeasibility(node2, node1));
}

/*
* Check that the SMPLList msg is deserialized properly.
*/
TEST_F(DynamicSceneNodeTest, testDeserialization)
{
    kimera::DynamicSceneGraph d_graph(nh, nh_private);

    std::vector<double> test_joints;
    for(size_t i = 0; i < kimera::NUM_JOINTS * 3; i++){
        test_joints.push_back(i);
    }
    graph_cmr_ros::SMPL smpl;
    smpl.header.stamp = ros::Time();
    smpl.centroid = {1, 2, 3};
    smpl.joints = test_joints;

    kimera::DynamicSceneNode test_node;
    d_graph.dynamicSceneNodeFromSMPL(smpl, test_node);
    // TODO (argupta) decide how to best test this.
}

/*
* Check that if we send one human, we get the correct number of graphs and connectivity.
*/
TEST_F(DynamicSceneNodeTest, testGraphConnectionSingleStill)
{
    kimera::DynamicSceneGraph d_graph(nh, nh_private);
    // Simulate messages for 1 human -> pass to callback (obey dynamic feasibility check)

    boost::shared_ptr<graph_cmr_ros::SMPLList> msg(new graph_cmr_ros::SMPLList);
    std::vector<double> test_joints;
    for(size_t i = 0; i < kimera::NUM_JOINTS * 3; i++){
        test_joints.push_back(i);
    }
    graph_cmr_ros::SMPL smpl;
    float base_time = 0.1;
    smpl.header.stamp = ros::Time(base_time);
    smpl.centroid = {1, 1, 1};
    smpl.joints = test_joints;
    
    msg->human_meshes.push_back(smpl);

    // Append the same node several times over some number of timesteps, and ensure everything stays the same.
    for (size_t i = 0 ; i < 2 ; i++){
        d_graph.humanCallback(msg);

        // Ensure internal variables that are meant to be updated together are.
        EXPECT_EQ(d_graph.pose_graphs_.size(), d_graph.graph_priors_.size()) << "STILL -- graphs not aligned with priors -- iter: " << i;

        // Ensure that there are the correct number of graphs.
        EXPECT_EQ(d_graph.pose_graphs_.size(), 1u) << "STILL -- too many pose graphs -- iter: " << i;

        // Ensure that there are the right number of nodes in the graph.
        EXPECT_EQ(d_graph.last_poses_.size(), 1u) << "STILL -- too many last poses -- iter: " << i;
        EXPECT_EQ(d_graph.last_poses_[0].second, 0) << "STILL -- too many poses in chain -- iter: " << i;

        // Increment time to ensure that the checks pass.
        base_time += 2.0;
        msg->human_meshes[0].header.stamp = ros::Time(base_time);
    }

    // Append a slightly offset centroid node over several timesteps, ensure things change accordingly.
    for (size_t i = 0 ; i < 2 ; i++){
        msg->human_meshes[0].centroid[1] += 1.0;
        msg->human_meshes[0].centroid[2] += 1.0;
        base_time += 2.0;
        msg->human_meshes[0].header.stamp = ros::Time(base_time);
        d_graph.humanCallback(msg);

        // Ensure internal variables that are meant to be updated together are.
        EXPECT_EQ(d_graph.pose_graphs_.size(), d_graph.graph_priors_.size()) << "MOVING -- graphs not aligned with priors -- iter: " << i;

        // Ensure that there are the correct number of graphs.
        EXPECT_EQ(d_graph.pose_graphs_.size(), 1) << "MOVING -- too many pose graphs -- iter: " << i;

        // Ensure that there are the right number of nodes in the graph.
        EXPECT_EQ(d_graph.last_poses_.size(), 1) << "MOVING -- too many last poses -- iter: " << i;
        EXPECT_EQ(d_graph.last_poses_[0].second, 1 + i) << "MOVING -- incorrect poses in chain -- iter: " << i;
    }

    // Add a drastic jump to ensure that a new graph is added.
    for (size_t i = 0 ; i < 2 ; i++){
        msg->human_meshes[0].centroid[1] += 2.0;
        msg->human_meshes[0].centroid[2] += 1.0;

        base_time += 1.0;
        msg->human_meshes[0].header.stamp = ros::Time(base_time);
        d_graph.humanCallback(msg);

        // Ensure internal variables that are meant to be updated together are.
        EXPECT_EQ(d_graph.pose_graphs_.size(), d_graph.graph_priors_.size()) << "MULTI -- graphs not aligned with priors -- iter: " << i;

        // Ensure that there are the correct number of graphs.
        EXPECT_EQ(d_graph.pose_graphs_.size(), 2 + i) << "MULTI -- too many pose graphs -- iter: " << i;

        // Ensure that there are the right number of nodes in the graph.
        EXPECT_EQ(d_graph.last_poses_.size(), 2 + i) << "MULTI -- too many last poses -- iter: " << i;
        for (size_t j = 0; j <= i; j++){
            if (j == 0) EXPECT_EQ(d_graph.last_poses_[j].second, 2) << "multi -- incorrect poses in chain -- iter: " << i;
            else EXPECT_EQ(d_graph.last_poses_[j].second, 0) << "multi -- incorrect poses in chain -- iter: " << i;
        }
    }
}

int main(int argc, char* argv[]){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    return RUN_ALL_TESTS();
}
