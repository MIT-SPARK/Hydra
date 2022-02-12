#include <gflags/gflags.h>
#include <gtest/gtest.h>
#include <glog/logging.h>

#include <tf/transform_broadcaster.h>
#include "kimera_scene_graph/dynamic_scene_graph_evaluator.h"

// Keep node handles persistent.
class DynamicHumanNodeEvaluatorTestFixture : public ::testing::Test {
 public:
  DynamicHumanNodeEvaluatorTestFixture() {}
  ~DynamicHumanNodeEvaluatorTestFixture() {}

 protected:
  void SetUp() override {}
  void TearDown() override {}

 protected:
  ros::NodeHandle nh;
  ros::NodeHandle nh_private;
};

TEST_F(DynamicHumanNodeEvaluatorTestFixture, testVariance) {
  LOG(INFO) << "Running testVariance";
  kimera::DynamicSceneGraphEvaluator dsn_eval(nh, nh_private);
  float err = 0;
  float sq_err = 0;
  int count = 0;
  float variance = 0;
  float mean = 50;
  for (int i = 1; i <= 100; i++) {
    err += i;
    sq_err += pow(i, 2);
    variance += pow(mean - i, 2);
    count++;
  }
  variance = variance / count;
  float online_variance = dsn_eval.calcVariance(err, sq_err, count);
  EXPECT_LT(abs(variance - online_variance), 0.3)
      << "Variance: " << variance << "  Online Variance: " << online_variance;
}

TEST_F(DynamicHumanNodeEvaluatorTestFixture, testSmallestCentroidDistAndAng) {
  LOG(INFO) << "Running testSmallestCentroidDistAndAng";
  kimera::DynamicSceneGraphEvaluator dsn_eval(nh, nh_private);
  kimera::DynamicHumanNode node;
  node.pose_ =
      gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(1.0, 1.0, 1.0));

  float base_time = 0.1;
  node.msg_.header.stamp = ros::Time(base_time);

  // Test when the time is not in the ground-truth
  EXPECT_EQ(dsn_eval.smallestCentroidDistAndAng(node).first, -1);
  EXPECT_EQ(dsn_eval.smallestCentroidDistAndAng(node).second, -1);

  // Test when the time is in the ground-truth
  tf::Transform tr1(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 1, -0.5));
  tf::Transform tr2(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 1, 20));

  tf::StampedTransform tf1, tf2;
  tf1.setData(tr1);
  tf2.setData(tr2);
  dsn_eval.time_to_gt_[node.msg_.header.stamp.toSec()] =
      std::vector<tf::StampedTransform>({tf1, tf2});
  EXPECT_EQ(abs(dsn_eval.smallestCentroidDistAndAng(node).first), 1.5)
      << dsn_eval.smallestCentroidDistAndAng(node).first;
  EXPECT_LT(abs(dsn_eval.smallestCentroidDistAndAng(node).second), 1e-6)
      << dsn_eval.smallestCentroidDistAndAng(node).second;
}

/*
TEST_F(DynamicHumanNodeEvaluatorTestFixture, testErrorServiceCall){
    DynamicSceneGraphEvaluator dsg_eval(nh, nh_private);
    tf::TransformBroadcaster br;

    dsg_eval.dynamic_scene_graph_.feasible_dyn_rate_ = 2.0;
    dsg_eval.dynamic_scene_graph_.feasible_mesh_rate_ = 1.5;
    dsg_eval.dynamic_scene_graph_.time_cap_ = 10.0;
    dsg_eval.dynamic_scene_graph_.check_position_closeness_ = false;
    dsg_eval.world_frame_ = "world";
    dsg_eval.num_humans_ = 1;
    dsg_eval.human_height_ = 2.1;

    boost::shared_ptr<graph_cmr_ros::SMPLList> msg(new graph_cmr_ros::SMPLList);
    std::vector<double> test_joints;
    for(size_t i = 0; i < kimera::NUM_JOINTS * 3; i++){
        test_joints.push_back(i);
    }
    graph_cmr_ros::SMPL smpl;
    auto base_time = ros::Time::now();
    smpl.header.stamp = base_time;
    smpl.centroid = {1, 1, 1};
    smpl.orientation = {1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0};
    smpl.joints = test_joints;

    msg->human_meshes.push_back(smpl);

    // Publish a "fake" transformation
    gtsam::Rot3 rot(1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
    gtsam::Quaternion quat = rot.toQuaternion();
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(smpl.centroid[0] + 1.0, smpl.centroid[1] +
1.0, smpl.centroid[2] - dsg_eval.human_height_ / 2.0) );
    transform.setRotation(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w())
);

    br.sendTransform(tf::StampedTransform(transform, base_time, "world",
"object_0"));

    // Append the same node several times over some number of timesteps, and
ensure everything stays the same.
    for (size_t i = 0 ; i < 2 ; i++){
        dsg_eval.dynamic_scene_graph_.humanCallback(msg);
        dsg_eval.humanCallback(msg);

        // Increment time to ensure that the checks pass.
        ros::Duration(1.0).sleep();
        base_time = ros::Time::now();
        msg->human_meshes[0].header.stamp = base_time;
        br.sendTransform(tf::StampedTransform(transform, base_time, "world",
"object_0"));
        ros::Duration(0.5).sleep();
    }

    // Run the Error calculations for the nodes that should have been in place.
    // TODO (argupta) check original nodes have correct values + optimized is
less
    kimera_scene_graph::CentroidErrorRequest::Request req;
    kimera_scene_graph::CentroidErrorRequest::Response res;
    dsg_eval.errorServiceCall(req, res);

    EXPECT_TRUE(abs(res.raw_error - sqrt(2.0)) < 0.001) << res.raw_error;
    EXPECT_TRUE(res.raw_variance < 0.001) << res.raw_error;
    EXPECT_TRUE(res.raw_error > res.optimized_error);

    // Append a slightly offset centroid node over several timesteps, ensure
things change accordingly.
    for (size_t i = 0 ; i < 2 ; i++){
        ros::Duration(1.0).sleep();
        base_time = ros::Time::now();

        msg->human_meshes[0].centroid[1] += 1.0;
        msg->human_meshes[0].centroid[2] += 1.0;
        msg->human_meshes[0].header.stamp = base_time;
        br.sendTransform(tf::StampedTransform(transform, base_time, "world",
"object_0"));
        ros::Duration(0.5).sleep();

        dsg_eval.dynamic_scene_graph_.humanCallback(msg);
        dsg_eval.humanCallback(msg);

        dsg_eval.errorServiceCall(req, res);

        // Run the Error calculations for the nodes that should have been in
place.
        CHECK_TRUE(res.raw_error > res.optimized_error);
    }
}*/
