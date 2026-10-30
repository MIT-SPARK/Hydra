#include <gtest/gtest.h>
#include <hydra_multi/backend/loop_closure_initial_align.h>
#include <kimera_pgmo/deformation_graph.h>

namespace hydra_multi {

using Info = InitialAlignModule::Info;

TEST(LoopClosureInitialAlignTests, AddIntraRobotLoopClosure) {
  LoopClosureInitialAlignModule::Config ia_config;
  LoopClosureInitialAlignModule ia(ia_config);

  EXPECT_FALSE(
      ia.addLoopClosure(gtsam::Symbol('a', 0), gtsam::Symbol('a', 2), gtsam::Pose3()));
  EXPECT_EQ(0, ia.getLoopClosures().size());
}

TEST(LoopClosureInitialAlignTests, AddNonRobotLoopClosure) {
  LoopClosureInitialAlignModule::Config ia_config;
  LoopClosureInitialAlignModule ia(ia_config);

  EXPECT_FALSE(
      ia.addLoopClosure(gtsam::Symbol('s', 0), gtsam::Symbol('u', 2), gtsam::Pose3()));
  EXPECT_EQ(0, ia.getLoopClosures().size());
}

TEST(LoopClosureInitialAlignTests, AddSingleInterRobotLoopClosure) {
  LoopClosureInitialAlignModule::Config ia_config;
  LoopClosureInitialAlignModule ia(ia_config);

  EXPECT_TRUE(
      ia.addLoopClosure(gtsam::Symbol('a', 0), gtsam::Symbol('b', 2), gtsam::Pose3()));
  EXPECT_EQ(1, ia.getLoopClosures().size());
  EXPECT_TRUE(ia.getLoopClosures().count(0));
  EXPECT_TRUE(ia.getLoopClosures()[0].count(1));
  EXPECT_EQ(1, ia.getLoopClosures()[0][1].size());
}

TEST(LoopClosureInitialAlignTests, AddThreeInterRobotLoopClosure) {
  LoopClosureInitialAlignModule::Config ia_config;
  LoopClosureInitialAlignModule ia(ia_config);

  EXPECT_TRUE(
      ia.addLoopClosure(gtsam::Symbol('a', 0), gtsam::Symbol('b', 1), gtsam::Pose3()));
  EXPECT_TRUE(
      ia.addLoopClosure(gtsam::Symbol('b', 1), gtsam::Symbol('a', 1), gtsam::Pose3()));
  EXPECT_TRUE(
      ia.addLoopClosure(gtsam::Symbol('a', 1), gtsam::Symbol('c', 0), gtsam::Pose3()));
  EXPECT_TRUE(
      ia.addLoopClosure(gtsam::Symbol('c', 1), gtsam::Symbol('b', 2), gtsam::Pose3()));
  EXPECT_EQ(1, ia.getLoopClosures()[0][1].size());
  EXPECT_EQ(1, ia.getLoopClosures()[1][0].size());
  EXPECT_EQ(1, ia.getLoopClosures()[0][2].size());
  EXPECT_EQ(1, ia.getLoopClosures()[2][1].size());
}

TEST(LoopClosureInitialAlignTests, UpdatePrivatePairwiseTransforms) {
  LoopClosureInitialAlignModule::Config ia_config;
  LoopClosureInitialAlignModule ia(ia_config);

  gtsam::Pose3 b1_T_a1 = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
  gtsam::Pose3 a1_T_c0 = gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(1, 0, 0));
  gtsam::Pose3 c1_T_b2 = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 0));
  ia.addLoopClosure(gtsam::Symbol('a', 0), gtsam::Symbol('b', 1), gtsam::Pose3());
  ia.addLoopClosure(gtsam::Symbol('b', 1), gtsam::Symbol('a', 1), b1_T_a1);
  ia.addLoopClosure(gtsam::Symbol('a', 1), gtsam::Symbol('c', 0), a1_T_c0);
  ia.addLoopClosure(gtsam::Symbol('c', 1), gtsam::Symbol('b', 2), c1_T_b2);
  ia.addLoopClosure(gtsam::Symbol('c', 2), gtsam::Symbol('b', 0), gtsam::Pose3());

  // Add odometry to deformation graph
  auto dgraph = std::make_shared<kimera_pgmo::DeformationGraph>();
  gtsam::Pose3 odom_step(gtsam::Rot3(0, 0, 1, 0), gtsam::Point3(0, 0, 0));
  for (char prefix : {'a', 'b', 'c'}) {
    gtsam::Pose3 odom_pose;
    for (size_t i = 0; i < 3; i++) {
      dgraph->addNewNode(gtsam::Symbol(prefix, i), 0, odom_pose, false);
      odom_pose = odom_pose.compose(odom_step);
    }
  }

  Info::Ptr info(new Info);
  info->dgraph = dgraph;
  ia.updatePairwiseTransforms(info);

  auto pt = ia.getPairwiseTransforms();

  EXPECT_TRUE(gtsam::assert_equal(odom_step.inverse(), pt[0][1][0]));
  EXPECT_TRUE(gtsam::assert_equal(
      odom_step.compose(b1_T_a1).compose(odom_step.inverse()), pt[1][0][0]));
  EXPECT_TRUE(gtsam::assert_equal(odom_step.compose(a1_T_c0), pt[0][2][0]));
  EXPECT_TRUE(gtsam::assert_equal(
      odom_step.compose(c1_T_b2).compose((odom_step.compose(odom_step)).inverse()),
      pt[2][1][0]));
  EXPECT_TRUE(gtsam::assert_equal(odom_step.compose(odom_step), pt[2][1][1], 0.1));
}

TEST(LoopClosureInitialAlignTests, ComputeInitialTransformsIdentity) {
  LoopClosureInitialAlignModule::Config ia_config;
  LoopClosureInitialAlignModule ia(ia_config);

  Info::Ptr info(new Info);
  auto dgraph = std::make_shared<kimera_pgmo::DeformationGraph>();

  info->loop_closures.push_back(
      {gtsam::Symbol('a', 0), gtsam::Symbol('b', 1), gtsam::Pose3()});
  info->loop_closures.push_back(
      {gtsam::Symbol('b', 1), gtsam::Symbol('a', 1), gtsam::Pose3()});
  info->loop_closures.push_back(
      {gtsam::Symbol('a', 1), gtsam::Symbol('c', 0), gtsam::Pose3()});
  info->loop_closures.push_back(
      {gtsam::Symbol('c', 1), gtsam::Symbol('b', 2), gtsam::Pose3()});

  // Add odometry to deformation graph
  for (char prefix : {'a', 'b', 'c'}) {
    gtsam::Pose3 odom_pose;
    for (size_t i = 0; i < 3; i++) {
      dgraph->addNewNode(gtsam::Symbol(prefix, i), 0, gtsam::Pose3(), false);
    }
  }

  info->dgraph = dgraph;
  ia.update(info);

  auto frames = ia.getFrames();

  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(), frames.at(0)));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(), frames.at(1)));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(), frames.at(2)));
}

TEST(LoopClosureInitialAlignTests, ComputeInitialTransforms) {
  LoopClosureInitialAlignModule::Config ia_config;
  LoopClosureInitialAlignModule ia(ia_config);

  Info::Ptr info(new Info);
  auto dgraph = std::make_shared<kimera_pgmo::DeformationGraph>();

  info->loop_closures.push_back({gtsam::Symbol('a', 0),
                                 gtsam::Symbol('b', 1),
                                 gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0))});
  info->loop_closures.push_back({gtsam::Symbol('b', 1),
                                 gtsam::Symbol('a', 1),
                                 gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-1, 0, 0))});
  info->loop_closures.push_back(
      {gtsam::Symbol('a', 1),
       gtsam::Symbol('c', 0),
       gtsam::Pose3(gtsam::Rot3(0, 0, 1, 0), gtsam::Point3(1, 0, 0))});
  info->loop_closures.push_back(
      {gtsam::Symbol('c', 1), gtsam::Symbol('b', 2), gtsam::Pose3()});
  info->loop_closures.push_back(
      {gtsam::Symbol('b', 0),
       gtsam::Symbol('c', 2),
       gtsam::Pose3(gtsam::Rot3(0, 0, 1, 0), gtsam::Point3(0, 0, 0))});

  // Add odometry to deformation graph
  for (char prefix : {'a', 'b', 'c'}) {
    gtsam::Pose3 odom_pose;
    for (size_t i = 0; i < 3; i++) {
      dgraph->addNewNode(gtsam::Symbol(prefix, i), 0, gtsam::Pose3(), false);
    }
  }

  info->dgraph = dgraph;
  ia.update(info);

  auto frames = ia.getFrames();

  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(), frames.at(0)));
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)), frames.at(1), 1e-2));
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(0, 0, 1, 0), gtsam::Point3(1, 0, 0)).inverse(),
      frames.at(2),
      1e-2));
}

TEST(LoopClosureInitialAlignTests, ComputeInitialTransformsSharedFrame) {
  // robots 1-3 are a fused prior map (one frame) and robot 0 only closes with robot 3
  LoopClosureInitialAlignModule::Config ia_config;
  ia_config.reference_robot_id = 1;
  LoopClosureInitialAlignModule ia(ia_config);

  Info::Ptr info(new Info);
  info->robot_frames = {{1, 1}, {2, 1}, {3, 1}};
  const gtsam::Pose3 a0_T_d1(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
  info->loop_closures.push_back(
      {gtsam::Symbol('a', 0), gtsam::Symbol('d', 1), a0_T_d1});
  info->loop_closures.push_back(
      {gtsam::Symbol('c', 0), gtsam::Symbol('d', 0), gtsam::Pose3()});

  auto dgraph = std::make_shared<kimera_pgmo::DeformationGraph>();
  for (char prefix : {'a', 'b', 'c', 'd'}) {
    for (size_t i = 0; i < 3; i++) {
      dgraph->addNewNode(gtsam::Symbol(prefix, i), 0, gtsam::Pose3(), false);
    }
  }

  info->dgraph = dgraph;
  ia.update(info);

  // the loop closure within the prior map is dropped and robot 3 aligns frame 1
  EXPECT_EQ(1, ia.getLoopClosures().size());
  EXPECT_EQ(1, ia.getLoopClosures()[0][1].size());

  auto frames = ia.getFrames();
  EXPECT_EQ(2, frames.size());
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(), frames.at(1)));
  EXPECT_TRUE(gtsam::assert_equal(a0_T_d1.inverse(), frames.at(0), 1e-2));
}

TEST(LoopClosureInitialAlignTests, ComputeInitialGuess) {
  LoopClosureInitialAlignModule::Config ia_config;
  LoopClosureInitialAlignModule ia(ia_config);

  Info::Ptr info(new Info);
  auto dgraph = std::make_shared<kimera_pgmo::DeformationGraph>();

  info->loop_closures.push_back({gtsam::Symbol('a', 0),
                                 gtsam::Symbol('b', 1),
                                 gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0))});
  info->loop_closures.push_back({gtsam::Symbol('b', 1),
                                 gtsam::Symbol('a', 1),
                                 gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-1, 0, 0))});
  info->loop_closures.push_back(
      {gtsam::Symbol('a', 1),
       gtsam::Symbol('c', 0),
       gtsam::Pose3(gtsam::Rot3(0, 0, 1, 0), gtsam::Point3(1, 0, 0))});
  info->loop_closures.push_back(
      {gtsam::Symbol('c', 1), gtsam::Symbol('b', 2), gtsam::Pose3()});
  info->loop_closures.push_back(
      {gtsam::Symbol('b', 0),
       gtsam::Symbol('c', 2),
       gtsam::Pose3(gtsam::Rot3(0, 0, 1, 0), gtsam::Point3(0, 0, 0))});

  // Add odometry to deformation graph
  gtsam::Values initial;
  for (char prefix : {'a', 'b', 'c'}) {
    gtsam::Pose3 odom_pose;
    for (size_t i = 0; i < 3; i++) {
      dgraph->addNewNode(gtsam::Symbol(prefix, i), 0, gtsam::Pose3(), false);
      initial.insert(gtsam::Symbol(prefix, i), gtsam::Pose3());
    }
  }

  info->dgraph = dgraph;
  ia.update(info);

  NodeIdRobotMap id_robot_map;
  // Add places
  initial.insert(gtsam::Symbol('p', 0),
                 gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)));
  id_robot_map[gtsam::Symbol('p', 0)] = 0;
  initial.insert(gtsam::Symbol('p', 1),
                 gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)));
  id_robot_map[gtsam::Symbol('p', 1)] = 1;
  initial.insert(gtsam::Symbol('p', 2),
                 gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)));
  id_robot_map[gtsam::Symbol('p', 2)] = 2;

  // Add objects
  initial.insert(gtsam::Symbol('O', 0),
                 gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-1, 0, 0)));
  id_robot_map[gtsam::Symbol('O', 0)] = 0;
  initial.insert(gtsam::Symbol('O', 1),
                 gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-1, 0, 0)));
  id_robot_map[gtsam::Symbol('O', 1)] = 1;
  initial.insert(gtsam::Symbol('O', 2),
                 gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-1, 0, 0)));
  id_robot_map[gtsam::Symbol('O', 2)] = 2;

  gtsam::Values aligned = ia.computeInitialGuess(info, initial, id_robot_map);
  for (size_t i = 0; i < 3; i++) {
    EXPECT_TRUE(gtsam::assert_equal(
        gtsam::Pose3(), aligned.at<gtsam::Pose3>(gtsam::Symbol('a', i)), 1e-2));
    EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
                                    aligned.at<gtsam::Pose3>(gtsam::Symbol('b', i)),
                                    1e-2));
    EXPECT_TRUE(gtsam::assert_equal(
        gtsam::Pose3(gtsam::Rot3(0, 0, 1, 0), gtsam::Point3(1, 0, 0)).inverse(),
        aligned.at<gtsam::Pose3>(gtsam::Symbol('c', i)),
        1e-2));
  }
}

}  // namespace hydra_multi
