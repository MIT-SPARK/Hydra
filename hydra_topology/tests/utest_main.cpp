#include <ros/ros.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

auto main(int argc, char **argv) -> int {
  ::testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "utest_hydra_topology");

  FLAGS_minloglevel = 3;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  return RUN_ALL_TESTS();
}
