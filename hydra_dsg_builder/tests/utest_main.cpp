#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <ros/ros.h>

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  ros::init(argc, argv, "test_hydra_dsg_builder");

  FLAGS_logtostderr = true;
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  auto res = RUN_ALL_TESTS();

  ros::shutdown();

  return res;
}
