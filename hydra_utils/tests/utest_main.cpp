#include <gtest/gtest.h>
#include <ros/ros.h>


auto main(int argc, char **argv) -> int {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "hydra_utest");

  return RUN_ALL_TESTS();
}
