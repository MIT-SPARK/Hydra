#include "kimera_dsg_builder/offline_dsg_builder.h"

#include <ros/ros.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <voxblox/core/common.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "kimera_offline_dsg_builder");

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  kimera::OfflineDsgBuilder builder(nh, nh_private);

  LOG(INFO) << "Starting scene graph construction.";
  builder.reconstruct();
  LOG(INFO) << "Finished scene graph construction.";

  ros::spin();

  return 0;
}
