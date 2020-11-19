#include "kimera_scene_graph/scene_graph_builder.h"

#include <ros/ros.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <voxblox/core/common.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "kimera_scene_graph_simulator");

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  kimera::SceneGraphBuilder scene_graph_builder(nh, nh_private);

  LOG(INFO) << "Starting scene graph construction.";
  scene_graph_builder.sceneGraphReconstruction(false);
  LOG(INFO) << "Finishing scene graph construction.";

  ros::spin();

  return 0;
}
