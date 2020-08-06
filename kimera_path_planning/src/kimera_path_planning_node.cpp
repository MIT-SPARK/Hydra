/**
 * @file   kimera_path_planning_node.cpp
 * @brief  Main for Kimera-Path-Planning
 * @author Antoni Rosinol
 */

#include <ros/ros.h>

#include <glog/logging.h>
#include <gflags/gflags.h>

#include "kimera_path_planning/kimera_path_planning.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "kimera_path_planning");

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  kimera::SceneGraphGlobalPlanner scene_graph_global_planner(nh, nh_private);
  LOG(INFO) << "Generate Sparse Graph.";
  scene_graph_global_planner.generateSparseGraph();
  LOG(INFO) << "Generate Scene Graph.";
  scene_graph_global_planner.generateSceneGraph();

  ros::spin();

  return EXIT_SUCCESS;
}
