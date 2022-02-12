/**
 * @file   kimera_dynamic_scene_graph_eval_node.cpp
 * @brief  Main for Dynamic-Kimera-Scene-Graph
 * @author Marcus Abate
 */

#include <ros/ros.h>

#include <glog/logging.h>
#include <gflags/gflags.h>

#include "kimera_scene_graph/dynamic_scene_graph_evaluator.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "kimera_dynamic_scene_graph_eval");

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  kimera::DynamicSceneGraphEvaluator dynamic_graph(nh, nh_private);

  ros::spin();

  return EXIT_SUCCESS;
}
