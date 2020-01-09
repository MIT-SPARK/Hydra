/**
 * @file   kimera_scene_graph_node.cpp
 * @brief  Main for Kimera-Scene-Graph
 * @author Antoni Rosinol
 */

#include "kimera_scene_graph/scene_graph_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "kimera_scene_graph");

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // TODO(Toni): at the end this should be building the graph
  // (instead of the SimulationSceneGraphServer).
  // kimera::SceneGraphServer node(nh, nh_private);

  ros::spin();

  return EXIT_SUCCESS;
}
