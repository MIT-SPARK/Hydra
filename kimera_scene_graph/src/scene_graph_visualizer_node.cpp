#include "kimera_scene_graph/scene_graph_visualizer.h"

#include <kimera_dsg/scene_graph.h>
#include <ros/ros.h>

#include <glog/logging.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "scene_graph_visualizer");

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string scene_graph_input_path = "";
  nh_private.param(
      "scene_graph_input_path", scene_graph_input_path, scene_graph_input_path);

  kimera::SceneGraphVisualizer scene_graph_visualizer(
      nh, nh_private, kimera::getDefaultLayerIds());

  CHECK(!scene_graph_input_path.empty()) << "Empty scene graph input path...";
  VLOG(1) << "Loading scene graph from: " << scene_graph_input_path.c_str();
  kimera::DynamicSceneGraph::Ptr scene_graph(new kimera::DynamicSceneGraph());
  // kimera::load(scene_graph_input_path, &scene_graph);

  VLOG(1) << "Starting scene graph visualizer.";
  scene_graph_visualizer.visualize(scene_graph);
  VLOG(1) << "Finished scene graph visualizer.";

  ros::spin();

  return 0;
}
