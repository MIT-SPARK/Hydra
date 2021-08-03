#include "kimera_topology/topology_server.h"

#include <kimera_semantics_ros/semantic_tsdf_server.h>
#include <voxblox_ros/tsdf_server.h>

#include <glog/logging.h>
#include <ros/ros.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "kimera_topology_node");

  FLAGS_minloglevel = 3;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  ros::NodeHandle pnh("~");

  bool use_semantic_tsdf_server = false;
  pnh.getParam("use_semantic_tsdf_server", use_semantic_tsdf_server);
  if (use_semantic_tsdf_server) {
    ROS_INFO("Using Semantic TSDF Server");
    kimera::topology::TopologyServer<kimera::SemanticTsdfServer> server(pnh);
    server.spin();
  } else {
    ROS_INFO("Using Normal (Non-Semantic) TSDF Server");
    kimera::topology::TopologyServer<voxblox::TsdfServer> server(pnh);
    server.spin();
  }

  return 0;
}
