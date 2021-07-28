#include "kimera_topology/topology_server.h"

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

  // TODO(nathan) ros param here for switching between semantic / non-semantic
  // integration

  ros::NodeHandle pnh("~");
  kimera::topology::TopologyServer<voxblox::TsdfServer> server(pnh);
  server.spin();

  return 0;
}
