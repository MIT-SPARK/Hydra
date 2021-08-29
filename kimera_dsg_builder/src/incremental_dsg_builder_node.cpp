#include "kimera_dsg_builder/incremental_dsg_frontend.h"


int main(int argc, char* argv[]) {
  ros::init(argc, argv, "incremental_dsg_builder_node");

  FLAGS_minloglevel = 3;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  ros::NodeHandle nh("~");

  kimera::incremental::DsgFrontend frontend(nh);
  frontend.spin();
  return 0;
}
