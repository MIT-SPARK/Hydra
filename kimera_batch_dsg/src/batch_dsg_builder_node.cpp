#include <kimera_semantics_ros/semantic_tsdf_server.h>
#include "kimera_dsg_builder/offline_dsg_builder.h"

#include <ros/ros.h>
#include <ros/topic_manager.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <voxblox/core/common.h>

#include <chrono>

inline bool haveClock() {
  return ros::TopicManager::instance()->getNumPublishers("/clock");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "kimera_offline_dsg_builder");

  FLAGS_minloglevel = 3;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  kimera::SemanticTsdfServer tsdf_server(nh, nh_private);

  ros::WallRate r(10);

  ROS_INFO("Waiting for bag to start");
  while (ros::ok() && !haveClock()) {
    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("Integrating TSDF");
  while (ros::ok() && haveClock()) {
    ros::spinOnce();
    r.sleep();
  }

  // make sure we're done integrating the tsdf
  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(5.0));

  std::string dsg_output_path = "";
  ros::param::get("~dsg_output_path", dsg_output_path);

  ROS_INFO("Bag finished, making DSG builder");
  kimera::OfflineDsgBuilder builder(nh, nh_private, tsdf_server);
  ROS_INFO("Starting scene graph construction");
  auto start = std::chrono::high_resolution_clock::now();
  builder.reconstruct();
  auto stop = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_s = stop - start;
  std::ofstream file;
  file.open(dsg_output_path + "/batch_timing.csv",
            std::ofstream::out | std::ofstream::app);
  file << elapsed_s.count() << "\n";
  file.close();
  ROS_INFO("Finished scene graph construction");

  ROS_WARN("Exiting!");

  bool visualize = false;
  ros::param::get("~visualize", visualize);
  if (visualize) {
    ros::spin();
  }

  builder.saveSceneGraph(dsg_output_path + "/dsg.json", false);
  builder.saveSceneGraph(dsg_output_path + "/dsg_with_mesh.json", true);

  tsdf_server.saveMap(dsg_output_path + "/tsdf.vxblx");

  return 0;
}
