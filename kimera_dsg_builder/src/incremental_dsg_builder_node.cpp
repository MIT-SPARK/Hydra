#include "kimera_dsg_builder/incremental_dsg_backend.h"
#include "kimera_dsg_builder/incremental_dsg_frontend.h"
#include "kimera_dsg_builder/timing_utilities.h"

#include <ros/topic_manager.h>

using kimera::ElapsedTimeRecorder;
using kimera::KimeraDsgLayers;
using kimera::LayerId;
using kimera::incremental::SharedDsgInfo;

inline bool haveClock() {
  return ros::TopicManager::instance()->getNumPublishers("/clock");
}

std::optional<uint64_t> getTimeNs(const kimera::DynamicSceneGraph& graph, gtsam::Symbol key) {
  kimera::NodeSymbol node(key.chr(), key.index());
  if (!graph.hasNode(node)) {
    LOG(ERROR) << "Missing node << " << node.getLabel() << "when logging loop closure";
  }
  return graph.getDynamicNode(node).value().get().timestamp.count();
}

void spinUntilBagFinished() {
  ros::WallRate r(50);
  ROS_INFO("Waiting for bag to start");
  while (ros::ok() && !haveClock()) {
    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("Running...");
  while (ros::ok() && haveClock()) {
    ros::spinOnce();
    r.sleep();
  }

  ros::spinOnce();  // make sure all the callbacks are processed
  ROS_WARN("Exiting!");
  ros::shutdown();
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "incremental_dsg_builder_node");

  FLAGS_minloglevel = 3;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  ros::NodeHandle nh("~");
  std::string dsg_output_path = "";
  nh.getParam("dsg_output_path", dsg_output_path);

  bool exit_after_bag = false;
  nh.getParam("exit_after_bag", exit_after_bag);

  const LayerId mesh_layer_id = 1;
  const std::map<LayerId, char>& layer_id_map{{KimeraDsgLayers::OBJECTS, 'o'},
                                              {KimeraDsgLayers::PLACES, 'p'},
                                              {KimeraDsgLayers::ROOMS, 'r'},
                                              {KimeraDsgLayers::BUILDINGS, 'b'}};

  SharedDsgInfo::Ptr frontend_dsg(new SharedDsgInfo(layer_id_map, mesh_layer_id));
  SharedDsgInfo::Ptr backend_dsg(new SharedDsgInfo(layer_id_map, mesh_layer_id));

  std::list<kimera::incremental::LoopClosureLog> loop_closures;
  {  // scope for frontend / backend pair
    kimera::incremental::DsgBackend backend(nh, frontend_dsg, backend_dsg);
    kimera::incremental::DsgFrontend frontend(nh, frontend_dsg);

    frontend.start();
    backend.start();

    if (exit_after_bag) {
      spinUntilBagFinished();
    } else {
      ros::spin();
    }

    if (!dsg_output_path.empty()) {
      // save pgmo trajectory
      std_srvs::Empty temp;
      try {
        backend.saveTrajectoryCallback(temp.request, temp.response);
      } catch (const std::exception& e) {
        LOG(ERROR) << "Saving trajectory failed: " << e.what();
      }
    }
    loop_closures = backend.getLoopClosures();
  }

  if (!dsg_output_path.empty()) {
    LOG(INFO) << "[DSG Node] Saving scene graph and other stats and logs to "
              << dsg_output_path;
    backend_dsg->graph->save(dsg_output_path + "/backend/dsg.json", false);
    backend_dsg->graph->save(dsg_output_path + "/backend/dsg_with_mesh.json");
    frontend_dsg->graph->save(dsg_output_path + "/frontend/dsg.json", false);
    frontend_dsg->graph->save(dsg_output_path + "/frontend/dsg_with_mesh.json");

    // Output timing statistics
    const ElapsedTimeRecorder& timer = ElapsedTimeRecorder::instance();
    timer.logAllElapsed(dsg_output_path);
    timer.logStats(dsg_output_path);
    LOG(INFO) << "[DSG Node] Saved scene graph, stats, and logs to " << dsg_output_path;

    const std::string output_csv = dsg_output_path + "/loop_closures.csv";
    std::ofstream output_file;
    output_file.open(output_csv);

    output_file << "time_from_ns,time_to_ns,x,y,z,qw,qx,qy,qz,type" << std::endl;
    for (const auto& loop_closure : loop_closures) {
      auto time_from = getTimeNs(*frontend_dsg->graph, loop_closure.from);
      auto time_to = getTimeNs(*frontend_dsg->graph, loop_closure.to);
      if (!time_from || !time_to) {
        continue;
      }
      output_file << *time_from << "," << *time_to << ",";
      gtsam::Point3 pos = loop_closure.to_T_from.translation();
      output_file << pos.x() << "," << pos.y() << "," << pos.z() << ",";
      gtsam::Quaternion quat = loop_closure.to_T_from.rotation().toQuaternion();
      output_file << quat.w() << ", " << quat.x() << "," << quat.y() << "," << quat.z() <<",";
      output_file << (loop_closure.dsg ? 1 : 0);
      output_file << std::endl;
    }
  }

  return 0;
}


