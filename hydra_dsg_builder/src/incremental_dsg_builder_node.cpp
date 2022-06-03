/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra_dsg_builder/incremental_dsg_backend.h"
#include "hydra_dsg_builder/incremental_dsg_frontend.h"
#include "hydra_dsg_builder/incremental_dsg_lcd.h"

#include <hydra_utils/timing_utilities.h>
#include <ros/topic_manager.h>
#include <std_srvs/Empty.h>

using hydra::DsgLayers;
using hydra::LayerId;
using hydra::incremental::SharedDsgInfo;
using hydra::timing::ElapsedTimeRecorder;

enum class ExitMode { CLOCK, SERVICE, NORMAL };

inline bool haveClock() {
  return ros::TopicManager::instance()->getNumPublishers("/clock");
}

ExitMode getExitMode(const ros::NodeHandle& nh) {
  std::string exit_mode_str = "NORMAL";
  nh.getParam("exit_mode", exit_mode_str);

  if (exit_mode_str == "CLOCK") {
    return ExitMode::CLOCK;
  } else if (exit_mode_str == "SERVICE") {
    return ExitMode::SERVICE;
  } else if (exit_mode_str == "NORMAL") {
    return ExitMode::NORMAL;
  } else {
    ROS_WARN_STREAM("Unrecognized option: " << exit_mode_str
                                            << ". Defaulting to NORMAL");
    return ExitMode::NORMAL;
  }
}

void spinWhileClockPresent() {
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
}

struct ServiceFunctor {
  ServiceFunctor() : should_exit(false) {}

  bool callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    should_exit = true;
    return true;
  }

  bool should_exit;
};

void spinUntilExitRequested() {
  ServiceFunctor functor;

  ros::NodeHandle nh("~");
  ros::ServiceServer service =
      nh.advertiseService("shutdown", &ServiceFunctor::callback, &functor);

  ros::WallRate r(50);
  ROS_INFO("Running...");
  while (ros::ok() && !functor.should_exit) {
    ros::spinOnce();
    r.sleep();
  }

  ros::spinOnce();  // make sure all the callbacks are processed
  ROS_WARN("Exiting!");
}

std::optional<uint64_t> getTimeNs(const hydra::DynamicSceneGraph& graph,
                                  gtsam::Symbol key) {
  hydra::NodeSymbol node(key.chr(), key.index());
  if (!graph.hasNode(node)) {
    LOG(ERROR) << "Missing node << " << node.getLabel() << "when logging loop closure";
    LOG(ERROR) << "Num dynamic nodes: " << graph.numDynamicNodes();
    return std::nullopt;
  }

  return graph.getDynamicNode(node).value().get().timestamp.count();
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "incremental_dsg_builder_node");

  FLAGS_minloglevel = 3;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh("~");
  std::string dsg_output_path = "";
  nh.getParam("log_path", dsg_output_path);

  const auto exit_mode = getExitMode(nh);

  bool enable_lcd = false;
  nh.getParam("enable_lcd", enable_lcd);

  nh.getParam("disable_timer_output", ElapsedTimeRecorder::instance().disable_output);

  const LayerId mesh_layer_id = 1;
  const std::map<LayerId, char>& layer_id_map{{DsgLayers::OBJECTS, 'o'},
                                              {DsgLayers::PLACES, 'p'},
                                              {DsgLayers::ROOMS, 'r'},
                                              {DsgLayers::BUILDINGS, 'b'}};

  SharedDsgInfo::Ptr frontend_dsg(new SharedDsgInfo(layer_id_map, mesh_layer_id));
  SharedDsgInfo::Ptr backend_dsg(new SharedDsgInfo(layer_id_map, mesh_layer_id));

  pcl::PolygonMesh frontend_mesh;
  std::vector<ros::Time> frontend_mesh_times;

  std::list<hydra::incremental::LoopClosureLog> loop_closures;
  {  // scope for frontend / backend pair
    hydra::incremental::DsgBackend backend(nh, frontend_dsg, backend_dsg);
    hydra::incremental::DsgFrontend frontend(nh, frontend_dsg);
    std::shared_ptr<hydra::incremental::DsgLcd> lcd;
    if (enable_lcd) {
      lcd.reset(new hydra::incremental::DsgLcd(nh, frontend_dsg));
    }

    frontend.start();
    backend.start();
    if (lcd) {
      lcd->start();
    }

    switch (exit_mode) {
      case ExitMode::CLOCK:
        spinWhileClockPresent();
        break;
      case ExitMode::SERVICE:
        spinUntilExitRequested();
        break;
      case ExitMode::NORMAL:
      default:
        ros::spin();
        break;
    }

    frontend.stop();
    if (lcd) {
      lcd->stop();
    }
    backend.stop();
    loop_closures = backend.getLoopClosures();

    if (!dsg_output_path.empty()) {
      frontend_mesh = frontend.getFrontendMesh();
      frontend_mesh_times = frontend.getFrontendMeshStamps();
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

    output_file << "time_from_ns,time_to_ns,x,y,z,qw,qx,qy,qz,type,level" << std::endl;
    for (const auto& loop_closure : loop_closures) {
      // pose = src.between(dest) or to_T_from
      auto time_from = getTimeNs(*frontend_dsg->graph, loop_closure.dest);
      auto time_to = getTimeNs(*frontend_dsg->graph, loop_closure.src);
      if (!time_from || !time_to) {
        continue;
      }

      const gtsam::Point3 pos = loop_closure.src_T_dest.translation();
      const gtsam::Quaternion quat = loop_closure.src_T_dest.rotation().toQuaternion();

      output_file << *time_from << "," << *time_to << ",";
      output_file << pos.x() << "," << pos.y() << "," << pos.z() << ",";
      output_file << quat.w() << ", " << quat.x() << "," << quat.y() << "," << quat.z()
                  << ",";
      output_file << (loop_closure.dsg ? 1 : 0) << "," << loop_closure.level;
      output_file << std::endl;
    }

    kimera_pgmo::WriteMeshWithStampsToPly(
        dsg_output_path + "/frontend/mesh.ply", frontend_mesh, frontend_mesh_times);
  }

  return 0;
}
