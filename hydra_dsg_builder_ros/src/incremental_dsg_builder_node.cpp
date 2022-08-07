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
#include "hydra_dsg_builder_ros/node_utilities.h"
#include "hydra_dsg_builder_ros/ros_backend.h"
#include "hydra_dsg_builder_ros/ros_frontend.h"

#include <hydra_dsg_builder/incremental_dsg_backend.h>
#include <hydra_dsg_builder/incremental_dsg_lcd.h>
#include <hydra_utils/dsg_streaming_interface.h>
#include <hydra_utils/timing_utilities.h>

using namespace hydra;
using namespace hydra::incremental;
using namespace hydra::timing;

/*
  // lcd visualization (output callback)
  if (config_.visualize_dsg_lcd) {
    ros::NodeHandle nh(config_.lcd_visualizer_ns);
    visualizer_queue_.reset(new ros::CallbackQueue());
    nh.setCallbackQueue(visualizer_queue_.get());

    lcd_visualizer_.reset(new lcd::LcdVisualizer(nh, config_.detector.object_radius_m));
    lcd_visualizer_->setGraph(lcd_graph_);
    lcd_visualizer_->setLcdDetector(lcd_detector_.get());
  }

  if (lcd_visualizer_) {
    lcd_visualizer_->setGraphUpdated();
    lcd_visualizer_->redraw();
  }
*/

void run(const ros::NodeHandle& nh,
         const SharedDsgInfo::Ptr& frontend_dsg,
         const SharedDsgInfo::Ptr& backend_dsg,
         const std::string& output_path) {
  const auto exit_mode = getExitMode(nh);

  bool enable_lcd = false;
  nh.getParam("enable_lcd", enable_lcd);

  int robot_id = 0;
  nh.getParam("robot_id", robot_id);
  RobotPrefixConfig prefix(robot_id);

  auto backend_config = load_config<DsgBackendConfig>(nh);
  auto pgmo_config = load_config<kimera_pgmo::KimeraPgmoConfig>(nh, "pgmo");
  auto lcd_config = load_config<DsgLcdModuleConfig>(nh, "");

  SharedModuleState::Ptr shared_state(new SharedModuleState());
  // TODO(nathan) fix robot_id for frontend
  ROSFrontend frontend(nh, frontend_dsg, shared_state, robot_id);
  // TODO(nathan) consider allowing sub-based version here as well
  DsgBackend backend(
      prefix, backend_config, pgmo_config, frontend_dsg, backend_dsg, shared_state);

  RosBackendVisualizer backend_visualizer(nh);
  backend.addOutputCallback([&backend_visualizer](const DynamicSceneGraph& graph,
                                                  const pcl::PolygonMesh& mesh,
                                                  const kimera_pgmo::DeformationGraph& dgraph,
                                                  size_t timestamp_ns) {
    backend_visualizer.publishOutputs(graph, mesh, dgraph, timestamp_ns);
  });

  std::shared_ptr<DsgLcd> lcd;
  if (enable_lcd) {
    lcd.reset(new DsgLcd(prefix, lcd_config, frontend_dsg, shared_state));
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

  if (!output_path.empty()) {
    frontend.save(output_path + "/frontend/");
    backend.save(output_path + "/backend/");
    if (lcd) {
      lcd->save(output_path + "/lcd/");
    }
  }
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

  nh.getParam("disable_timer_output", ElapsedTimeRecorder::instance().disable_output);

  bool log_timing_incrementally = false;
  nh.getParam("log_timing_incrementally", log_timing_incrementally);
  if (log_timing_incrementally && dsg_output_path != "") {
    ElapsedTimeRecorder::instance().setupIncrementalLogging(dsg_output_path);
  }

  const LayerId mesh_layer_id = 1;
  const std::map<LayerId, char> layer_id_map{{DsgLayers::OBJECTS, 'o'},
                                             {DsgLayers::PLACES, 'p'},
                                             {DsgLayers::ROOMS, 'r'},
                                             {DsgLayers::BUILDINGS, 'b'}};

  SharedDsgInfo::Ptr frontend_dsg(new SharedDsgInfo(layer_id_map, mesh_layer_id));
  SharedDsgInfo::Ptr backend_dsg(new SharedDsgInfo(layer_id_map, mesh_layer_id));

  run(nh, frontend_dsg, backend_dsg, dsg_output_path);

  if (!dsg_output_path.empty()) {
    LOG(INFO) << "[DSG Node] saving timing information to " << dsg_output_path;
    const ElapsedTimeRecorder& timer = ElapsedTimeRecorder::instance();
    timer.logAllElapsed(dsg_output_path);
    timer.logStats(dsg_output_path);
    LOG(INFO) << "[DSG Node] Saved timing information to " << dsg_output_path;
  }

  return 0;
}
