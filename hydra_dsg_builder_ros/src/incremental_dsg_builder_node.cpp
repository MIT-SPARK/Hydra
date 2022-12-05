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

#include <hydra_dsg_builder/incremental_dsg_lcd.h>
#include <hydra_topology/ros_reconstruction.h>
#include <hydra_utils/ros_utilities.h>
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

namespace hydra {

struct HydraRosConfig {
  bool enable_lcd = false;
  bool use_ros_backend = false;
  bool do_reconstruction = true;
  bool enable_frontend_output = true;
  double frontend_mesh_separation_s = 0.0;
};

template <typename Visitor>
void visit_config(const Visitor& v, HydraRosConfig& config) {
  v.visit("enable_lcd", config.enable_lcd);
  v.visit("use_ros_backend", config.use_ros_backend);
  v.visit("do_reconstruction", config.do_reconstruction);
  v.visit("enable_frontend_output", config.enable_frontend_output);
  v.visit("frontend_mesh_separation_s", config.frontend_mesh_separation_s);
}

}  // namespace hydra

DECLARE_CONFIG_OSTREAM_OPERATOR(hydra, HydraRosConfig)

namespace hydra {
DECLARE_STRUCT_NAME(HydraRosConfig);
DECLARE_STRUCT_NAME(DsgFrontendConfig);
DECLARE_STRUCT_NAME(DsgBackendConfig);
DECLARE_STRUCT_NAME(kimera_pgmo::KimeraPgmoConfig);
DECLARE_STRUCT_NAME(DsgLcdModuleConfig);
}  // namespace hydra

struct HydraRosPipeline {
  explicit HydraRosPipeline(const ros::NodeHandle& nh, int robot_id = 0)
      : prefix(robot_id) {
    config_ = load_config<hydra::HydraRosConfig>(nh);

    // TODO(nathan) parse and use at some point
    const LayerId mesh_layer_id = 1;
    const std::map<LayerId, char> layer_id_map{{DsgLayers::OBJECTS, 'o'},
                                               {DsgLayers::PLACES, 'p'},
                                               {DsgLayers::ROOMS, 'r'},
                                               {DsgLayers::BUILDINGS, 'b'}};

    frontend_dsg.reset(new SharedDsgInfo(layer_id_map, mesh_layer_id));
    backend_dsg.reset(new SharedDsgInfo(layer_id_map, mesh_layer_id));
    shared_state.reset(new SharedModuleState());

    if (config_.do_reconstruction) {
      const auto frontend_config = load_config<DsgFrontendConfig>(nh);
      frontend = std::make_shared<DsgFrontend>(
          prefix, frontend_config, frontend_dsg, shared_state);
      reconstruction =
          std::make_shared<RosReconstruction>(nh, prefix, frontend->getQueue());
    } else {
      frontend = std::make_shared<RosFrontend>(nh, prefix, frontend_dsg, shared_state);
    }

    if (config_.enable_frontend_output) {
      ros::NodeHandle frontend_nh(nh, "frontend");
      dsg_sender_.reset(new DsgSender(
          frontend_nh, "frontend", true, config_.frontend_mesh_separation_s));
      frontend->addOutputCallback(std::bind(&HydraRosPipeline::sendFrontendGraph,
                                            this,
                                            std::placeholders::_1,
                                            std::placeholders::_2));
    }

    const auto backend_config = load_config<DsgBackendConfig>(nh);
    if (config_.use_ros_backend) {
      backend = std::make_shared<RosBackend>(
          nh, prefix, frontend_dsg, backend_dsg, shared_state);
    } else {
      const auto pgmo_config = load_config<kimera_pgmo::KimeraPgmoConfig>(nh, "pgmo");
      backend = std::make_shared<DsgBackend>(
          prefix, backend_config, pgmo_config, frontend_dsg, backend_dsg, shared_state);
    }

    backend_visualizer =
        std::make_shared<RosBackendVisualizer>(nh, backend_config, prefix);
    backend->addOutputCallback(std::bind(&RosBackendVisualizer::publishOutputs,
                                         backend_visualizer.get(),
                                         std::placeholders::_1,
                                         std::placeholders::_2,
                                         std::placeholders::_3));

    if (config_.enable_lcd) {
      const auto lcd_config = load_config<DsgLcdModuleConfig>(nh, "");
      shared_state->lcd_queue.reset(new InputQueue<LcdInput::Ptr>());
      lcd.reset(new DsgLcd(prefix, lcd_config, frontend_dsg, shared_state));
    }
  }

  void start() {
    if (reconstruction) {
      reconstruction->start();
    }

    if (frontend) {
      frontend->start();
    }

    if (backend) {
      backend->start();
    }

    if (lcd) {
      lcd->start();
    }
  }

  void stop() {
    if (reconstruction) {
      reconstruction->stop();
    }

    if (frontend) {
      frontend->stop();
    }

    if (lcd) {
      lcd->stop();
    }

    if (backend) {
      backend->stop();
    }
  }

  void save(const std::string& output_path) {
    if (!output_path.empty()) {
      if (reconstruction) {
        reconstruction->save(output_path + "/topology/");
      }

      if (frontend) {
        frontend->save(output_path + "/frontend/");
      }

      if (backend) {
        backend->save(output_path + "/backend/");
      }

      if (lcd) {
        lcd->save(output_path + "/lcd/");
      }
    }
  }

  void sendFrontendGraph(const DynamicSceneGraph& graph, uint64_t timestamp_ns) {
    ros::Time stamp;
    stamp.fromNSec(timestamp_ns);
    dsg_sender_->sendGraph(graph, stamp);
  }

  hydra::HydraRosConfig config_;
  RobotPrefixConfig prefix;
  SharedDsgInfo::Ptr frontend_dsg;
  SharedDsgInfo::Ptr backend_dsg;
  SharedModuleState::Ptr shared_state;

  std::shared_ptr<ReconstructionModule> reconstruction;
  std::shared_ptr<DsgFrontend> frontend;
  std::shared_ptr<DsgBackend> backend;
  std::shared_ptr<RosBackendVisualizer> backend_visualizer;
  std::shared_ptr<DsgLcd> lcd;
  std::unique_ptr<DsgSender> dsg_sender_;
};

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

  nh.getParam("timing_disabled", ElapsedTimeRecorder::instance().timing_disabled);
  nh.getParam("disable_timer_output", ElapsedTimeRecorder::instance().disable_output);

  bool log_timing_incrementally = false;
  nh.getParam("log_timing_incrementally", log_timing_incrementally);
  if (log_timing_incrementally && dsg_output_path != "") {
    ElapsedTimeRecorder::instance().setupIncrementalLogging(dsg_output_path);
  }

  int robot_id = 0;
  nh.getParam("robot_id", robot_id);

  HydraRosPipeline hydra(nh, robot_id);
  hydra.start();

  const auto exit_mode = getExitMode(nh);
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

  hydra.stop();
  hydra.save(dsg_output_path);

  if (!dsg_output_path.empty()) {
    LOG(INFO) << "[DSG Node] saving timing information to " << dsg_output_path;
    const ElapsedTimeRecorder& timer = ElapsedTimeRecorder::instance();
    timer.logAllElapsed(dsg_output_path);
    timer.logStats(dsg_output_path);
    LOG(INFO) << "[DSG Node] Saved timing information to " << dsg_output_path;
  }

  return 0;
}
