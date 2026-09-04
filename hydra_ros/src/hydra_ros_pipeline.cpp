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
#include "hydra_ros/hydra_ros_pipeline.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/context.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <hydra/active_window/reconstruction_module.h>
#include <hydra/backend/backend_module.h>
#include <hydra/backend/zmq_interfaces.h>
#include <hydra/common/global_info.h>
#include <hydra/frontend/graph_builder.h>
#include <hydra/loop_closure/loop_closure_module.h>
#include <pose_graph_tools_ros/conversions.h>

#include <cstdint>
#include <memory>

#include "hydra_ros/backend/ros_backend_publisher.h"
#include "hydra_ros/frontend/ros_frontend_publisher.h"
#include "hydra_ros/utils/external_loop_closure_subscriber.h"
#include "hydra_ros/utils/status_monitor.h"

namespace hydra {

void declare_config(HydraRosPipeline::Config& config) {
  using namespace config;
  name("HydraRosConfig");
  base<VerbosityConfig>(config);
  field(config.preprint_config, "preprint_config");
  field(config.input, "input");
  field(config.active_window, "active_window");
  field(config.frontend, "frontend");
  field(config.backend, "backend");
  config.lcd.setOptional();
  field(config.lcd, "lcd");
  field(config.status_monitor, "status_monitor");
}

HydraRosPipeline::Config::Config()
    : VerbosityConfig(VerbosityConfig::default_verbosity("pipeline")) {}

HydraRosPipeline::HydraRosPipeline(int robot_id, int config_verbosity)
    : HydraPipeline(config::fromContext<PipelineConfig>(), robot_id, config_verbosity),
      config(config::checkValid(config::fromContext<Config>())) {
  if (config.preprint_config) {
    LOG(INFO) << "Using configuration to start Hydra\n" << config::toString(config);
  }

  MLOG(1) << "Starting Hydra-ROS with input configuration\n"
          << config::toString(config.input);
}

HydraRosPipeline::~HydraRosPipeline() {}

void HydraRosPipeline::init() {
  backend_ = config.backend.create(backend_dsg_, shared_state_);
  modules_["backend"] = CHECK_NOTNULL(backend_);

  frontend_ = config.frontend.create(frontend_dsg_, shared_state_);
  modules_["frontend"] = CHECK_NOTNULL(frontend_);

  active_window_ = config.active_window.create(frontend_->queue());
  modules_["active_window"] = CHECK_NOTNULL(active_window_);

  auto lcd = config.lcd.create(shared_state_);
  if (lcd) {
    frontend_->setLcdQueue(lcd->queue());
    modules_["lcd"] = std::move(lcd);
  }

  auto nh = ianvs::NodeHandle::this_node("~");
  backend_->addSink(std::make_shared<RosBackendPublisher>(nh / "backend"));
  frontend_->addSink(std::make_shared<RosFrontendPublisher>(nh / "frontend"));
  external_loop_closure_sub_.reset(new ExternalLoopClosureSubscriber(nh));

  status_monitor_ = std::make_unique<StatusMonitor>(config.status_monitor, nh);
  backend_->addSink(BackendModule::Sink::fromCallback(
      [this](uint64_t timestamp_ns, const auto&, const auto&) {
        status_monitor_->recordModuleCallback("backend",
                                              std::chrono::nanoseconds(timestamp_ns));
      }));

  active_window_->addSink(ActiveWindowModule::Sink::fromCallback(
      [this](uint64_t timestamp_ns, const auto&, const auto&) {
        status_monitor_->recordModuleCallback("active_window",
                                              std::chrono::nanoseconds(timestamp_ns));
      }));

  input_module_ =
      std::make_shared<RosInputModule>(config.input, active_window_->queue());
}

void HydraRosPipeline::start() {
  HydraPipeline::start();
  status_monitor_->start();
}

void HydraRosPipeline::stop() {
  // enforce stop order to make sure every data packet is processed
  input_module_->stop();
  active_window_->stop();
  frontend_->stop();
  backend_->stop();

  HydraPipeline::stop();
}

}  // namespace hydra
