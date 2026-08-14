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
#include <hydra/common/dsg_types.h>
#include <hydra/common/global_info.h>
#include <hydra/frontend/graph_builder.h>
#include <hydra/loop_closure/loop_closure_module.h>
#include <pose_graph_tools_ros/conversions.h>

#include <cstdint>
#include <memory>

#include "hydra_ros/backend/ros_backend_publisher.h"
#include "hydra_ros/frontend/ros_frontend_publisher.h"
#include "hydra_ros/utils/bow_subscriber.h"
#include "hydra_ros/utils/external_loop_closure_subscriber.h"
#include "hydra_ros/utils/status_monitor.h"

namespace hydra {

void declare_config(HydraRosPipeline::Config& config) {
  using namespace config;
  name("HydraRosConfig");
  field(config.active_window, "active_window");
  field(config.frontend, "frontend");
  field(config.backend, "backend");
  field(config.enable_frontend_output, "enable_frontend_output");
  field(config.enable_zmq_interface, "enable_zmq_interface");
  field(config.input, "input");
  config.features.setOptional();
  field(config.features, "features");
  field(config.verbosity, "verbosity");
  field(config.preprint_config, "preprint_config");
  field(config.status_monitor, "status_monitor");
}

HydraRosPipeline::HydraRosPipeline(int robot_id, int config_verbosity)
    : HydraPipeline(config::fromContext<PipelineConfig>(), robot_id, config_verbosity),
      config(config::checkValid(config::fromContext<Config>())) {
  if (config.preprint_config) {
    LOG(INFO) << "Using configuration to start Hydra\n" << config::toString(config);
  } else {
    LOG_IF(INFO, config.verbosity >= 1)
        << "Starting Hydra-ROS with input configuration\n"
        << config::toString(config.input);
  }
}

HydraRosPipeline::~HydraRosPipeline() {}

void HydraRosPipeline::init() {
  const auto& pipeline_config = GlobalInfo::instance().getConfig();

  auto nh = ianvs::NodeHandle::this_node("~");
  backend_ = config.backend.create(backend_dsg_, shared_state_);
  modules_["backend"] = CHECK_NOTNULL(backend_);

  frontend_ = config.frontend.create(frontend_dsg_, shared_state_);
  modules_["frontend"] = CHECK_NOTNULL(frontend_);

  active_window_ = config.active_window.create(frontend_->queue());
  modules_["active_window"] = CHECK_NOTNULL(active_window_);

  if (pipeline_config.enable_lcd) {
    initLCD();
    bow_sub_.reset(new BowSubscriber(nh));
  }

  status_monitor_ = std::make_unique<StatusMonitor>(config.status_monitor, nh);
  external_loop_closure_sub_.reset(new ExternalLoopClosureSubscriber(nh));

  auto bnh = nh / "backend";
  backend_->addSink(std::make_shared<RosBackendPublisher>(bnh));
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

  // TODO(nathan) make optional config
  if (config.enable_zmq_interface) {
    const auto zmq_config = config::fromContext<ZmqSink::Config>("backend/zmq_sink");
    backend_->addSink(std::make_shared<ZmqSink>(zmq_config));
  }

  if (config.enable_frontend_output) {
    CHECK(frontend_) << "Frontend module required!";
    frontend_->addSink(std::make_shared<RosFrontendPublisher>(nh / "frontend"));
  }

  input_module_ =
      std::make_shared<RosInputModule>(config.input, active_window_->queue());
  if (config.features) {
    modules_["features"] = config.features.create();  // has to come after input module
  }
}

void HydraRosPipeline::start() {
  HydraPipeline::start();
  status_monitor_->start();
}

void HydraRosPipeline::stop() {
  // TODO(nathan) log remaining queue sizes here or in stop
  // enforce stop order to make sure every data packet is processed
  input_module_->stop();
  // TODO(nathan) push extracting active window objects to module stop
  active_window_->stop();
  frontend_->stop();
  backend_->stop();

  HydraPipeline::stop();
}

void HydraRosPipeline::initLCD() {
  // TODO(nathan) push to pipeline config?
  auto lcd_config = config::fromContext<LoopClosureConfig>();
  lcd_config.detector.num_semantic_classes =
      GlobalInfo::instance().labelspace().total_labels;
  LOG_IF(INFO, config.verbosity >= 2)
      << "Number of classes for LCD: " << lcd_config.detector.num_semantic_classes;

  config::checkValid(lcd_config);

  auto lcd = std::make_shared<LoopClosureModule>(lcd_config, shared_state_);
  modules_["lcd"] = lcd;
  // TODO(nathan) rework sensor-level LCD request
}

}  // namespace hydra
