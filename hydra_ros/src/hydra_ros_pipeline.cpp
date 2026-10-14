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
#include <glog/logging.h>
#include <hydra/active_window/active_window_module.h>
#include <hydra/common/global_info.h>
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
  base<HydraPipeline::Config>(config);
  field(config.preprint_config, "preprint_config");
  field(config.input, "input");
  field(config.status_monitor, "status_monitor");
}

HydraRosPipeline::HydraRosPipeline(const Config& config)
    : HydraPipeline(config, config::fromContext<PipelineConfig>()),
      config(config::checkValid(config)) {
  if (config.preprint_config) {
    LOG(INFO) << "Using configuration to start Hydra\n" << config::toString(config);
  }

  MLOG(1) << "Starting Hydra-ROS with input\n" << config::toString(config.input);
}

HydraRosPipeline::~HydraRosPipeline() {}

void HydraRosPipeline::init() {
  auto nh = ianvs::NodeHandle::this_node("~");
  backend_->addSink(std::make_shared<RosBackendPublisher>(nh / "backend"));
  frontend_->addSink(std::make_shared<RosFrontendPublisher>(nh / "frontend"));
  external_loop_closure_sub_.reset(new ExternalLoopClosureSubscriber(nh));

  using std::chrono::nanoseconds;
  monitor_ = std::make_unique<StatusMonitor>(config.status_monitor, nh);
  backend_->addSink(BackendModule::Sink::fromCallback(
      [this](uint64_t stamp, const auto&, const auto&) {
        monitor_->recordModuleCallback("backend", nanoseconds(stamp));
      }));

  active_window_->addSink(ActiveWindowModule::Sink::fromCallback(
      [this](uint64_t stamp, const auto&, const auto&) {
        monitor_->recordModuleCallback("active_window", nanoseconds(stamp));
      }));

  input_ = std::make_unique<RosInputModule>(config.input, active_window_->queue());
}

void HydraRosPipeline::start() {
  input_->start();
  HydraPipeline::start();
  monitor_->start();
}

void HydraRosPipeline::stop() {
  // enforce stop order to make sure every data packet is processed
  input_->stop();
  HydraPipeline::stop();
}

}  // namespace hydra
