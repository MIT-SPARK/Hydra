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
#include <config_utilities/config.h>
#include <config_utilities/external_registry.h>
#include <config_utilities/formatting/asl.h>
#include <config_utilities/logging/log_to_spdlog.h>
#include <config_utilities/parsing/context.h>
#include <config_utilities/printing.h>
#include <config_utilities/types/path.h>
#include <hydra/utils/logging.h>
#include <ianvs/node_init.h>
#include <ianvs/spin_functions.h>

#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "hydra_ros/hydra_ros_pipeline.h"
#include "hydra_ros/utils/node_plugins.h"
#include "hydra_ros/utils/ros_log_sink.h"

namespace hydra {

struct NodeSettings {
  bool show_run_settings = true;
  bool exit_after_clock = false;
  bool force_shutdown = false;
  std::vector<std::string> paths;
  bool forward_logs_to_ros = true;
  std::string log_level = "info";

  bool use_multithread_executor = true;
  std::vector<config::VirtualConfig<AppPlugin, true>> node_plugins = {
      config::VirtualConfig<AppPlugin, true>{ConfigServerPlugin::Config{}}};
};

void declare_config(NodeSettings& config) {
  using namespace config;
  name("NodeSettings");
  field(config.show_run_settings, "show_run_settings");
  field(config.exit_after_clock, "exit_after_clock");
  field(config.force_shutdown, "force_shutdown");
  field(config.paths, "paths");
  field(config.forward_logs_to_ros, "forward_logs_to_ros");
  field(config.log_level, "log_level");

  field(config.use_multithread_executor, "use_multithread_executor");
  field(config.node_plugins, "node_plugins");
}

}  // namespace hydra

int main(int argc, char* argv[]) {
  config::initContext(argc, argv, true);
  config::setConfigSettingsFromContext();
  const auto settings = config::fromContext<hydra::NodeSettings>();

  hydra::logging::setLogLevel(settings.log_level);

  [[maybe_unused]] const auto node = ianvs::init_node(argc, argv, "hydra_ros_node");
  auto nh = ianvs::NodeHandle::this_node();

  std::shared_ptr<hydra::RosSink> ros_sink;
  if (settings.forward_logs_to_ros) {
    ros_sink = std::make_shared<hydra::RosSink>(nh.logger());
    hydra::logging::setSinks({ros_sink});
  }

  config::internal::Logger::setLogger(
      std::make_shared<config::internal::SpdlogLogger>(hydra::logging::logger()));
  if (settings.show_run_settings) {
    HYDRA_LOG(INFO, "Using node settings\n{}", config::toString(settings));
  }

  [[maybe_unused]] const auto plugins = config::loadExternalFactories(settings.paths);

  rclcpp::executors::MultiThreadedExecutor executor;
  {  // start hydra scope
    std::vector<std::unique_ptr<hydra::AppPlugin>> node_plugins;
    for (const auto& plugin : settings.node_plugins) {
      node_plugins.push_back(plugin.create());
    }

    const auto run_config = config::fromContext<hydra::RunContext::Config>();
    const auto context = std::make_shared<hydra::RunContext>(run_config);
    HYDRA_LOG(INFO, "Loaded run config:\n{}", config::toString(context->config));

    auto config = config::fromContext<hydra::HydraRosPipeline::Config>();
    hydra::HydraRosPipeline hydra(config, context);
    hydra.init();

    hydra.start();
    ianvs::spinAndWait(nh,
                       settings.exit_after_clock,
                       settings.use_multithread_executor ? &executor : nullptr);
    hydra.stop(settings.force_shutdown);
    hydra.save();
  }  // end hydra scope

  if (ros_sink) {
    hydra::logging::setLogToStderr();
  }

  rclcpp::shutdown();
  return 0;
}
