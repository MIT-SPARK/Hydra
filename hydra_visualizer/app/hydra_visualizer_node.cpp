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
#include <config_utilities/config_utilities.h>
#include <config_utilities/external_registry.h>
#include <config_utilities/parsing/context.h>
#include <config_utilities/settings.h>
#include <ianvs/node_init.h>

#include <rclcpp/rclcpp.hpp>

#include "hydra_visualizer/visualizer_node.h"

namespace hydra::visualizer {

struct NodeSettings {
  std::vector<std::string> external_library_paths;
};

void declare_config(NodeSettings& config) {
  using namespace config;
  name("NodeSettings");
  field(config.external_library_paths, "external_library_paths");
}

}  // namespace hydra::visualizer

int main(int argc, char** argv) {
  config::initContext(argc, argv, true);
  config::setConfigSettingsFromContext();
  const auto node_settings = config::fromContext<hydra::visualizer::NodeSettings>();

  [[maybe_unused]] const auto guard =
      ianvs::init_node(argc, argv, "hydra_visualizer_node");
  auto nh = ianvs::NodeHandle::this_node("~");

  RCLCPP_DEBUG_STREAM(nh.logger(), "Settings:\n" << config::toString(node_settings));

  [[maybe_unused]] const auto plugins =
      config::loadExternalFactories(node_settings.external_library_paths);

  rclcpp::executors::MultiThreadedExecutor executor;
  {  // start visualizer scope
    const auto config = config::fromContext<hydra::DsgVisualizer::Config>();
    RCLCPP_DEBUG_STREAM(nh.logger(), "Config:\n" << config::toString(config));
    auto node = std::make_shared<hydra::DsgVisualizer>(config, nh);
    node->start();

    executor.add_node(nh.as<rclcpp::node_interfaces::NodeBaseInterface>());
    executor.spin();
  }  // end visualizer scope

  rclcpp::shutdown();
  return 0;
}
