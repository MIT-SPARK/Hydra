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
#pragma once

#include <config_utilities/virtual_config.h>
#include <config_utilities_ros/ros_dynamic_config_server.h>
#include <ianvs/node_handle.h>

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>

#include "hydra_visualizer/io/graph_wrapper.h"
#include "hydra_visualizer/plugins/visualizer_plugin.h"
#include "hydra_visualizer/scene_graph_renderer.h"

namespace hydra {

class DsgVisualizer {
 public:
  struct Config {
    double loop_period_s = 0.1;
    SceneGraphRenderer::Config renderer;
    config::VirtualConfig<GraphWrapper> graph;
    // Specify additional plugins that should be loaded <name, config>
    std::map<std::string, config::VirtualConfig<VisualizerPlugin, true>> plugins;
  } const config;

  //! Construct the visualizer from a config
  DsgVisualizer(const Config& config, ianvs::NodeHandle nh);

  ~DsgVisualizer() = default;

  //! Loop and redraw when changes occur
  void start();

  //! Delete all currently published visualization artifacts and remake graph
  void reset();

  //! Add a new graph plugin
  void addPlugin(VisualizerPlugin::Ptr plugin);

  //! Delete all current plugins
  void clearPlugins();

 private:
  void spinOnce(bool force = false);

  void saveConfigs(const std::filesystem::path& output);

  ianvs::NodeHandle nh_;
  rclcpp::TimerBase::SharedPtr loop_timer_;

  GraphWrapper::Ptr graph_;
  SceneGraphRenderer::Ptr renderer_;
  std::vector<VisualizerPlugin::Ptr> plugins_;
  const config::RosDynamicConfigServer server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr redraw_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr save_sub_;
};

void declare_config(DsgVisualizer::Config& config);

}  // namespace hydra
