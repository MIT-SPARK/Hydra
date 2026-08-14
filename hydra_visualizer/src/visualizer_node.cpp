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

#include "hydra_visualizer/visualizer_node.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

#include <fstream>

namespace hydra {

using std_msgs::msg::String;

void declare_config(DsgVisualizer::Config& config) {
  using namespace config;
  name("HydraVisualizerConfig");
  field(config.loop_period_s, "loop_period_s", "s");
  field(config.renderer, "renderer");
  field(config.graph, "graph");
  field(config.plugins, "plugins");
}

DsgVisualizer::DsgVisualizer(const Config& config, ianvs::NodeHandle nh)
    : config(config::checkValid(config)), nh_(nh), server_(nh_.node()) {
  renderer_ = std::make_shared<SceneGraphRenderer>(config.renderer, nh_);
  for (auto&& [name, plugin] : config.plugins) {
    plugins_.push_back(plugin.create(nh_, name));
  }

  graph_ = config.graph.create(nh_);
  // TODO(nathan) think about flagging change instead
  redraw_service_ = nh_.create_service<std_srvs::srv::Empty>(
      "redraw",
      [this](const std_srvs::srv::Empty::Request::SharedPtr&,
             std_srvs::srv::Empty::Response::SharedPtr) { spinOnce(true); });
  reset_service_ = nh_.create_service<std_srvs::srv::Empty>(
      "reset",
      [this](const std_srvs::srv::Empty::Request::SharedPtr&,
             std_srvs::srv::Empty::Response::SharedPtr) { reset(); });
  save_sub_ = nh_.create_subscription<String>(
      "save_config", 1, [this](const String& msg) { saveConfigs(msg.data); });
}

void DsgVisualizer::start() {
  // default chrono time unit is seconds...
  const std::chrono::duration<double> period_s(config.loop_period_s);
  loop_timer_ = nh_.create_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period_s), true, [this]() {
        spinOnce();
      });
}

void DsgVisualizer::reset() {
  std_msgs::msg::Header header;
  header.stamp = nh_.now();

  renderer_->reset(header);
  for (const auto& plugin : plugins_) {
    if (plugin) {
      plugin->reset(header);
    }
  }

  graph_ = config.graph.create(nh_);
}

void DsgVisualizer::addPlugin(VisualizerPlugin::Ptr plugin) {
  plugins_.push_back(std::move(plugin));
}

void DsgVisualizer::clearPlugins() { plugins_.clear(); }

void DsgVisualizer::spinOnce(bool force) {
  if (!graph_) {
    return;
  }

  bool has_change = false;
  has_change = graph_->hasChange();
  has_change |= renderer_->hasChange();
  for (const auto& plugin : plugins_) {
    if (plugin) {
      has_change |= plugin->hasChange();
    }
  }

  if (!has_change && !force) {
    return;
  }

  {  // start scope for stamped graph (for optional critical region)
    const auto stamped_graph = graph_->get();
    if (!stamped_graph) {
      return;
    }

    std_msgs::msg::Header header;
    header.frame_id = stamped_graph.frame_id;
    header.stamp = stamped_graph.timestamp.value_or(nh_.now());
    renderer_->draw(header, *stamped_graph.graph);
    for (const auto& plugin : plugins_) {
      if (plugin) {
        plugin->draw(header, *stamped_graph.graph);
      }
    }
  }  // end scope for stamped graph

  graph_->clearChangeFlag();
  renderer_->clearChangeFlag();
  for (auto& plugin : plugins_) {
    if (plugin) {
      plugin->clearChangeFlag();
    }
  }
}

void DsgVisualizer::saveConfigs(const std::filesystem::path& output) {
  YAML::Node plugins(YAML::NodeType::Map);
  for (const auto& plugin : plugins_) {
    if (!plugin) {
      continue;
    }

    plugins[plugin->name] = plugin->dumpConfig();
  }

  YAML::Node root;
  root["renderer"] = renderer_->dumpConfig();
  root["plugins"] = plugins;

  std::ofstream fout(output);
  fout << root;
  LOG(INFO) << "Saved config to " << output;
}

}  // namespace hydra
