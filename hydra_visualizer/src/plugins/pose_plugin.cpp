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
#include "hydra_visualizer/plugins/pose_plugin.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>

#include <tf2_eigen/tf2_eigen.hpp>

namespace hydra {
namespace {

inline static const auto registration =
    config::RegistrationWithConfig<VisualizerPlugin,
                                   PosePlugin,
                                   PosePlugin::Config,
                                   ianvs::NodeHandle,
                                   std::string>("PosePlugin");

}

using geometry_msgs::msg::PoseArray;
using namespace spark_dsg;

void declare_config(PosePlugin::Config& config) {
  using namespace config;
  name("PosePlugin::Config");
  field(config.num_to_skip, "num_to_skip");
  field(config.layer, "layer");
  field(config.partition, "partition");
  field(config.num_received_before_warn, "num_received_before_warn");
}

PosePlugin::PosePlugin(const Config& config,
                       ianvs::NodeHandle nh,
                       const std::string& name)
    : VisualizerPlugin(name),
      config(config::checkValid(config)),
      num_received_(0),
      pub_(nh.create_publisher<PoseArray>(name, rclcpp::QoS(1).transient_local())) {}

void PosePlugin::draw(const std_msgs::msg::Header& header,
                      const DynamicSceneGraph& graph) {
  ++num_received_;
  PoseArray msg;
  msg.header = header;

  size_t num_seen = 0;
  size_t every_n_nodes = config.num_to_skip + 1;
  const auto layer_id = graph.getLayerKey(config.layer);
  if (!layer_id) {
    return;
  }

  const auto layer = graph.findLayer(layer_id->layer, config.partition);
  if (!layer) {
    const auto should_warn = num_received_ >= config.num_received_before_warn;
    LOG_IF(WARNING, should_warn)
        << "Missing layer '" << config.layer << "' and partition " << config.partition;
    return;
  }

  for (const auto& [node_id, node] : layer->nodes()) {
    if (num_seen % every_n_nodes != 0) {
      ++num_seen;
      continue;
    }

    ++num_seen;
    auto attrs = node->tryAttributes<AgentNodeAttributes>();
    if (!attrs) {
      LOG(WARNING) << "Node " << NodeSymbol(node_id).str()
                   << " does not have pose information!";
      continue;
    }

    auto& pose = msg.poses.emplace_back();
    pose.position = tf2::toMsg(attrs->position);
    pose.orientation = tf2::toMsg(attrs->world_R_body);
  }

  pub_->publish(msg);
}

void PosePlugin::reset(const std_msgs::msg::Header& header) {
  PoseArray msg;
  msg.header = header;
  pub_->publish(msg);
}

YAML::Node PosePlugin::dumpConfig() const {
  auto root = config::toYaml(config);
  root["type"] = "PosePlugin";
  return root;
}

}  // namespace hydra
