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
#include "hydra_visualizer/plugins/mesh_plugin.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

#include "hydra_visualizer/drawing.h"

namespace hydra {
namespace {

static const auto registration =
    config::RegistrationWithConfig<VisualizerPlugin,
                                   MeshPlugin,
                                   MeshPlugin::Config,
                                   ianvs::NodeHandle,
                                   std::string>("MeshPlugin");

}

using MeshMsg = kimera_pgmo_msgs::msg::Mesh;

using config::checkValid;
using spark_dsg::DynamicSceneGraph;
using visualizer::makeMeshMsg;

void declare_config(MeshPlugin::Config& config) {
  using namespace config;
  name("MeshPlugin::Config");
  field(config.coloring, "coloring");
  field(config.min_mesh_separation_s, "min_mesh_separation_s", "s");
}

MeshPlugin::MeshPlugin(const Config& config,
                       ianvs::NodeHandle nh,
                       const std::string& name)
    : VisualizerPlugin(name),
      config_("mesh_plugin", checkValid(config), [this]() { has_change_ = true; }),
      mesh_pub_(nh.create_publisher<MeshMsg>(name, rclcpp::QoS(1).transient_local())) {}

MeshPlugin::~MeshPlugin() {}

void MeshPlugin::draw(const std_msgs::msg::Header& header,
                      const DynamicSceneGraph& graph) {
  auto mesh = graph.mesh();
  if (!mesh || mesh->empty()) {
    return;
  }

  const auto config = config_.get();
  const rclcpp::Time now(header.stamp);
  if (last_pub_ && (now - *last_pub_).seconds() < config.min_mesh_separation_s) {
    return;
  }

  last_pub_ = now;
  auto msg = makeMeshMsg(header, *mesh, getMsgNamespace(), config.coloring.create());
  mesh_pub_->publish(msg);
}

void MeshPlugin::reset(const std_msgs::msg::Header& header) {
  kimera_pgmo_msgs::msg::Mesh msg;
  msg.header = header;
  msg.ns = getMsgNamespace();
  mesh_pub_->publish(msg);
}

std::string MeshPlugin::getMsgNamespace() const {
  // TODO(lschmid): Hardcoded for now. Eventually read from scene graph or so.
  return "robot0/dsg_mesh";
}

YAML::Node MeshPlugin::dumpConfig() const {
  auto root = config::toYaml(config_.get());
  root["type"] = "MeshPlugin";
  return root;
}

}  // namespace hydra
