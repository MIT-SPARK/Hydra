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
#include "hydra_visualizer/plugins/frontier_plugin.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/validation.h>
#include <spark_dsg/node_attributes.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include "hydra_visualizer/color/color_parsing.h"

using hydra::visualizer::makeColorMsg;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using namespace spark_dsg;

namespace hydra {
namespace {

static const auto registration_ =
    config::RegistrationWithConfig<LayerPlugin,
                                   FrontierPlugin,
                                   FrontierPlugin::Config,
                                   std::string>("FrontierPlugin");

}  // namespace

void declare_config(FrontierPlugin::Config& config) {
  using namespace config;
  name("FrontierPlugin::Config");
  field(config.alpha, "alpha");
  checkInRange(config.alpha, 0.0, 1.0, "alpha");
}

FrontierPlugin::FrontierPlugin(const Config& config, const std::string& ns)
    : ns_(ns),
      config_(ns + "_frontier_plugin", config, [this]() { has_change_ = true; }) {}

void FrontierPlugin::draw(const std_msgs::msg::Header& header,
                          const visualizer::DrawingContext& info,
                          const SceneGraphLayer& layer,
                          const Mesh*,
                          MarkerArray& msg,
                          MarkerTracker& tracker) {
  const auto config = config_.get();
  const auto ns = ns_ + "_frontier_ellipses";

  size_t id = 0;
  MarkerArray markers;
  for (const auto& [node_id, node] : layer.nodes()) {
    const auto attrs = node->tryAttributes<PlaceNodeAttributes>();
    if (!attrs || attrs->real_place) {
      continue;
    }

    auto& marker = markers.markers.emplace_back();
    marker.header = header;
    marker.type = Marker::SPHERE;
    marker.action = Marker::ADD;
    marker.id = id++;
    marker.ns = ns;
    marker.scale.x = attrs->frontier_scale.x();
    marker.scale.y = attrs->frontier_scale.y();
    marker.scale.z = attrs->frontier_scale.z();

    tf2::convert(attrs->position, marker.pose.position);
    tf2::convert(attrs->orientation, marker.pose.orientation);

    marker.pose.position.z += info.z_offset;
    marker.color = makeColorMsg(info.node_color(*node), config.alpha);
    msg.markers.push_back(marker);
  }

  tracker.add(markers, msg);
}

YAML::Node FrontierPlugin::dumpConfig() const {
  auto root = config::toYaml(config_.get());
  root["type"] = "FrontierPlugin";
  return root;
}

}  // namespace hydra
