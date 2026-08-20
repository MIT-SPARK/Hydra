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
#include "hydra_visualizer/plugins/places_freespace_plugin.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <spark_dsg/node_attributes.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include "hydra_visualizer/color/color_parsing.h"

namespace hydra {
namespace {

inline static const auto registration_ =
    config::RegistrationWithConfig<LayerPlugin,
                                   PlacesFreespacePlugin,
                                   PlacesFreespacePlugin::Config,
                                   std::string>("PlacesFreespacePlugin");

}

using spark_dsg::Color;
using spark_dsg::PlaceNodeAttributes;
using visualization_msgs::msg::Marker;

void declare_config(PlacesFreespacePlugin::Config& config) {
  using namespace config;
  name("PlacesFreespacePlugin::Config");
  field(config.collapse, "collapse");
  field(config.sphere_color, "sphere_color");
  field(config.sphere_alpha, "sphere_alpha");
  checkInRange(config.sphere_alpha, 0.0, 1.0, "sphere_alpha", false);
}

PlacesFreespacePlugin::PlacesFreespacePlugin(const Config& config,
                                             const std::string& ns)
    : LayerPlugin(),
      config_(ns, config::checkValid(config), [this]() { has_change_ = true; }) {}

void PlacesFreespacePlugin::draw(const std_msgs::msg::Header& header,
                                 const visualizer::DrawingContext& context,
                                 const spark_dsg::SceneGraphLayer& layer,
                                 const spark_dsg::Mesh*,
                                 visualization_msgs::msg::MarkerArray& msg,
                                 MarkerTracker& tracker) {
  const auto config = config_.get();

  size_t id = 0;
  for (const auto& [node_id, node] : layer.nodes()) {
    const auto attrs = node->tryAttributes<PlaceNodeAttributes>();
    if (!attrs) {
      continue;
    }

    Marker marker;
    marker.header = header;
    marker.type = Marker::SPHERE;
    marker.action = Marker::ADD;
    marker.id = id;
    marker.ns = "places_spheres";

    marker.scale.x = 2.0 * attrs->distance;
    marker.scale.y = 2.0 * attrs->distance;
    marker.scale.z = 2.0 * attrs->distance;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    tf2::convert(attrs->position, marker.pose.position);
    if (!config.collapse) {
      marker.pose.position.z += context.z_offset;
    }

    marker.color = visualizer::makeColorMsg(config.sphere_color);
    marker.color.a = config.sphere_alpha;
    tracker.add(marker, msg);
    ++id;
  }
}

YAML::Node PlacesFreespacePlugin::dumpConfig() const {
  auto root = config::toYaml(config_.get());
  root["type"] = "PlacesFreespacePlugin";
  return root;
}

}  // namespace hydra
