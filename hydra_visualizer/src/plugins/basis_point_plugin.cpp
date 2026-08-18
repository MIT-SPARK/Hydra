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
#include "hydra_visualizer/plugins/basis_point_plugin.h"

#include <config_utilities/config_utilities.h>
#include <config_utilities/factory.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <spark_dsg/node_attributes.h>

#include <tf2_eigen/tf2_eigen.hpp>

using namespace spark_dsg;
using visualization_msgs::msg::Marker;

namespace hydra {
namespace {

static const auto registration =
    config::RegistrationWithConfig<LayerPlugin,
                                   BasisPointPlugin,
                                   BasisPointPlugin::Config,
                                   std::string>("BasisPointPlugin");

struct BasisPoint {
  Eigen::Vector3d pos;
  Color color;
};

std::vector<BasisPoint> getBasisPoints(const visualizer::CategoricalColormap& cmap,
                                       const PlaceNodeAttributes& attrs,
                                       bool use_voxblox,
                                       const spark_dsg::Mesh* vertices) {
  std::vector<BasisPoint> to_return;
  if (!use_voxblox && !vertices) {
    return to_return;
  }

  if (use_voxblox) {
    for (const auto& info : attrs.voxblox_mesh_connections) {
      auto& point = to_return.emplace_back();
      point.pos = Eigen::Map<const Eigen::Vector3d>(info.voxel_pos);
      if (info.label) {
        point.color = cmap.getColor(*info.label);
      }
    }
  } else {
    for (const auto idx : attrs.pcl_mesh_connections) {
      auto& point = to_return.emplace_back();
      point.pos = vertices->pos(idx).cast<double>();
      point.color = vertices->colors.at(idx);
    }
  }

  return to_return;
}

}  // namespace

void declare_config(BasisPointPlugin::Config& config) {
  using namespace config;
  name("BasisPlugin::Config");
  field(config.collapse, "collapse");
  field(config.use_voxblox, "use_voxblox");
  field(config.edge_scale, "edge_scale");
  field(config.edge_alpha, "edge_alpha");
  field(config.point_scale, "point_scale");
  field(config.point_alpha, "point_alpha");
  field(config.colormap, "colormap");
}

BasisPointPlugin::BasisPointPlugin(const Config& config, const std::string& ns)
    : LayerPlugin(),
      ns_(ns),
      config_(ns, config::checkValid(config), [this]() { has_change_ = true; }) {}

void BasisPointPlugin::draw(const std_msgs::msg::Header& header,
                            const visualizer::DrawingContext& context,
                            const spark_dsg::SceneGraphLayer& layer,
                            const spark_dsg::Mesh* mesh,
                            visualization_msgs::msg::MarkerArray& msg,
                            MarkerTracker& tracker) {
  const auto config = config_.get();
  const visualizer::CategoricalColormap colormap(config.colormap);
  if (!config.use_voxblox && !mesh) {
    return;
  }

  Marker edge_marker;
  edge_marker.header = header;
  edge_marker.type = Marker::LINE_LIST;
  edge_marker.action = Marker::ADD;
  edge_marker.id = 0;
  edge_marker.ns = "places_parent_edges";
  edge_marker.pose.orientation.w = 1.0;
  edge_marker.scale.x = config.edge_scale;
  edge_marker.color.a = config.edge_alpha;

  Marker bp_marker;
  bp_marker.header = header;
  bp_marker.type = Marker::CUBE_LIST;
  bp_marker.action = Marker::ADD;
  bp_marker.id = 0;
  bp_marker.ns = "places_parents";
  bp_marker.pose.orientation.w = 1.0;
  bp_marker.scale.x = config.point_scale;
  bp_marker.scale.y = config.point_scale;
  bp_marker.scale.z = config.point_scale;

  for (const auto& [node_id, node] : layer.nodes()) {
    const auto attrs = node->tryAttributes<PlaceNodeAttributes>();
    if (!attrs) {
      continue;
    }

    geometry_msgs::msg::Point start;
    tf2::convert(attrs->position, start);
    if (!config.collapse) {
      start.z += context.z_offset;
    }

    const auto points = getBasisPoints(colormap, *attrs, config.use_voxblox, mesh);
    for (const auto& basis_point : points) {
      auto pos = basis_point.pos;
      if (!config.collapse) {
        pos.z() += context.z_offset;
      }

      edge_marker.points.push_back(start);
      tf2::convert(pos, edge_marker.points.emplace_back());
      tf2::convert(pos, bp_marker.points.emplace_back());

      auto& color = bp_marker.colors.emplace_back();
      color = visualizer::makeColorMsg(basis_point.color, config.point_alpha);
    }
  }

  tracker.add(edge_marker, msg);
  tracker.add(bp_marker, msg);
}

YAML::Node BasisPointPlugin::dumpConfig() const {
  auto root = config::toYaml(config_.get());
  root["type"] = "BasisPointPlugin";
  return root;
}

}  // namespace hydra
