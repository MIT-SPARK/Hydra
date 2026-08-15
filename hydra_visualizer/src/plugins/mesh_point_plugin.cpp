#include "hydra_visualizer/plugins/mesh_point_plugin.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/yaml.h>

#include "hydra_visualizer/color/colormap_utilities.h"

namespace hydra {

using spark_dsg::ObjectNodeAttributes;
using spark_dsg::Place2dNodeAttributes;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace {

static const auto registration =
    config::RegistrationWithConfig<LayerPlugin,
                                   MeshPointPlugin,
                                   MeshPointPlugin::Config,
                                   std::string>("MeshPointPlugin");

void fillMarker(const MeshPointPlugin::Config& config,
                const visualizer::LayerInfo& info,
                const Place2dNodeAttributes& attrs,
                const spark_dsg::Mesh& mesh,
                const spark_dsg::Color& node_color,
                Marker& marker) {
  for (const auto idx : attrs.mesh_connections) {
    const auto& pos = mesh.pos(idx);
    auto& point = marker.points.emplace_back();
    point.x = pos.x();
    point.y = pos.y();
    point.z = pos.z() + info.z_offset;

    auto& color = marker.colors.emplace_back();
    color.a = config.alpha;
    if (!config.use_node_color && mesh.has_colors) {
      visualizer::fillColorMsg(mesh.color(idx), color);
    } else {
      visualizer::fillColorMsg(node_color, color);
    }
  }
}

void fillMarker(const MeshPointPlugin::Config& config,
                const visualizer::LayerInfo& info,
                const ObjectNodeAttributes& attrs,
                const spark_dsg::Mesh& mesh,
                const spark_dsg::Color& node_color,
                Marker& marker) {
  for (const auto idx : attrs.mesh_connections) {
    const auto& pos = mesh.pos(idx);
    auto& point = marker.points.emplace_back();
    point.x = pos.x();
    point.y = pos.y();
    point.z = pos.z() + info.z_offset;

    auto& color = marker.colors.emplace_back();
    color.a = config.alpha;
    if (!config.use_node_color && mesh.has_colors) {
      visualizer::fillColorMsg(mesh.color(idx), color);
    } else {
      visualizer::fillColorMsg(node_color, color);
    }
  }
}

}  // namespace

void declare_config(MeshPointPlugin::Config& config) {
  using namespace config;
  name("MeshPointPlugin::Config");
  field(config.point_size, "point_size");
  field(config.use_spheres, "use_spheres");
  field(config.use_node_color, "use_node_color");
  field(config.alpha, "alpha");
  check(config.point_size, GT, 0.0, "point_size");
  checkInRange(config.alpha, 0.0, 1.0, "alpha");
}

MeshPointPlugin::MeshPointPlugin(const Config& config, const std::string& ns)
    : ns_(ns),
      config_(ns + "_mesh_point_plugin", config, [this]() { has_change_ = true; }) {}

void MeshPointPlugin::draw(const std_msgs::msg::Header& header,
                           const visualizer::LayerInfo& info,
                           const spark_dsg::SceneGraphLayer& layer,
                           const spark_dsg::Mesh* mesh,
                           MarkerArray& msg,
                           MarkerTracker& tracker) {
  if (!mesh) {
    return;
  }

  const auto config = config_.get();

  Marker marker;
  marker.header = header;
  marker.type = config.use_spheres ? Marker::SPHERE_LIST : Marker::CUBE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = ns_ + "_mesh_points";
  marker.scale.x = config.point_size;
  marker.scale.y = config.point_size;
  marker.scale.z = config.point_size;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;

  marker.points.reserve(layer.numNodes());
  marker.colors.reserve(layer.numNodes());
  for (const auto& [node_id, node] : layer.nodes()) {
    if (info.filter && !info.filter(*node)) {
      continue;
    }

    const auto color = info.node_color(*node);
    auto obj_attrs = node->tryAttributes<ObjectNodeAttributes>();
    if (obj_attrs) {
      fillMarker(config, info, *obj_attrs, *mesh, color, marker);
      continue;
    }

    auto place_attrs = node->tryAttributes<Place2dNodeAttributes>();
    if (place_attrs) {
      fillMarker(config, info, *place_attrs, *mesh, color, marker);
      continue;
    }
  }

  tracker.add(marker, msg);
}

YAML::Node MeshPointPlugin::dumpConfig() const {
  auto root = config::toYaml(config_.get());
  root["type"] = "MeshPointPlugin";
  return root;
}

}  // namespace hydra
