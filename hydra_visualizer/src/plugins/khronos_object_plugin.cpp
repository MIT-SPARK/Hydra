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

// Adapted from Khronos, original notice replicated below:
/** -----------------------------------------------------------------------------
 * Copyright (c) 2024 Massachusetts Institute of Technology.
 * All Rights Reserved.
 *
 * AUTHORS:      Lukas Schmid <lschmid@mit.edu>, Marcus Abate <mabate@mit.edu>,
 *               Yun Chang <yunchang@mit.edu>, Luca Carlone <lcarlone@mit.edu>
 * AFFILIATION:  MIT SPARK Lab, Massachusetts Institute of Technology
 * YEAR:         2024
 * SOURCE:       https://github.com/MIT-SPARK/Khronos
 * LICENSE:      BSD 3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------------- */

#include "hydra_visualizer/plugins/khronos_object_plugin.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/validation.h>
#include <spark_dsg/colormaps.h>
#include <spark_dsg/node_symbol.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include "hydra_visualizer/color/color_parsing.h"
#include "hydra_visualizer/drawing.h"

namespace hydra {
namespace {
static const auto registration =
    config::RegistrationWithConfig<VisualizerPlugin,
                                   KhronosObjectPlugin,
                                   KhronosObjectPlugin::Config,
                                   ianvs::NodeHandle,
                                   std::string>("KhronosObjectPlugin");
}  // namespace

namespace colormaps = spark_dsg::colormaps;
using spark_dsg::Color;
using spark_dsg::DynamicSceneGraph;
using spark_dsg::KhronosObjectAttributes;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

void declare_config(KhronosObjectPlugin::Config& config) {
  using namespace config;
  name("KhronosObjectPlugin");
  field(config.queue_size, "queue_size");
  enum_field(
      config.dynamic_color_mode, "dynamic_color_mode", {"ID", "SEMANTIC", "CONSTANT"});
  field(config.id_color_revolutions, "id_color_revolutions");
  field(config.dynamic_color, "dynamic_color");
  field(config.dynamic_bbox_scale, "dynamic_bbox_scale", "m");
  field(config.layer, "layer");

  check(config.queue_size, GE, 0, "queue_size");
  check(config.id_color_revolutions, GT, 0, "id_color_revolutions");
  check(config.dynamic_bbox_scale, GT, 0, "dynamic_bbox_scale");
}

KhronosObjectPlugin::KhronosObjectPlugin(const Config& config,
                                         ianvs::NodeHandle nh,
                                         const std::string& name)
    : VisualizerPlugin(name),
      config_("khronos_object_plugin", config::checkValid(config)),
      dynamic_pub_(
          nh.create_publisher<MarkerArray>("dynamic_objects", config.queue_size)),
      static_pub_(nh.create_publisher<kimera_pgmo_msgs::msg::Mesh>("static_objects",
                                                                   config.queue_size)),
      tf_broadcaster_(nh.node()) {}

void KhronosObjectPlugin::draw(const std_msgs::msg::Header& header,
                               const DynamicSceneGraph& graph) {
  const auto config = config_.get();
  if (!graph.hasLayer(config.layer)) {
    return;
  }
  drawDynamicObjects(config, header, graph);
  drawStaticObjects(config, header, graph);
}

void KhronosObjectPlugin::reset(const std_msgs::msg::Header& header) {
  // Reset dynamic markers.
  MarkerArray dynamic_msg;
  tracker_.clearPrevious(header, dynamic_msg);
  if (!dynamic_msg.markers.empty()) {
    dynamic_pub_->publish(dynamic_msg);
  }

  // Reset static meshes.
  for (const auto& id : previous_objects_) {
    resetObject(header, id);
  }
  previous_objects_.clear();
}

void KhronosObjectPlugin::drawDynamicObjects(const Config& config,
                                             const std_msgs::msg::Header& header,
                                             const DynamicSceneGraph& graph) {
  if (dynamic_pub_->get_subscription_count() == 0) {
    return;
  }

  // For dynamic objects show the start and end bbox with a line for the trajectory.
  const auto& objects = graph.getLayer(config.layer);
  MarkerArray msg;

  for (const auto& [node_id, node] : objects.nodes()) {
    const auto attrs = node->tryAttributes<KhronosObjectAttributes>();
    if (!attrs || attrs->trajectory_positions.empty()) {
      continue;
    }

    const uint64_t id = spark_dsg::NodeSymbol(node_id).categoryId();
    spark_dsg::BoundingBox bbox = attrs->bounding_box;
    const auto color = visualizer::makeColorMsg(getDynamicColor(config, *attrs, id));

    Marker bbox_template;
    bbox_template.type = Marker::LINE_LIST;
    bbox_template.action = Marker::ADD;
    bbox_template.color = color;
    bbox_template.scale.x = config.dynamic_bbox_scale;
    bbox_template.scale.y = config.dynamic_bbox_scale;
    bbox_template.scale.z = config.dynamic_bbox_scale;
    bbox_template.pose.orientation.w = 1.f;
    bbox_template.header = header;

    // Start bbox.
    bbox.world_P_center = attrs->trajectory_positions.front();
    auto marker = bbox_template;
    marker.id = id;
    marker.ns = "start_bbox";
    visualizer::drawBoundingBox(bbox, color, marker);
    tracker_.add(marker, msg);

    bbox.world_P_center = attrs->trajectory_positions.back();
    auto& marker2 = bbox_template;
    marker2.id = id;
    marker2.ns = "end_bbox";
    visualizer::drawBoundingBox(bbox, color, marker2);
    tracker_.add(marker2, msg);

    // Add the trajectory.
    Marker line;
    line.action = Marker::ADD;
    line.color = color;
    line.header = header;
    line.id = id;
    line.ns = "trajectory";
    line.type = Marker::LINE_STRIP;
    line.points.reserve(attrs->trajectory_positions.size());
    line.scale.x = 0.05;
    line.pose.orientation.w = 1.f;
    for (const auto& point : attrs->trajectory_positions) {
      tf2::convert(point.cast<double>().eval(), line.points.emplace_back());
    }
    tracker_.add(line, msg);
  }

  // Clear objects that are no longer present.
  tracker_.clearPrevious(header, msg);
  dynamic_pub_->publish(msg);
}

void KhronosObjectPlugin::drawStaticObjects(const Config& config,
                                            const std_msgs::msg::Header& header,
                                            const DynamicSceneGraph& dsg) {
  if (static_pub_->get_subscription_count() == 0) {
    return;
  }

  std::unordered_set<uint64_t> present_objects;
  const auto& objects = dsg.getLayer(config.layer);
  for (const auto& [node_id, node] : objects.nodes()) {
    const auto attrs = node->tryAttributes<KhronosObjectAttributes>();
    if (!attrs) {
      continue;
    }
    const uint64_t id = spark_dsg::NodeSymbol(node_id).categoryId();

    // Always update the transform.
    publishTransform(header, *attrs, id);
    present_objects.emplace(id);

    // Publish the mesh.
    // TODO(lschmid): In the future consider porting the object mesh colorings, not used
    // for now I think though.
    const hydra::MeshColoring::Ptr coloring = nullptr;
    auto msg = visualizer::makeMeshMsg(header, attrs->mesh, getNamespace(id), coloring);
    msg.header = header;
    msg.header.frame_id = getFrameName(id);
    static_pub_->publish(msg);
  }

  // Delete objects no longer present.
  for (auto it = previous_objects_.begin(); it != previous_objects_.end();) {
    if (!present_objects.count(*it)) {
      resetObject(header, *it);
      it = previous_objects_.erase(it);
    } else {
      ++it;
    }
  }
  previous_objects_.insert(present_objects.begin(), present_objects.end());
}

std::string KhronosObjectPlugin::getNamespace(const uint64_t id) {
  return std::to_string(spark_dsg::NodeSymbol('O', id));
}

std::string KhronosObjectPlugin::getFrameName(const uint64_t id) {
  return "khronos_object_" + std::to_string(id);
}

void KhronosObjectPlugin::resetObject(const std_msgs::msg::Header& header,
                                      const uint64_t id) {
  kimera_pgmo_msgs::msg::Mesh msg;
  msg.header = header;
  msg.ns = getNamespace(id);
  static_pub_->publish(msg);
}

void KhronosObjectPlugin::publishTransform(const std_msgs::msg::Header& header,
                                           const KhronosObjectAttributes& attrs,
                                           const uint64_t id) {
  geometry_msgs::msg::TransformStamped msg;
  msg.header = header;
  msg.child_frame_id = getFrameName(id);
  // The khronos meshes are in bounding box coordinates.
  msg.transform.translation.x = attrs.bounding_box.world_P_center.x();
  msg.transform.translation.y = attrs.bounding_box.world_P_center.y();
  msg.transform.translation.z = attrs.bounding_box.world_P_center.z();
  const Eigen::Quaternionf quat(attrs.bounding_box.world_R_center);
  msg.transform.rotation.w = quat.w();
  msg.transform.rotation.x = quat.x();
  msg.transform.rotation.y = quat.y();
  msg.transform.rotation.z = quat.z();
  tf_broadcaster_.sendTransform(msg);
}

Color KhronosObjectPlugin::getDynamicColor(const Config& config,
                                           const KhronosObjectAttributes& attrs,
                                           const uint64_t id) const {
  switch (config.dynamic_color_mode) {
    case Config::DynamicColorMode::ID:
      return colormaps::rainbowId(id, config.id_color_revolutions);
    case Config::DynamicColorMode::SEMANTIC:
      return colormaps::distinct150Id(attrs.semantic_label);
    case Config::DynamicColorMode::CONSTANT:
      return config.dynamic_color;
    default:
      return Color::gray();
  }
}

YAML::Node KhronosObjectPlugin::dumpConfig() const {
  auto root = config::toYaml(config_.get());
  root["type"] = "KhronosObjectPlugin";
  return root;
}

}  // namespace hydra
