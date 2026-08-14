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
#include "hydra_ros/frontend/object_visualizer.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <hydra/common/global_info.h>
#include <hydra_visualizer/drawing.h>
#include <kimera_pgmo/mesh_delta.h>

namespace hydra {
namespace {

static const auto registration =
    config::RegistrationWithConfig<MeshSegmenter::Sink,
                                   ObjectVisualizer,
                                   ObjectVisualizer::Config>("ObjectVisualizer");

}

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

void declare_config(ObjectVisualizer::Config& config) {
  using namespace config;
  name("ObjectVisualizerConfig");
  field(config.module_ns, "module_ns");
  field(config.point_scale, "point_scale");
  field(config.point_alpha, "point_alpha");
  field(config.use_spheres, "use_spheres");
  field(config.bounding_box_scale, "bounding_box_scale");
  field(config.colormap, "colormap");
}

ObjectVisualizer::ObjectVisualizer(const Config& config)
    : config(config::checkValid(config)),
      nh_(ianvs::NodeHandle::this_node(config.module_ns)),
      pubs_(nh_),
      colormap_(config.colormap) {}

std::string ObjectVisualizer::printInfo() const { return config::toString(config); }

struct DeltaPointAdaptor : spark_dsg::BoundingBox::PointAdaptor {
  DeltaPointAdaptor(const kimera_pgmo::MeshDelta& delta,
                    const std::vector<size_t>& indices)
      : delta(delta), indices(indices) {}

  size_t size() const override { return indices.size(); }

  Eigen::Vector3f get(size_t index) const override {
    return delta.getVertex(indices.at(index)).pos;
  }
  const kimera_pgmo::MeshDelta& delta;
  const std::vector<size_t>& indices;
};

void ObjectVisualizer::call(uint64_t timestamp_ns,
                            const kimera_pgmo::MeshDelta& delta,
                            const LabelIndices& label_indices,
                            const MeshSegmenter::LabelClusters& clusters) const {
  pubs_.publish("active_vertices", [&]() {
    auto arr = std::make_unique<MarkerArray>();
    auto& msg = arr->markers.emplace_back();
    msg.header.stamp = rclcpp::Time(timestamp_ns);
    msg.header.frame_id = GlobalInfo::instance().getFrames().odom;
    msg.ns = "active_vertices";
    msg.id = 0;
    std::vector<size_t> active(delta.getNumActiveVertices());
    std::iota(active.begin(), active.end(), delta.getNumArchivedVertices());
    fillMarkerFromCloud(delta, active, msg);
    return arr;
  });

  pubs_.publish("label_vertices", [&]() {
    auto arr = std::make_unique<MarkerArray>();
    for (const auto& [label, indices] : label_indices) {
      auto& msg = arr->markers.emplace_back();
      msg.header.stamp = rclcpp::Time(timestamp_ns);
      msg.header.frame_id = GlobalInfo::instance().getFrames().odom;
      msg.ns = "label_vertices_" + std::to_string(label);
      msg.id = 0;
      fillMarkerFromCloud(delta, indices, msg);
    }

    return arr;
  });

  pubs_.publish("object_clusters", [&]() {
    auto arr = std::make_unique<MarkerArray>();
    auto& msg = arr->markers.emplace_back();
    msg.header.stamp = rclcpp::Time(timestamp_ns);
    msg.header.frame_id = GlobalInfo::instance().getFrames().odom;
    msg.type = Marker::LINE_LIST;
    msg.action = Marker::ADD;
    msg.ns = "object_clusters";
    msg.id = 0;
    msg.scale.x = config.bounding_box_scale;
    for (const auto& [label, label_clusters] : clusters) {
      const auto color = visualizer::makeColorMsg(colormap_(label));
      for (const auto& cluster : label_clusters) {
        const DeltaPointAdaptor adaptor(delta, cluster.indices);
        const spark_dsg::BoundingBox bbox(adaptor);
        visualizer::drawBoundingBox(bbox, color, msg);
      }
    }

    return arr;
  });
}

void ObjectVisualizer::fillMarkerFromCloud(const kimera_pgmo::MeshDelta& delta,
                                           const std::vector<size_t>& indices,
                                           Marker& msg) const {
  msg.type = config.use_spheres ? Marker::SPHERE_LIST : Marker::CUBE_LIST;
  msg.action = Marker::ADD;

  msg.scale.x = config.point_scale;
  msg.scale.y = config.point_scale;
  msg.scale.z = config.point_scale;
  msg.pose.orientation.w = 1.0;

  msg.points.reserve(indices.size());
  msg.colors.reserve(indices.size());
  for (const auto idx : indices) {
    const auto& p = delta.getVertex(idx);
    auto& point = msg.points.emplace_back();
    point.x = p.pos.x();
    point.y = p.pos.y();
    point.z = p.pos.z();
    auto& color = msg.colors.emplace_back();
    color.r = p.traits.color[0] / 255.0f;
    color.g = p.traits.color[1] / 255.0f;
    color.b = p.traits.color[2] / 255.0f;
    color.a = config.point_alpha;
  }
}

}  // namespace hydra
