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
#include "hydra_ros/frontend/keyframe_visualizer.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/types/conversions.h>
#include <config_utilities/validation.h>
#include <hydra/common/global_info.h>
#include <hydra/input/camera.h>
#include <hydra_visualizer/color/color_parsing.h>
#include <hydra_visualizer/color/colormap_utilities.h>

#include <array>
#include <cmath>
#include <tf2_eigen/tf2_eigen.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace hydra {
namespace {

static const auto registration =
    config::RegistrationWithConfig<KeyframeSelector::Sink,
                                   KeyframeVisualizer,
                                   KeyframeVisualizer::Config>("KeyframeVisualizer");

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

using Corners = std::array<Eigen::Vector3d, 4>;

Corners getFrustumCorners(const Sensor& sensor, double depth) {
  double hfov = M_PI / 2.0;  // roughly 3/4 aspect ratio for 90 degree fov
  double vfov = 3.0 * M_PI / 8.0;
  const auto camera = dynamic_cast<const Camera*>(&sensor);
  if (camera) {
    const auto& config = camera->getConfig();
    hfov = 2.0 * std::atan2(0.5 * config.width, config.fx);
    vfov = 2.0 * std::atan2(0.5 * config.height, config.fy);
  }

  // order is top left, top right, bottom left, bottom right
  const auto half_width = depth * std::tan(0.5 * hfov);
  const auto half_height = depth * std::tan(0.5 * vfov);
  Corners corners;
  if (camera) {
    corners[0] = Eigen::Vector3d(-half_width, -half_height, depth);
    corners[1] = Eigen::Vector3d(half_width, -half_height, depth);
    corners[2] = Eigen::Vector3d(half_width, half_height, depth);
    corners[3] = Eigen::Vector3d(-half_width, half_height, depth);
  } else {
    corners[0] = Eigen::Vector3d(depth, half_width, half_height);
    corners[1] = Eigen::Vector3d(depth, -half_width, half_height);
    corners[2] = Eigen::Vector3d(depth, half_width, -half_height);
    corners[3] = Eigen::Vector3d(depth, -half_width, -half_height);
  }

  return corners;
}

void fillEdges(const Corners& corners, const Eigen::Vector3d& origin, Marker& edges) {
  // outline of image plane
  tf2::convert(corners[0], edges.points.emplace_back());
  tf2::convert(corners[1], edges.points.emplace_back());
  tf2::convert(corners[1], edges.points.emplace_back());
  tf2::convert(corners[2], edges.points.emplace_back());
  tf2::convert(corners[2], edges.points.emplace_back());
  tf2::convert(corners[3], edges.points.emplace_back());
  tf2::convert(corners[3], edges.points.emplace_back());
  tf2::convert(corners[0], edges.points.emplace_back());
  // origin to corners
  tf2::convert(origin, edges.points.emplace_back());
  tf2::convert(corners[0], edges.points.emplace_back());
  tf2::convert(origin, edges.points.emplace_back());
  tf2::convert(corners[1], edges.points.emplace_back());
  tf2::convert(origin, edges.points.emplace_back());
  tf2::convert(corners[2], edges.points.emplace_back());
  tf2::convert(origin, edges.points.emplace_back());
  tf2::convert(corners[3], edges.points.emplace_back());
}

void fillPlanes(const Corners& corners, Marker& planes, bool both_sides) {
  // first side
  tf2::convert(corners[0], planes.points.emplace_back());
  tf2::convert(corners[1], planes.points.emplace_back());
  tf2::convert(corners[2], planes.points.emplace_back());
  tf2::convert(corners[0], planes.points.emplace_back());
  tf2::convert(corners[2], planes.points.emplace_back());
  tf2::convert(corners[3], planes.points.emplace_back());
  if (both_sides) {
    tf2::convert(corners[3], planes.points.emplace_back());
    tf2::convert(corners[2], planes.points.emplace_back());
    tf2::convert(corners[0], planes.points.emplace_back());
    tf2::convert(corners[2], planes.points.emplace_back());
    tf2::convert(corners[1], planes.points.emplace_back());
    tf2::convert(corners[0], planes.points.emplace_back());
  }
}

MarkerArray drawCameraFrustums(const KeyframeVisualizer::Config& config,
                               const std::list<InputData::ConstPtr>& frames) {
  MarkerArray msg;
  if (frames.empty()) {
    return msg;
  }

  const auto with_image_plane = config.image_plane_alpha > 0.0;
  msg.markers.resize(with_image_plane ? 2 : 1);
  auto& edges = msg.markers[0];
  edges.ns = "keyframe_edges";
  edges.id = 0;
  edges.type = Marker::LINE_LIST;
  edges.action = Marker::ADD;
  edges.color = visualizer::makeColorMsg(config.color, 1.0);
  edges.scale.x = config.line_width;
  edges.points.reserve(16 * frames.size());

  Marker* planes = nullptr;
  if (with_image_plane) {
    const size_t num_points = config.draw_both_image_plane_sides ? 12 : 6;
    planes = &msg.markers[1];
    planes->ns = "keyframe_image_planes";
    planes->id = 0;
    planes->type = Marker::TRIANGLE_LIST;
    planes->action = Marker::ADD;
    planes->color = visualizer::makeColorMsg(config.color, config.image_plane_alpha);
    planes->scale.x = 1.0;
    planes->scale.y = 1.0;
    planes->scale.z = 1.0;
    planes->points.reserve(num_points * frames.size());
  }

  for (const auto& frame : frames) {
    const auto corners = getFrustumCorners(frame->getSensor(), config.far_distance);
    const auto pose = frame->getSensorPose();

    Corners curr_corners;
    curr_corners[0] = pose * corners[0];
    curr_corners[1] = pose * corners[1];
    curr_corners[2] = pose * corners[2];
    curr_corners[3] = pose * corners[3];

    fillEdges(curr_corners, pose.translation().cast<double>(), edges);
    if (planes) {
      fillPlanes(curr_corners, *planes, config.draw_both_image_plane_sides);
    }
  }

  return msg;
}

}  // namespace

void declare_config(KeyframeVisualizer::Config& config) {
  using namespace config;
  name("KeyframeVisualizer::Config");
  field(config.ns, "ns");
  field(config.far_distance, "far_distance");
  field(config.line_width, "line_width");
  field(config.color, "color");
  field(config.image_plane_alpha, "image_plane_alpha");
  field(config.draw_both_image_plane_sides, "draw_both_image_plane_sides");
  check(config.far_distance, GT, 0.0, "far_distance");
  check(config.line_width, GT, 0.0, "line_width");
  checkInRange(config.image_plane_alpha, 0.0, 1.0, "image_plane_alpha");
}

KeyframeVisualizer::KeyframeVisualizer(const Config& config)
    : nh_(ianvs::NodeHandle::this_node(config.ns)),
      pubs_(nh_),
      config_("keyframe_visualizer", config) {}

void KeyframeVisualizer::call(uint64_t timestamp_ns,
                              const KeyframeSelector::Keyframes& frames) const {
  std_msgs::msg::Header header;
  header.frame_id = GlobalInfo::instance().getFrames().odom;
  header.stamp = rclcpp::Time(timestamp_ns);
  pubs_.publish("frustums", header, [&]() -> MarkerArray {
    return drawCameraFrustums(config_.get(), frames);
  });
}

}  // namespace hydra
