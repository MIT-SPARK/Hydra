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
#include "hydra_visualizer/groundtruth/bounding_box_publisher.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/printing.h>
#include <config_utilities/types/eigen_matrix.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

#include <algorithm>

#include <tf2_eigen/tf2_eigen.hpp>

#include "hydra_visualizer/color/colormap_utilities.h"
#include "hydra_visualizer/drawing.h"

namespace {

// intentionally anonymized and not a convert struct to avoid duplicate definitions

inline YAML::Node toYaml(const Eigen::Vector3f& v) {
  YAML::Node node;
  node.push_back(v.x());
  node.push_back(v.y());
  node.push_back(v.z());
  return node;
}

inline bool fromYaml(const YAML::Node& node, Eigen::Vector3f& v) {
  if (!node || !node.IsSequence() || node.size() != 3) {
    return false;
  }

  v = Eigen::Vector3f(node[0].as<float>(), node[1].as<float>(), node[2].as<float>());
  return true;
}

inline YAML::Node toYaml(const Eigen::Quaternionf& q) {
  YAML::Node node;
  node["w"] = q.w();
  node["x"] = q.x();
  node["y"] = q.y();
  node["z"] = q.z();
  return node;
}

inline bool fromYaml(const YAML::Node& node, Eigen::Quaternionf& q) {
  if (!node["w"] || !node["x"] || !node["y"] || !node["z"]) {
    return false;
  }

  q = Eigen::Quaternionf(node["w"].as<float>(),
                         node["x"].as<float>(),
                         node["y"].as<float>(),
                         node["z"].as<float>());
  return true;
}

}  // namespace

namespace YAML {

template <>
struct convert<spark_dsg::BoundingBox> {
  static Node encode(const spark_dsg::BoundingBox& rhs) {
    Node node;
    node["center"] = toYaml(rhs.world_P_center);
    node["extents"] = toYaml(rhs.dimensions);
    Eigen::Quaternionf q(rhs.world_R_center);
    node["rotation"] = toYaml(q);
    return node;
  }

  static bool decode(const Node& node, spark_dsg::BoundingBox& rhs) {
    Eigen::Vector3f center;
    if (!fromYaml(node["center"], center)) {
      return false;
    }

    Eigen::Vector3f extents;
    if (!fromYaml(node["extents"], extents)) {
      return false;
    }

    Eigen::Quaternionf rotation;
    if (!fromYaml(node["rotation"], rotation)) {
      return false;
    }

    rhs = spark_dsg::BoundingBox(center, extents, rotation);
    return true;
  }
};

}  // namespace YAML

namespace hydra {

using spark_dsg::BoundingBox;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace {

// TODO(nathan) clean this up
std::string printBoxes(const BoundingBoxPublisher::Annotations& annotations) {
  std::stringstream ss;
  ss << "{";
  for (const auto& [label, boxes] : annotations) {
    ss << label << ": [";
    for (const auto& box : boxes) {
      ss << box;
    }
    ss << "]";
  }
  ss << "}";
  return ss.str();
}

double maxHeight(const BoundingBoxPublisher::Annotations& annotations) {
  double max_z = 0.0;
  for (const auto& [label, boxes] : annotations) {
    for (const auto& box : boxes) {
      const double curr_z = box.corners().at(6).z();
      max_z = std::max(curr_z, max_z);
    }
  }

  return max_z;
}
}  // namespace

void declare_config(BoundingBoxPublisher::Config& config) {
  using namespace config;
  name("BoundingBoxPublisher::Config");
  field<Path>(config.filepath, "filepath");
  field(config.frame_id, "frame_id");
  field(config.marker_ns, "marker_ns");
  field(config.draw_labels, "draw_labels");
  field(config.scale, "scale");
  field(config.text_scale, "text_scale");
  field(config.text_line_scale, "text_line_scale");
  field(config.text_height, "text_height");
  field(config.alpha, "alpha");
}

BoundingBoxPublisher::BoundingBoxPublisher(const rclcpp::NodeOptions& options)
    : Node("bounding_box_publisher", options),
      config(config::checkValid(config)),
      pub_(create_publisher<MarkerArray>("~/gt_markers",
                                         rclcpp::QoS(1).transient_local())),
      marker_ns_(config.marker_ns.empty() ? get_effective_namespace()
                                          : config.marker_ns),
      colormap_(config.colormap) {
  using namespace std::placeholders;
  std_msgs::msg::String msg;
  msg.data = config.filepath.string();
  load(msg);
  sub_ = create_subscription<std_msgs::msg::String>(
      "load", 1, std::bind(&BoundingBoxPublisher::load, this, _1));
}

void BoundingBoxPublisher::drawBoxes(const std_msgs::msg::Header& header,
                                     const Annotations& annotations,
                                     MarkerArray& msg) const {
  Marker marker;
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.id = 0;
  marker.ns = marker_ns_;
  marker.scale.x = config.scale;
  marker.pose.orientation.w = 1.0;

  size_t idx = 0;
  for (auto&& [label, boxes] : annotations) {
    const auto color = visualizer::makeColorMsg(colormap_.getColor(idx), config.alpha);
    ++idx;

    for (const auto& box : boxes) {
      visualizer::drawBoundingBox(box, color, marker);
    }
  }

  tracker_.add(marker, msg);
}

void BoundingBoxPublisher::drawLabels(const std_msgs::msg::Header& header,
                                      const Annotations& annotations,
                                      MarkerArray& msg) const {
  // TODO(nathan) line to bbox center
  size_t idx = 0;
  const auto max_z = maxHeight(annotations);
  for (auto&& [label, boxes] : annotations) {
    for (const auto& box : boxes) {
      Marker marker;
      marker.header = header;
      marker.ns = marker_ns_ + "_labels";
      marker.id = idx;
      marker.type = Marker::TEXT_VIEW_FACING;
      marker.action = Marker::ADD;
      marker.pose.position.x = box.world_P_center.x();
      marker.pose.position.y = box.world_P_center.y();
      marker.pose.position.z = box.world_P_center.z() + config.text_height;
      if (config.text_height > 0.0) {
        marker.pose.position.z += max_z;
      }

      marker.pose.orientation.w = 1.0;
      marker.scale.z = config.text_scale;
      marker.color.a = 1.0;
      marker.text = label;
      tracker_.add(marker, msg);
      ++idx;

      /*      if (config.draw_labels && config.text_height > 0.0) {*/
      /*geometry_msgs::Point center_point;*/
      /*tf2::convert(pos, center_point);*/
      /*center_point.z += max_z + config.text_height;*/
      /*}*/
    }
  }
}

void BoundingBoxPublisher::load(const std_msgs::msg::String& msg) {
  const std::filesystem::path filepath(msg.data);
  if (!std::filesystem::exists(filepath)) {
    LOG(WARNING) << "Filepath '" << filepath.string() << "' does not exist!";
    return;
  }

  const auto node = YAML::LoadFile(msg.data);
  const auto boxes = node.as<Annotations>();
  VLOG(3) << "Loaded bounding boxes from '" << msg.data << "':\n" << printBoxes(boxes);

  std_msgs::msg::Header header;
  header.stamp = get_clock()->now();
  header.frame_id = config.frame_id;

  MarkerArray markers;
  drawBoxes(header, boxes, markers);
  if (config.draw_labels) {
    drawBoxes(header, boxes, markers);
  }

  tracker_.clearPrevious(header, markers);
  pub_->publish(markers);
}

}  // namespace hydra
