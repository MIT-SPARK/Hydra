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
#include "hydra_ros/input/ros_sensors.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/printing.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/common/global_info.h>
#include <ianvs/message_wait_functor.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "hydra_ros/utils/tf_lookup.h"

namespace hydra {

using sensor_msgs::msg::CameraInfo;

struct TempCameraConfig : Camera::Config {};

void declare_config(TempCameraConfig& config) {
  using namespace config;
  name("Sensor::Config");
  base<Sensor::Config>(config);
}

void fillConfigFromInfo(const CameraInfo& msg, Camera::Config& cam_config) {
  cam_config.width = msg.width;
  cam_config.height = msg.height;
  cam_config.fx = msg.k[0];
  cam_config.fy = msg.k[4];
  cam_config.cx = msg.k[2];
  cam_config.cy = msg.k[5];
}

std::optional<sensor_msgs::msg::CameraInfo> getCameraInfo(
    const RosCamera::Config& config, const std::string& ns) {
  auto nh = ianvs::NodeHandle::this_node(ns);
  const auto resolved_topic = nh.resolve_name("camera_info", false);
  LOG(INFO) << "Waiting for CameraInfo on " << resolved_topic
            << " to initialize sensor model";

  const auto start = nh.now();
  const auto qos = rclcpp::QoS(1).durability(
      config.latch_info_sub ? rclcpp::DurabilityPolicy::TransientLocal
                            : rclcpp::DurabilityPolicy::Volatile);
  const size_t timeout = std::floor(config.warning_timeout_s * 1000);

  std::optional<sensor_msgs::msg::CameraInfo> msg;
  while (!msg && rclcpp::ok()) {
    msg = ianvs::getSingleMessage<CameraInfo>(nh, "camera_info", true, qos, timeout);
    if (!msg) {
      LOG(WARNING) << "Waiting for CameraInfo on topic '" << resolved_topic << "'";
    }

    const auto diff = nh.now() - start;
    if (config.error_timeout_s && (diff.seconds() > config.error_timeout_s)) {
      LOG(ERROR) << "Sensor intrinsics lookup timed out on '" << resolved_topic << "'";
      break;
    }
  }

  return msg;
}

ParamSensorExtrinsics::Config lookupExtrinsics(const RosExtrinsics::Config& config,
                                               const std::string& sensor_frame,
                                               const std::string& robot_frame) {
  LOG(INFO) << "Looking for sensor extrinsics '" << robot_frame << "_T_" << sensor_frame
            << "' via TF";

  auto nh = ianvs::NodeHandle::this_node();
  auto clock = nh.node().get<rclcpp::node_interfaces::NodeClockInterface>();

  tf2_ros::Buffer buffer(clock->get_clock());
  tf2_ros::TransformListener listener(buffer);

  size_t warning_tries = config.warning_timeout_s / config.wait_duration_s;

  const auto start = nh.now();
  PoseStatus status;
  while (!status && rclcpp::ok()) {
    std::string message;
    status = lookupTransform(buffer,
                             std::nullopt,
                             robot_frame,
                             sensor_frame,
                             warning_tries,
                             config.wait_duration_s,
                             config.verbosity,
                             &message);
    if (!status) {
      LOG(WARNING) << "Waiting for sensor extrinsics " << message;
    }

    const auto diff = nh.now() - start;
    if (config.error_timeout_s && (diff.seconds() > config.error_timeout_s)) {
      LOG(ERROR) << "Sensor extrinsics lookup timed out for " << message;
      break;
    }
  }

  CHECK(status.is_valid) << "Could not look up extrinsics from ros!";

  ParamSensorExtrinsics::Config params;
  params.body_R_sensor = status.target_R_source;
  params.body_p_sensor = status.target_p_source;
  return params;
}

RosExtrinsics::RosExtrinsics(const Config&) {
  throw std::runtime_error("Cannot directly insantiate RosExtrinsics object!");
}

RosCamera::RosCamera(const Config&) {
  throw std::runtime_error("Cannot directly insantiate RosExtrinsics object!");
}

void declare_config(RosExtrinsics::Config& config) {
  using namespace config;
  name("RosExtrinsics::Config");
  field(config.sensor_frame, "sensor_frame");
  field(config.robot_frame, "robot_frame");
  field(config.warning_timeout_s, "warning_timeout_s", "s");
  field(config.error_timeout_s, "error_timeout_s", "s");
  field(config.wait_duration_s, "wait_duration_s", "s");
  field(config.verbosity, "verbosity");
  check(config.warning_timeout_s, GE, 0.0, "warning_timeout_s");
  check(config.error_timeout_s, GE, 0.0, "error_timeout_s");
}

void declare_config(RosCamera::Config& config) {
  using namespace config;
  name("RosCamera::Config");
  base<Sensor::Config>(config);
  field(config.ns, "ns");
  field(config.warning_timeout_s, "warning_timeout_s", "s");
  field(config.error_timeout_s, "error_timeout_s", "s");
  field(config.latch_info_sub, "latch_info_sub");
  check(config.warning_timeout_s, GE, 0.0, "warning_timeout_s");
  check(config.error_timeout_s, GE, 0.0, "error_timeout_s");
}

namespace input {

using VirtualSensor = config::VirtualConfig<Sensor>;

VirtualSensor loadExtrinsics(const VirtualSensor& sensor,
                             const std::string& sensor_frame = "") {
  if (!sensor) {
    return sensor;
  }

  auto base_contents = config::toYaml(sensor);
  const auto base_config = config::fromYaml<Sensor::Config>(base_contents);
  if (!base_config.extrinsics || base_config.extrinsics.getType() != "ros") {
    return sensor;
  }

  const auto contents = config::toYaml(base_config.extrinsics);
  const auto derived = config::fromYaml<RosExtrinsics::Config>(contents);
  const auto frame = derived.sensor_frame.empty() ? sensor_frame : derived.sensor_frame;
  const auto parent = derived.robot_frame.empty()
                          ? GlobalInfo::instance().getFrames().robot
                          : derived.robot_frame;
  if (frame.empty()) {
    LOG(ERROR) << "sensor frame required if not constructing from camera info!";
    return {};
  }

  const auto info = lookupExtrinsics(derived, frame, parent);
  config::VirtualConfig<SensorExtrinsics> new_config(info);
  base_contents["extrinsics"] = config::toYaml(new_config);
  return config::fromYaml<VirtualSensor>(base_contents);
}

VirtualSensor loadSensor(const VirtualSensor& sensor, const std::string& sensor_name) {
  if (!sensor || sensor.getType() != "camera_info") {
    return loadExtrinsics(sensor);
  }

  const auto contents = config::toYaml(sensor);
  const auto derived = config::fromYaml<RosCamera::Config>(contents);

  const auto ns =
      derived.ns.empty() ? "~/input/" + sensor_name + std::string("/rgb") : derived.ns;
  const auto msg = getCameraInfo(derived, ns);
  if (!msg) {
    return {};
  }

  // get base class fields (all other derived fields will be overriden by ros)
  Camera::Config config = config::fromYaml<TempCameraConfig>(contents);
  fillConfigFromInfo(*msg, config);

  VirtualSensor new_sensor(config);
  new_sensor = loadExtrinsics(new_sensor, msg->header.frame_id);
  LOG(INFO) << "Initialized camera as " << std::endl << config::toString(new_sensor);
  return new_sensor;
}

}  // namespace input
}  // namespace hydra
