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
#include "hydra_ros/input/ros_input_module.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/common/global_info.h>

#include "hydra_ros/input/ros_sensors.h"

namespace hydra {
namespace {

inline bool isNumber(const std::string& name) {
  return std::find_if(name.begin(), name.end(), [](char c) {
           return !std::isdigit(c);
         }) == name.end();
}

}  // namespace

void declare_config(RosInputModule::Config& config) {
  using namespace config;
  name("RosInputModule::Config");
  base<InputModule::Config>(config);
  field(config.tf_lookup, "tf_lookup");
  field(config.clear_queue_on_fail, "clear_queue_on_fail");
}

InputModule::Config RosInputModule::Config::remapSensors() const {
  InputModule::Config to_return;
  for (const auto& [name, input_pair] : inputs) {
    auto sensor_name = name;
    if (isNumber(sensor_name)) {
      // this happens when the yaml receiver list is specified without a name
      sensor_name = "sensor" + name;
      LOG(WARNING) << "Invalid ROS name found for sensor: '" << name
                   << "'. Remapping to '" << sensor_name << "'";
    }

    to_return.inputs[sensor_name] = input_pair;
    to_return.inputs[sensor_name].sensor =
        input::loadSensor(input_pair.sensor, sensor_name);
  }

  return to_return;
}

RosInputModule::RosInputModule(const Config& config, const OutputQueue::Ptr& queue)
    : InputModule(config.remapSensors(), queue),
      config(config),
      lookup_(config.tf_lookup),
      have_first_pose_(false) {}

RosInputModule::~RosInputModule() = default;

std::string RosInputModule::printInfo() const { return config::toString(config); }

PoseStatus RosInputModule::getBodyPose(uint64_t timestamp_ns) {
  const auto pose_status = lookup_.getBodyPose(timestamp_ns);
  if (pose_status && !have_first_pose_) {
    have_first_pose_ = true;
  }

  if (!pose_status && !have_first_pose_ && config.clear_queue_on_fail) {
    LOG(WARNING) << "Clearing input queues while pose is unavailable";
    for (auto& receiver : receivers_) {
      receiver->queue.clear();
    }
  }

  return pose_status;
}

}  // namespace hydra
