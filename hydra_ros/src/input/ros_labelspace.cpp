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
#include "hydra_ros/input/ros_labelspace.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/common/global_info.h>
#include <ianvs/message_wait_functor.h>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <semantic_inference_msgs/msg/labelspace.hpp>

namespace hydra {
namespace {

static const auto reg =
    config::RegistrationWithConfig<Labelspace, RosLabelspace, RosLabelspace::Config>(
        "from_msg");

}

using nlohmann::json;
using MsgType = semantic_inference_msgs::msg::Labelspace;

RosLabelspace::RosLabelspace(const Config& config) : config(config) {
  auto nh = ianvs::NodeHandle::this_node(config.ns);
  const auto resolved_topic = nh.resolve_name("labelspace", false);
  LOG(INFO) << "Waiting for labelspace on '" << resolved_topic << "'";

  std::optional<MsgType> maybe_msg;
  const auto start = nh.now();
  const auto qos = rclcpp::QoS(1).durability(
      config.latch_info_sub ? rclcpp::DurabilityPolicy::TransientLocal
                            : rclcpp::DurabilityPolicy::Volatile);
  const size_t timeout = std::floor(config.warning_timeout_s * 1000);
  while (!maybe_msg && rclcpp::ok()) {
    maybe_msg = ianvs::getSingleMessage<MsgType>(nh, "labelspace", true, qos, timeout);
    if (!maybe_msg) {
      LOG(WARNING) << "Waiting for labelspace on topic '" << resolved_topic << "'";
    }

    const auto diff = nh.now() - start;
    if (config.error_timeout_s && (diff.seconds() > config.error_timeout_s)) {
      LOG(ERROR) << "labelspace lookup timed out on '" << resolved_topic << "'";
      break;
    }
  }

  if (rclcpp::ok() && !maybe_msg) {
    LOG(FATAL) << "Failed to lookup labelspace on '" << resolved_topic << "'";
  }

  const auto& msg = *maybe_msg;
  total_labels = msg.names.size();
  invalid_labels.insert(msg.invalid.begin(), msg.invalid.end());
  for (size_t i = 0; i < msg.names.size(); ++i) {
    const auto& name = msg.names[i];
    if (name.empty()) {
      continue;
    }

    label_names[i] = name;
  }

  const auto metadata = json::parse(msg.metadata);
  if (metadata.count("object_labels")) {
    metadata.at("object_labels").get_to(object_labels);
  }

  if (metadata.count("dynamic_labels")) {
    metadata.at("dynamic_labels").get_to(dynamic_labels);
  }

  if (metadata.count("surface_places_labels")) {
    metadata.at("surface_places_labels").get_to(surface_places_labels);
  }

  LOG(INFO) << "Loaded labelspace from ros:\n"
            << config::toString(static_cast<const Labelspace&>(*this));
}

void declare_config(RosLabelspace::Config& config) {
  using namespace config;
  name("RosLabelspace::Config");
  field(config.ns, "ns");
  field(config.warning_timeout_s, "warning_timeout_s", "s");
  field(config.error_timeout_s, "error_timeout_s", "s");
  field(config.latch_info_sub, "latch_info_sub");
  check(config.warning_timeout_s, GE, 0.0, "warning_timeout_s");
  check(config.error_timeout_s, GE, 0.0, "error_timeout_s");
}

}  // namespace hydra
