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
#include "hydra_ros/utils/qos_config.h"

#include <config_utilities/config.h>
#include <config_utilities/types/enum.h>

namespace hydra {
namespace {

size_t depth_from_policy(const rclcpp::QoS& qos) {
  if (qos.history() == rclcpp::HistoryPolicy::KeepAll) {
    return 0;
  }

  return qos.depth();
}

}  // namespace

QoSConfig::QoSConfig(const rclcpp::QoS& qos)
    : depth(depth_from_policy(qos)),
      reliability(qos.reliability()),
      durability(qos.durability()),
      liveliness(qos.liveliness()) {}

QoSConfig::QoSConfig(size_t depth,
                     rclcpp::ReliabilityPolicy reliability,
                     rclcpp::DurabilityPolicy durability,
                     rclcpp::LivelinessPolicy liveliness)
    : depth(depth),
      reliability(reliability),
      durability(durability),
      liveliness(liveliness) {}

QoSConfig::operator rclcpp::QoS() const {
  rclcpp::QoS qos(rclcpp::KeepAll{});
  if (depth > 0) {
    qos = rclcpp::QoS(rclcpp::KeepLast(depth));
  }

  return qos.reliability(reliability).durability(durability).liveliness(liveliness);
}

void declare_config(QoSConfig& config) {
  using namespace config;
  name("QoSConfig");
  field(config.depth, "depth");
  enum_field(config.reliability,
             "reliability",
             {{rclcpp::ReliabilityPolicy::BestEffort, "BestEffort"},
              {rclcpp::ReliabilityPolicy::Reliable, "Reliable"},
              {rclcpp::ReliabilityPolicy::SystemDefault, "SystemDefault"},
              {rclcpp::ReliabilityPolicy::BestAvailable, "BestAvailable"}});
  enum_field(config.durability,
             "durability",
             {{rclcpp::DurabilityPolicy::Volatile, "Volatile"},
              {rclcpp::DurabilityPolicy::TransientLocal, "TransientLocal"},
              {rclcpp::DurabilityPolicy::SystemDefault, "SystemDefault"},
              {rclcpp::DurabilityPolicy::BestAvailable, "BestAvailable"}});
  enum_field(config.liveliness,
             "liveliness",
             {{rclcpp::LivelinessPolicy::Automatic, "Auotmatic"},
              {rclcpp::LivelinessPolicy::ManualByTopic, "ManualByTopic"},
              {rclcpp::LivelinessPolicy::SystemDefault, "SystemDefault"},
              {rclcpp::LivelinessPolicy::BestAvailable, "BestAvailable"}});
}

}  // namespace hydra
