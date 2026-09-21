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
#pragma once
#include <config_utilities/validation.h>
#include <hydra/common/message_queue.h>
#include <ianvs/node_handle.h>

#include "hydra_ros/utils/qos_config.h"

namespace hydra {

struct MessageSyncQueueConfig {
  //! Max queue depth
  size_t queue_size = 10;
  //! QoS settings for subscribers
  QoSConfig qos = rclcpp::SensorDataQoS();
};

template <typename MsgT>
class MessageSyncQueue {
 public:
  using Config = MessageSyncQueueConfig;
  using MsgPtr = typename MsgT::ConstPtr;

  MessageSyncQueue(const Config& config,
                   ianvs::NodeHandle nh,
                   const std::string& topic);

  MsgPtr sync(uint64_t timestamp_ns);

  const Config config;

 protected:
  MessageQueue<MsgPtr> messages_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Subscription<MsgT>::SharedPtr sub_;
};

template <typename MsgT>
MessageSyncQueue<MsgT>::MessageSyncQueue(const Config& config,
                                         ianvs::NodeHandle nh,
                                         const std::string& topic)
    : config(config::checkValid(config)),
      messages_(config.queue_size),
      group_(nh.as<rclcpp::node_interfaces::NodeBaseInterface>()->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive)),
      sub_(nh.create_subscription<MsgT>(
          topic,
          config.qos,
          [this](const MsgPtr& msg) { messages_.push(msg); },
          group_)) {}

template <typename MsgT>
auto MessageSyncQueue<MsgT>::sync(uint64_t timestamp_ns) -> MsgPtr {
  const auto pending = messages_.size();
  for (size_t i = 0; i < pending; ++i) {
    const rclcpp::Time stamp(messages_.front()->header.stamp);
    const uint64_t stamp_ns = stamp.nanoseconds();
    if (stamp_ns > timestamp_ns) {
      return nullptr;
    }

    const auto curr_msg = messages_.pop();
    if (stamp_ns == timestamp_ns) {
      return curr_msg;
    }
  }

  return nullptr;
}

void declare_config(MessageSyncQueueConfig& config);

}  // namespace hydra
