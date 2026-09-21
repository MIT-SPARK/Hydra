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

struct OptionalMessageSyncConfig {
  //! Max queue depth
  size_t queue_size = 10;
  //! QoS settings for subscribers
  QoSConfig qos =
      rclcpp::SensorDataQoS().reliability(rclcpp::ReliabilityPolicy::Reliable);
};

/**
 * @brief Class to add an optional field to a packets if it matches the timestamp
 *
 * Synchronization is tricky in this case; implementation assumes that messages come in
 * timestamp order and discards any received optional messages that take longer than the
 * period of received packets (a new packet will push the previously received one)
 *
 */
template <typename PacketT, typename MsgT>
class OptionalMessageSync {
 public:
  using Config = OptionalMessageSyncConfig;
  using MessagePtr = typename MsgT::ConstPtr;
  using PacketPtr = typename PacketT::Ptr;
  using Callback = std::function<void(const PacketPtr&, const MessagePtr&)>;

  OptionalMessageSync(const Config& config,
                      ianvs::NodeHandle nh,
                      const std::string& topic,
                      const Callback& callback);

  ~OptionalMessageSync();

  void push(const PacketT::Ptr& packet);

  const Config config;

 protected:
  void spin();

  std::atomic<bool> should_shutdown_;

  std::mutex mutex_;
  Callback callback_;
  MessageQueue<PacketPtr> packets_;
  MessageQueue<MessagePtr> messages_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Subscription<MsgT>::SharedPtr sub_;

  std::thread thread_;
};

template <typename PacketT, typename MsgT>
OptionalMessageSync<PacketT, MsgT>::OptionalMessageSync(const Config& config,
                                                        ianvs::NodeHandle nh,
                                                        const std::string& topic,
                                                        const Callback& callback)
    : config(config::checkValid(config)),
      should_shutdown_(false),
      callback_(callback),
      messages_(config.queue_size),
      group_(nh.as<rclcpp::node_interfaces::NodeBaseInterface>()->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive)),
      sub_(nh.create_subscription<MsgT>(
          topic,
          config.qos,
          [this](const MessagePtr& msg) { messages_.push_evict(msg); },
          group_)),
      thread_(&OptionalMessageSync<PacketT, MsgT>::spin, this) {}

template <typename PacketT, typename MsgT>
OptionalMessageSync<PacketT, MsgT>::~OptionalMessageSync() {
  should_shutdown_ = true;
  thread_.join();
}

template <typename PacketT, typename MsgT>
void OptionalMessageSync<PacketT, MsgT>::push(const PacketT::Ptr& packet) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (packets_.empty()) {
    // set the current packet for synchronization
    packets_.push(packet);
    return;
  }

  callback_(packets_.pop(), nullptr);
}

template <typename PacketT, typename MsgT>
void OptionalMessageSync<PacketT, MsgT>::spin() {
  while (!should_shutdown_) {
    if (!packets_.poll() || !messages_.poll()) {
      continue;  // don't do work if no messages
    }

    // lock mutex so that new push doesn't drop old packet while syncing
    std::lock_guard<std::mutex> lock(mutex_);
    if (packets_.empty()) {
      continue;  // packet popped during message poll
    }

    const auto timestamp_ns = packets_.front()->timestamp_ns;
    const auto pending = messages_.size();
    for (size_t i = 0; i < pending; ++i) {
      const rclcpp::Time stamp(messages_.front()->header.stamp);
      const uint64_t stamp_ns = stamp.nanoseconds();
      if (stamp_ns > timestamp_ns) {
        break;
      }

      const auto curr_msg = messages_.pop();
      if (stamp_ns == timestamp_ns) {
        const auto packet = packets_.pop();
        callback_(packet, curr_msg);
        break;  // found match, return
      }
    }
  }
}

void declare_config(OptionalMessageSyncConfig& config);

}  // namespace hydra
