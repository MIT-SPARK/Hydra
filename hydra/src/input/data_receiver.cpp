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
#include "hydra/input/data_receiver.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

#include <chrono>

namespace hydra {

DataReceiver::Config::Config()
    : VerbosityConfig(VerbosityConfig::default_verbosity("data_receiver")) {}

DataReceiver::DataReceiver(const Config& config, const std::string& _sensor_name)
    : config(config::checkValid(config)),
      sensor_name(_sensor_name),
      queue_(config.max_packets) {
  for (const auto& filter : config.filters) {
    filters_.push_back(filter.create());
  }
}

bool DataReceiver::init() { return initImpl(); }

SensorInputPacket::Ptr DataReceiver::poll() {
  while (!queue_.empty()) {
    const auto packet = pollOnce();
    if (packet) {
      return packet;
    }
  }

  return nullptr;
}

SensorInputPacket::Ptr DataReceiver::pollOnce() {
  if (queue_.empty()) {
    return nullptr;
  }

  const auto packet = queue_.pop();
  const auto timestamp = packet->timestamp_ns;
  const std::chrono::nanoseconds curr_time_ns(timestamp);
  if (last_received_) {
    std::chrono::nanoseconds last_time_ns(last_received_->timestamp_ns);
    std::chrono::duration<double> separation_s = curr_time_ns - last_time_ns;
    if (separation_s.count() < config.input_separation_s) {
      MLOG(3) << "Dropping input @ " << timestamp << " [ns] with separation of "
              << separation_s.count() << " [s]";
      return nullptr;
    }
  }

  for (const auto& filter : filters_) {
    if (!filter) {
      continue;
    }

    if (!filter->valid(*packet, last_received_.get())) {
      return nullptr;
    }
  }

  MLOG(2) << "Got input @ " << timestamp << " [ns]";
  last_received_ = packet;
  return last_received_;
}

void DataReceiver::clear() { queue_.clear(); }

size_t DataReceiver::numQueued() const { return queue_.size(); }

void declare_config(DataReceiver::Config& config) {
  using namespace config;
  name("DataReceiver::Config");
  base<VerbosityConfig>(config);
  field(config.max_packets, "max_packets");
  field(config.input_separation_s, "input_separation_s");
  field(config.filters, "filters");
}

}  // namespace hydra
