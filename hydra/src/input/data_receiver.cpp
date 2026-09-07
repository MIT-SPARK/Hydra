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

#include "hydra/common/global_info.h"

namespace hydra {

void declare_config(DataReceiver::Config& config) {
  using namespace config;
  name("DataReceiver::Config");
  base<VerbosityConfig>(config);
  field(config.max_packets, "max_packets");
  field(config.input_separation_s, "input_separation_s");
  field(config.filters, "filters");
  field(config.adapters, "adapters");
  field(config.received_window_size, "received_window_size");
}

DataReceiver::Config::Config()
    : VerbosityConfig(VerbosityConfig::default_verbosity("data_receiver")) {}

DataReceiver::DataReceiver(const Config& config,
                           const Sensor::ConstPtr& sensor,
                           const OutputQueue::Ptr& output)
    : config(config::checkValid(config)),
      sensor(sensor),
      sensor_name(sensor->name),
      queue_(config.max_packets),
      output_queue_(output) {
  for (const auto& filter : config.filters) {
    filters_.push_back(filter.create());
  }

  for (const auto& adapter : config.adapters) {
    adapters_.push_back(adapter.create());
  }
}

DataReceiver::~DataReceiver() { stopImpl(); }

bool DataReceiver::start() {
  const auto success = initImpl();
  thread_ = std::make_unique<std::thread>(&DataReceiver::spin, this);
  return success;
}

void DataReceiver::stop() { stopImpl(); }

void DataReceiver::clear() { queue_.clear(); }

void DataReceiver::spin() {
  bool should_shutdown = false;
  while (!should_shutdown) {
    const auto has_data = queue_.poll();
    if (GlobalInfo::instance().force_shutdown() || !has_data) {
      should_shutdown = should_shutdown_;
    }

    if (!has_data) {
      continue;
    }

    const auto packet = queue_.pop();
    pushPacket(packet);
  }
}

void DataReceiver::recordTimestamp(uint64_t timestamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  received_window_.push_back(timestamp);
  if (received_window_.size() > config.received_window_size) {
    received_window_.pop_front();
  }
}

void DataReceiver::pushPacket(SensorInputPacket::Ptr packet) {
  const auto timestamp = packet->timestamp_ns;
  recordTimestamp(timestamp);

  const std::chrono::nanoseconds curr_time_ns(timestamp);
  if (last_received_) {
    std::chrono::nanoseconds last_time_ns(last_received_->timestamp_ns);
    std::chrono::duration<double> separation_s = curr_time_ns - last_time_ns;
    if (separation_s.count() < config.input_separation_s) {
      MLOG(3) << "Dropping input @ " << timestamp << " [ns] with separation of "
              << separation_s.count() << " [s]";
      return;
    }
  }

  for (const auto& filter : filters_) {
    if (filter && !filter->valid(*packet, last_received_.get())) {
      return;
    }
  }

  MLOG(2) << "Got input @ " << timestamp << " [ns]";
  last_received_ = packet;

  auto data = std::make_shared<InputData>(sensor);
  data->timestamp_ns = timestamp;
  packet->fillInputData(*data);
  for (const auto& adapter : adapters_) {
    if (adapter) {
      adapter->update(*data);
    }
  }

  output_queue_->push(data);
}

auto DataReceiver::getStats() const -> RateStats {
  std::vector<double> values;
  {  // critical section
    std::lock_guard<std::mutex> lock(mutex_);
    if (received_window_.size() <= 1) {
      return {};
    }

    values.resize(received_window_.size() - 1);
    for (size_t i = 1; i < received_window_.size(); ++i) {
      const auto diff_ns = std::abs(received_window_[i] - received_window_[i - 1]);
      const double rate_hz = 1.0 / (1.0e-9 * diff_ns);
      values[i - 1] = rate_hz;
    }
  }  // end critical section

  std::sort(values.begin(), values.end());

  RateStats stats;
  stats.num_measurements = received_window_.size() - 1;
  stats.min = std::numeric_limits<double>::max();
  for (const auto& value : values) {
    stats.min = std::min(stats.min, value);
    stats.max = std::max(stats.max, value);
    stats.mean += value;
  }

  const auto mid = values.size() / 2;
  if (values.size() % 2 == 0) {
    // this is safe because values.size() >= 1 and 2 is the first value
    // where this will trigger
    stats.median = (values[mid] + values[mid + 1]) / 2.0;
  } else {
    stats.median = values[mid];
  }

  stats.mean /= stats.num_measurements;
  for (const auto& value : values) {
    const auto diff = value - stats.mean;
    stats.variance += diff * diff;
  }

  stats.variance /= stats.num_measurements;
  return stats;
}

std::string DataReceiver::RateStats::str() const {
  if (!num_measurements) {
    return "n/a";
  }

  std::stringstream ss;
  ss << std::setprecision(3) << "mean: " << mean;
  if (num_measurements > 1) {
    ss << std::setprecision(3) << " ± " << variance;
  }

  ss << " (min: " << min << ", max: " << max << ", median: " << median << ") [hz] over "
     << num_measurements << " measurements";
  return ss.str();
}

void DataReceiver::stopImpl() {
  should_shutdown_ = true;
  if (thread_) {
    MLOG(1) << "stopping receiver '" << sensor_name << "'";
    thread_->join();
    thread_.reset();
    MLOG(1) << "stopped receiver '" << sensor_name << "'";
    MLOG(1) << "remaining in receiver '" << sensor_name << "' queue: " << queue_.size();
  }
}

}  // namespace hydra
