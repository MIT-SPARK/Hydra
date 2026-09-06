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

#include <deque>

#include "hydra/common/message_queue.h"
#include "hydra/input/input_adapter.h"
#include "hydra/input/input_filter.h"
#include "hydra/input/sensor.h"
#include "hydra/input/sensor_input_packet.h"
#include "hydra/utils/logging.h"

namespace hydra {

class DataReceiver {
 public:
  using DataQueue = MessageQueue<SensorInputPacket::Ptr>;
  using OutputQueue = MessageQueue<InputData::Ptr>;

  struct Config : VerbosityConfig {
    Config();

    //! Maximum queue size (0 means unlimited)
    size_t max_packets = 0;
    //! Enforced time separation between packets
    double input_separation_s = 0.0;
    //! Filters to discard invalid inputs
    std::vector<config::VirtualConfig<InputFilter, true>> filters;
    //! Adapters to pre-process input packets
    std::vector<config::VirtualConfig<InputAdapter, true>> adapters;
    //! Number of timestamps to keep for monitoring rate
    size_t received_window_size = 21;
  } const config;

  DataReceiver(const Config& config,
               const Sensor::ConstPtr& sensor,
               const OutputQueue::Ptr& output);
  virtual ~DataReceiver();

  bool start();

  void stop();

  void clear();

  struct RateStats {
    size_t num_measurements = 0;
    double mean = 0.0;
    double min = 0.0;
    double max = 0.0;
    double median = 0.0;
    double variance = 0.0;

    std::string str() const;
  };
  RateStats getStats() const;

  const Sensor::ConstPtr sensor;
  const std::string sensor_name;

 protected:
  void spin();

  void pushPacket(SensorInputPacket::Ptr packet);

  void recordTimestamp(uint64_t timestamp);

  virtual bool initImpl() = 0;

  DataQueue queue_;
  OutputQueue::Ptr output_queue_;
  std::atomic<bool> should_shutdown_{false};
  std::unique_ptr<std::thread> thread_;

  SensorInputPacket::Ptr last_received_;
  std::vector<std::unique_ptr<InputFilter>> filters_;
  std::vector<std::unique_ptr<InputAdapter>> adapters_;

  mutable std::mutex mutex_;
  std::deque<int64_t> received_window_;
};

void declare_config(DataReceiver::Config& config);

}  // namespace hydra
