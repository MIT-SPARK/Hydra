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

#include "hydra/common/common.h"

namespace hydra {

DataReceiver::DataReceiver(const Config& config, size_t sensor_id)
    : config(config::checkValid(config)), sensor_id_(sensor_id) {}

bool DataReceiver::init() { return initImpl(); }

bool DataReceiver::checkInputTimestamp(uint64_t timestamp_ns) {
  if (last_time_received_) {
    std::chrono::nanoseconds curr_time_ns(timestamp_ns);
    std::chrono::nanoseconds last_time_ns(*last_time_received_);
    std::chrono::duration<double> separation_s = curr_time_ns - last_time_ns;
    if (separation_s.count() < config.input_separation_s) {
      VLOG(10) << "[Data Receiver] Dropping input @ " << timestamp_ns
               << " [ns] with separation of " << separation_s.count() << " [s]";
      return false;
    }
  }

  last_time_received_ = timestamp_ns;
  VLOG(5) << "[Data Receiver] Got input @ " << timestamp_ns << " [ns]";
  return true;
}

void declare_config(DataReceiver::Config& config) {
  using namespace config;
  name("DataReceiver::Config");
  field(config.sensor, "sensor");
  field(config.input_separation_s, "input_separation_s");
}

}  // namespace hydra
