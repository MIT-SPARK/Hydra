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
#include "hydra/input/input_module.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

namespace hydra {

void declare_config(InputModule::Config::InputPair& config) {
  using namespace config;
  name("InputModule::Config::InputPair");
  field(config.sensor, "sensor");
  field(config.receiver, "receiver");
}

void declare_config(InputModule::Config& config) {
  using namespace config;
  name("InputModule::Config");
  base<VerbosityConfig>(config);
  field(config.inputs, "inputs");
  checkCondition(!config.inputs.empty(), "At least one input must be specified");
}

InputModule::Config::Config()
    : VerbosityConfig(VerbosityConfig::default_verbosity("input")) {}

InputModule::InputModule(const Config& config, const DataQueue::Ptr& output_queue)
    : config(config::checkValid(config)),
      input_queue_(new DataQueue()),
      output_queue_(output_queue) {
  for (const auto& [name, pair] : config.inputs) {
    Sensor::ConstPtr sensor = pair.sensor.create(name);
    if (!sensor) {
      throw std::runtime_error("Could not create valid sensor for '" + name + "'");
    }

    receivers_.emplace_back(pair.receiver.create(sensor, input_queue_));
  }
}

InputModule::~InputModule() { stopImpl(); }

void InputModule::start() {
  for (auto& receiver : receivers_) {
    receiver->start();
  }

  data_thread_.reset(new std::thread(&InputModule::dataSpin, this));
  MLOG(0) << "started!";
}

void InputModule::stop() { stopImpl(); }

void InputModule::stopImpl() {
  should_shutdown_ = true;

  for (auto& receiver : receivers_) {
    receiver->stop();
  }

  if (data_thread_) {
    MLOG(1) << "stopping input thread";
    data_thread_->join();
    data_thread_.reset();
    MLOG(1) << "stopped input thread";
  }
}

std::string InputModule::printInfo() const { return config::toString(config); }

void InputModule::dataSpin() {
  while (!should_shutdown_) {
    auto has_data = input_queue_->poll();
    if (!has_data) {
      continue;
    }

    const auto data = input_queue_->pop();
    if (!data) {
      continue;
    }

    const auto curr_time = data->timestamp_ns;
    MLOG(3) << "popped input @ " << curr_time << " [ns]";

    const auto odom_T_body = getBodyPose(*data);
    if (!odom_T_body) {
      LOG(WARNING) << "[input] dropping input @ " << curr_time
                   << " [ns] due to missing pose";
      continue;
    }

    data->world_T_body = Eigen::Translation<double, 3>(odom_T_body.target_p_source) *
                         odom_T_body.target_R_source;

    MLOG(3) << "output queue state: size=" << output_queue_->size()
            << " (max=" << output_queue_->max_size << ") @ " << curr_time << " [ns]";

    output_queue_->push(data);
  }
}

}  // namespace hydra
