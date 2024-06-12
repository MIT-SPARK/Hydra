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

#include "hydra/common/common.h"
#include "hydra/common/global_info.h"

namespace hydra {

void declare_config(InputModule::Config& config) {
  using namespace config;
  name("InputModule::Config");
  field(config.receivers, "receivers");
  checkCondition(!config.receivers.empty(), "At least one receiver must be specified");
}

InputModule::InputModule(const Config& config, const OutputQueue::Ptr& queue)
    : config(config::checkValid(config)), queue_(queue) {
  // Setup the receivers and instatiate their sensors globally.
  std::vector<config::VirtualConfig<Sensor>> sensor_configs;
  for (size_t i = 0; i < config.receivers.size(); ++i) {
    receivers_.emplace_back(config.receivers[i].create(i));
    sensor_configs.push_back(receivers_.back()->config.sensor);
  }

  GlobalInfo::instance().setSensors(sensor_configs);
}

InputModule::~InputModule() { stopImpl(); }

void InputModule::start() {
  for (auto& receiver : receivers_) {
    receiver->init();
  }
  data_thread_.reset(new std::thread(&InputModule::dataSpin, this));
  LOG(INFO) << "[Hydra Input] started!";
}

void InputModule::stop() { stopImpl(); }

void InputModule::stopImpl() {
  should_shutdown_ = true;

  if (data_thread_) {
    VLOG(2) << "[Hydra Input] stopping input thread";
    data_thread_->join();
    data_thread_.reset();
    VLOG(2) << "[Hydra Input] stopped input thread";
  }
  for (size_t i = 0; i < receivers_.size(); ++i) {
    VLOG(2) << "[Hydra Input] remaining in data queue[" << i
            << "]: " << receivers_[i]->queue.size();
  }
}

void InputModule::save(const LogSetup&) {}

std::string InputModule::printInfo() const {
  std::stringstream ss;
  ss << config::toString(config);
  return ss.str();
}

void InputModule::dataSpin() {
  while (!should_shutdown_) {
    for (const auto& receiver : receivers_) {
      const bool has_data = receiver->queue.poll();
      if (!has_data) {
        continue;
      }

      const auto packet = receiver->queue.pop();
      const auto curr_time = packet->timestamp_ns;
      VLOG(2) << "[Hydra Input] popped input @ " << curr_time << " [ns]";

      const auto odom_T_body = getBodyPose(curr_time);
      if (!odom_T_body) {
        LOG(WARNING) << "[Hydra Input] dropping input @ " << curr_time
                     << " [ns] due to missing pose";
        continue;
      }

      InputPacket::Ptr input(new InputPacket());
      input->timestamp_ns = curr_time;
      input->sensor_input = packet;
      input->world_t_body = odom_T_body.target_p_source;
      input->world_R_body = odom_T_body.target_R_source;
      queue_->push(input);
    }
  }
}

}  // namespace hydra
