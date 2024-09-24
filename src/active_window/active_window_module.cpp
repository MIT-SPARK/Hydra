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
#include "hydra/active_window/active_window_module.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>

#include "hydra/common/global_info.h"

namespace hydra {

void declare_config(ActiveWindowModule::Config& config) {
  using namespace config;
  name("ActiveWindowModule::Config");
  field(config.max_input_queue_size, "max_input_queue_size");
  field(config.volumetric_map, "volumetric_map");
  config.map_window.setOptional();
  field(config.map_window, "map_window");
  field(config.sinks, "sinks");
}

ActiveWindowModule::Config::Config(bool with_semantics, bool with_tracking)
    : max_input_queue_size(0),
      volumetric_map({0.1, 16, 0.3, with_semantics, with_tracking}) {}

ActiveWindowModule::ActiveWindowModule(const Config& config,
                                       const OutputQueue::Ptr& queue)
    : config(config::checkValid(config)),
      input_queue_(new InputQueue(config.max_input_queue_size)),
      output_queue_(queue),
      sinks_(Sink::instantiate(config.sinks)),
      map_(config.volumetric_map),
      map_window_(config.map_window ? config.map_window.create()
                                    : GlobalInfo::instance().createVolumetricWindow()) {
}

void ActiveWindowModule::start() {
  spin_thread_.reset(new std::thread(&ActiveWindowModule::spin, this));
  LOG(INFO) << "[Active Window] started!";
}

void ActiveWindowModule::stop() { stopImpl(); }

void ActiveWindowModule::stopImpl() {
  should_shutdown_ = true;

  if (spin_thread_) {
    VLOG(2) << "[Active Window] stopping!";
    spin_thread_->join();
    spin_thread_.reset();
    VLOG(2) << "[Active Window] stopped!";
  }

  VLOG(2) << "[Active Window] input queue: " << input_queue_->size();
  if (output_queue_) {
    VLOG(2) << "[Active Window] output queue: " << output_queue_->size();
  } else {
    VLOG(2) << "[Active Window] output queue: n/a";
  }
}

void ActiveWindowModule::save(const LogSetup&) {}

std::string ActiveWindowModule::printInfo() const {
  return config::toString(config) + "\n" + Sink::printSinks(sinks_);
}

void ActiveWindowModule::spin() {
  bool should_shutdown = false;
  while (!should_shutdown) {
    bool has_data = input_queue_->poll();
    if (hydra::GlobalInfo::instance().force_shutdown() || !has_data) {
      should_shutdown = should_shutdown_;
    }

    if (!has_data) {
      continue;
    }

    const auto msg = input_queue_->pop();
    auto output = spinOnce(*msg);
    if (!output) {
      continue;
    }

    Sink::callAll(sinks_, msg->timestamp_ns, map_, *output);
    if (output_queue_) {
      output_queue_->push(output);
    }
  }
}

bool ActiveWindowModule::step(const InputPacket::Ptr& msg) {
  if (!msg) {
    return false;
  }

  auto output = spinOnce(*msg);
  if (!output) {
    return false;
  }

  Sink::callAll(sinks_, msg->timestamp_ns, map_, *output);
  if (output_queue_) {
    output_queue_->push(output);
  }

  return true;
}

void ActiveWindowModule::addSink(const Sink::Ptr& sink) {
  if (sink) {
    sinks_.push_back(sink);
  }
}

}  // namespace hydra
