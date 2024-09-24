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
#include <atomic>
#include <memory>
#include <thread>

#include "hydra/active_window/active_window_output.h"
#include "hydra/active_window/volumetric_window.h"
#include "hydra/common/message_queue.h"
#include "hydra/common/module.h"
#include "hydra/common/output_sink.h"
#include "hydra/input/input_packet.h"
#include "hydra/reconstruction/volumetric_map.h"

namespace hydra {

class ActiveWindowModule : public Module {
 public:
  using Ptr = std::shared_ptr<ActiveWindowModule>;
  using InputQueue = MessageQueue<InputPacket::Ptr>;
  using OutputQueue = MessageQueue<ActiveWindowOutput::Ptr>;
  using Sink = OutputSink<uint64_t, const VolumetricMap&, const ActiveWindowOutput&>;

  struct Config {
    size_t max_input_queue_size;
    VolumetricMap::Config volumetric_map;
    config::VirtualConfig<VolumetricWindow> map_window;
    std::vector<Sink::Factory> sinks;

    // construct to allow downstream modules to set defaults
    Config(bool with_semantics = true, bool with_tracking = false);
  } const config;

  ActiveWindowModule(const Config& config, const OutputQueue::Ptr& output_queue);

  virtual ~ActiveWindowModule() = default;

  void start() override;

  void stop() override;

  void save(const LogSetup& log_setup) override;

  std::string printInfo() const override;

  void spin();

  bool step(const InputPacket::Ptr& input);

  void addSink(const Sink::Ptr& sink);

  InputQueue::Ptr queue() const { return input_queue_; }

  const VolumetricMap& map() const { return map_; }

 protected:
  virtual ActiveWindowOutput::Ptr spinOnce(const InputPacket& input) = 0;

  void stopImpl();

  InputQueue::Ptr input_queue_;
  OutputQueue::Ptr output_queue_;
  std::atomic<bool> should_shutdown_{false};
  std::unique_ptr<std::thread> spin_thread_;

  Sink::List sinks_;

  VolumetricMap map_;
  std::unique_ptr<VolumetricWindow> map_window_;
};

void declare_config(ActiveWindowModule::Config& config);

}  // namespace hydra
