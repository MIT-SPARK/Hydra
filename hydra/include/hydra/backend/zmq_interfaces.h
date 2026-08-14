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
#include <map>
#include <memory>
#include <mutex>
#include <thread>

#include "hydra/backend/backend_module.h"

namespace spark_dsg {
class ZmqReceiver;
class ZmqSender;
}  // namespace spark_dsg

namespace hydra {

class ZmqSink : public BackendModule::Sink {
 public:
  struct Config {
    std::string url = "tcp://127.0.0.1:8001";
    bool send_mesh = true;
    size_t num_threads = 2;
  } const config;

  explicit ZmqSink(const Config& config);
  virtual ~ZmqSink();
  void call(uint64_t timestamp_ns,
            const DynamicSceneGraph& graph,
            const kimera_pgmo::DeformationGraph& dgraph) const override;
  std::string printInfo() const override;

 private:
  std::unique_ptr<spark_dsg::ZmqSender> sender_;
};

void declare_config(ZmqSink::Config& config);

class ZmqRoomLabelUpdater : public UpdateFunctor {
 public:
  struct Config {
    std::string url = "tcp://127.0.0.1:8002";
    size_t num_threads = 2;
    size_t poll_time_ms = 10;
  } const config;

  ZmqRoomLabelUpdater(const Config& config);
  virtual ~ZmqRoomLabelUpdater();
  void call(const DynamicSceneGraph&,
            SharedDsgInfo& graph,
            const UpdateInfo::ConstPtr&) const override;

 private:
  void checkForUpdates();

  mutable std::mutex mutex_;
  std::unique_ptr<std::thread> thread_;
  std::atomic<bool> should_shutdown_{false};
  std::unique_ptr<spark_dsg::ZmqReceiver> receiver_;
  std::map<NodeId, std::string> room_name_map_;
};

void declare_config(ZmqRoomLabelUpdater::Config& conf);

}  // namespace hydra
