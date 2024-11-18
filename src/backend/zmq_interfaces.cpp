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
#include "hydra/backend/zmq_interfaces.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/zmq_interface.h>

namespace hydra {
namespace {

static const auto sink_registration =
    config::RegistrationWithConfig<BackendModule::Sink, ZmqSink, ZmqSink::Config>(
        "ZmqSink");

static const auto update_registration =
    config::RegistrationWithConfig<UpdateFunctor,
                                   ZmqRoomLabelUpdater,
                                   ZmqRoomLabelUpdater::Config>("ZmqRoomLabelUpdater");

}  // namespace

void declare_config(ZmqSink::Config& config) {
  using namespace config;
  name("ZmqSink::Config");
  field(config.url, "url");
  field(config.num_threads, "num_threads");
  field(config.send_mesh, "send_mesh");
}

ZmqSink::ZmqSink(const Config& config) : config(config::checkValid(config)) {
  sender_ = std::make_unique<spark_dsg::ZmqSender>(config.url, config.num_threads);
}

ZmqSink::~ZmqSink() = default;

void ZmqSink::call(uint64_t timestamp_ns,
                   const DynamicSceneGraph& graph,
                   const kimera_pgmo::DeformationGraph& /*dgraph*/) const {
  VLOG(5) << "Sending graph via zmq to '" << config.url << "' @ " << timestamp_ns
          << " [ns]";
  CHECK_NOTNULL(sender_)->send(graph, config.send_mesh);
}

std::string ZmqSink::printInfo() const { return config::toString(config); }

ZmqRoomLabelUpdater::ZmqRoomLabelUpdater(const Config& config)
    : config(config::checkValid(config)) {
  receiver_ = std::make_unique<spark_dsg::ZmqReceiver>(config.url, config.num_threads);
  thread_ = std::make_unique<std::thread>(&ZmqRoomLabelUpdater::checkForUpdates, this);
}

ZmqRoomLabelUpdater::~ZmqRoomLabelUpdater() {
  should_shutdown_ = true;
  if (thread_) {
    VLOG(2) << "[Hydra Backend] joining zmq thread and stopping";
    thread_->join();
    thread_.reset();
    VLOG(2) << "[Hydra Backend] stopped!";
  }
}

void ZmqRoomLabelUpdater::checkForUpdates() {
  while (!should_shutdown_) {
    if (!receiver_->recv(config.poll_time_ms)) {
      continue;
    }

    auto update_graph = receiver_->graph();
    if (!update_graph) {
      LOG(ERROR) << "zmq receiver graph is invalid";
      continue;
    }

    // start critical section for updating room label map
    std::lock_guard<std::mutex> lock(mutex_);
    const auto& rooms = update_graph->getLayer(DsgLayers::ROOMS);
    for (const auto& [node_id, node] : rooms.nodes()) {
      room_name_map_[node_id] = node->attributes<SemanticNodeAttributes>().name;
    }
  }
}

void ZmqRoomLabelUpdater::call(const DynamicSceneGraph&,
                               SharedDsgInfo& dsg,
                               const UpdateInfo::ConstPtr&) const {
  // start critical section for reading from room label map
  std::lock_guard<std::mutex> lock(mutex_);
  const auto& rooms = dsg.graph->getLayer(DsgLayers::ROOMS);
  for (const auto& [node_id, node] : rooms.nodes()) {
    const auto iter = room_name_map_.find(node_id);
    if (iter == room_name_map_.end()) {
      continue;
    }

    node->attributes<SemanticNodeAttributes>().name = iter->second;
  }

  return;
}

void declare_config(ZmqRoomLabelUpdater::Config& config) {
  using namespace config;
  name("ZmqRoomLabelUpdater::Config");
  field(config.url, "url");
  field(config.num_threads, "num_threads");
  field(config.poll_time_ms, "poll_time_ms");
}

}  // namespace hydra
