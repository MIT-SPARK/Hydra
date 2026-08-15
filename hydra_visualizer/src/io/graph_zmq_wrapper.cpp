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
#include "hydra_visualizer/io/graph_zmq_wrapper.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

namespace hydra {
namespace {
static const auto registration =
    config::RegistrationWithConfig<GraphWrapper,
                                   GraphZmqWrapper,
                                   GraphZmqWrapper::Config,
                                   ianvs::NodeHandle>("GraphFromZmq");
}

using spark_dsg::DynamicSceneGraph;

void declare_config(GraphZmqWrapper::Config& config) {
  using namespace config;
  name("GraphZmqWrapper::Config");
  field(config.frame_id, "frame_id");
  field(config.url, "url");
  field(config.num_threads, "num_threads");
  field(config.poll_time_ms, "poll_time_ms");
  checkCondition(!config.frame_id.empty(), "'frame_id' must be non-empty!");
}

GraphZmqWrapper::GraphZmqWrapper(const Config& config, ianvs::NodeHandle)
    : config(config::checkValid(config)), has_change_(false), should_shutdown_(false) {
  receiver_.reset(new spark_dsg::ZmqReceiver(config.url, config.num_threads));
  recv_thread_.reset(new std::thread(&GraphZmqWrapper::spin, this));
}

GraphZmqWrapper::~GraphZmqWrapper() {
  should_shutdown_ = true;
  if (recv_thread_) {
    LOG(INFO) << "joining receive thread...";
    recv_thread_->join();
    LOG(INFO) << "joined receive thread";
    recv_thread_.reset();
  }
  receiver_.reset();
}

bool GraphZmqWrapper::hasChange() const {
  std::lock_guard<std::mutex> lock(graph_mutex_);
  return has_change_;
}

void GraphZmqWrapper::clearChangeFlag() {
  std::lock_guard<std::mutex> lock(graph_mutex_);
  has_change_ = false;
}

StampedGraph GraphZmqWrapper::get() const {
  std::unique_lock<std::mutex> lock(graph_mutex_);
  if (!graph_) {
    return {nullptr};
  }

  // pass the lock for the visualizer to use the graph
  return {graph_, config.frame_id, std::nullopt, std::move(lock)};
}

void GraphZmqWrapper::spin() {
  while (!should_shutdown_) {
    // we always receive all messages
    if (!receiver_->recv(config.poll_time_ms, true)) {
      continue;
    }

    std::lock_guard<std::mutex> lock(graph_mutex_);
    if (receiver_->graph()) {
      graph_ = receiver_->graph()->clone();
    }

    has_change_ = true;
    VLOG(1) << "Got graph!";
  }
}

}  // namespace hydra
