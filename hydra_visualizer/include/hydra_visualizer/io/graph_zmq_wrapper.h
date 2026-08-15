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
#include <config_utilities/factory.h>
#include <ianvs/node_handle.h>
#include <spark_dsg/zmq_interface.h>

#include <atomic>
#include <mutex>
#include <thread>

#include "hydra_visualizer/io/graph_wrapper.h"

namespace hydra {

class GraphZmqWrapper : public GraphWrapper {
 public:
  struct Config {
    std::string frame_id;
    std::string url = "tcp://127.0.0.1:8001";
    size_t num_threads = 2;
    size_t poll_time_ms = 10;
  } const config;

  explicit GraphZmqWrapper(const Config& config, ianvs::NodeHandle nh);

  virtual ~GraphZmqWrapper();

  bool hasChange() const override;

  void clearChangeFlag() override;

  StampedGraph get() const override;

 private:
  void spin();

  bool has_change_;
  std::atomic<bool> should_shutdown_;
  mutable std::mutex graph_mutex_;
  std::unique_ptr<std::thread> recv_thread_;
  std::unique_ptr<spark_dsg::ZmqReceiver> receiver_;
  spark_dsg::DynamicSceneGraph::Ptr graph_;
};

void declare_config(GraphZmqWrapper::Config& config);

}  // namespace hydra
