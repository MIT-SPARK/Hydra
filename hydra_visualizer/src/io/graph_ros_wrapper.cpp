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
#include "hydra_visualizer/io/graph_ros_wrapper.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/serialization/graph_binary_serialization.h>

#include <rclcpp/create_subscription.hpp>

namespace hydra {
namespace {
static const auto registration =
    config::RegistrationWithConfig<GraphWrapper,
                                   GraphRosWrapper,
                                   GraphRosWrapper::Config,
                                   ianvs::NodeHandle>("GraphFromRos");
}

using hydra_msgs::msg::DsgUpdate;
using spark_dsg::DynamicSceneGraph;

GraphRosWrapper::GraphRosWrapper(const Config& config, ianvs::NodeHandle nh)
    : config(config::checkValid(config)),
      has_change_(false),
      nh_(nh / config.wrapper_ns),
      sub_(nh_.create_subscription<DsgUpdate>(
          "dsg", 1, &GraphRosWrapper::callback, this)) {}

bool GraphRosWrapper::hasChange() const { return has_change_; }

void GraphRosWrapper::clearChangeFlag() { has_change_ = false; }

StampedGraph GraphRosWrapper::get() const {
  // lock and pass information to the visualizer
  std::unique_lock<std::mutex> lock(graph_mutex_);
  return {graph_, last_frame_id_, last_time_, std::move(lock)};
}

void GraphRosWrapper::callback(const DsgUpdate::ConstSharedPtr& msg) {
  // technically we shouldn't need a mutex here because the wrapper subscriber should be
  // in the same callback group as the rest of the node, but easy enough to pass a lock
  // to the visualizer
  std::lock_guard<std::mutex> lock(graph_mutex_);

  last_time_ = msg->header.stamp;
  last_frame_id_ = msg->header.frame_id;
  if (last_frame_id_.empty()) {
    LOG(ERROR) << "Received scene graph with empty frame_id field!";
    return;
  }

  try {
    if (!graph_) {
      graph_ = spark_dsg::io::binary::readGraph(msg->layer_contents);
    } else {
      spark_dsg::io::binary::updateGraph(*graph_, msg->layer_contents);
    }

    has_change_ = true;
  } catch (const std::exception& e) {
    LOG(ERROR) << "Received invalid message: " << e.what();
    return;
  }
}

void declare_config(GraphRosWrapper::Config& config) {
  using namespace config;
  name("GraphRosWrapper::Config");
  field(config.wrapper_ns, "wrapper_ns");
}

}  // namespace hydra
