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
#include "hydra_ros/utils/dsg_streaming_interface.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/common/dsg_types.h>
#include <hydra/utils/display_utilities.h>
#include <hydra/utils/pgmo_mesh_traits.h>
#include <hydra/utils/timing_utilities.h>
#include <kimera_pgmo/utils/common_functions.h>
#include <kimera_pgmo_ros/conversion/mesh.h>
#include <spark_dsg/serialization/graph_binary_serialization.h>

namespace hydra {

using hydra_msgs::msg::DsgUpdate;
using MeshMsg = kimera_pgmo_msgs::msg::Mesh;

void declare_config(DsgSender::Config& config) {
  using namespace config;
  name("DsgSender::Config");
  field(config.frame_id, "frame_id");
  field(config.timer_name, "publish_dsg");
  field(config.serialize_dsg_mesh, "serialize_dsg_mesh");
  field(config.publish_mesh, "publish_mesh");
  field(config.min_mesh_separation_s, "min_mesh_separation_s");
  field(config.min_dsg_separation_s, "min_dsg_separation_s");
  checkCondition(!config.frame_id.empty(), "frame_id empty!");
}

DsgSender::Config DsgSender::Config::with_name(const std::string& name) const {
  auto ret = *this;
  ret.timer_name = name;
  return ret;
}

DsgSender::Config DsgSender::Config::with_frame(const std::string& frame) const {
  auto ret = *this;
  ret.frame_id = frame;
  return ret;
}

DsgSender::DsgSender(const Config& config, ianvs::NodeHandle nh)
    : config(config::checkValid(config)) {
  pub_ = nh.create_publisher<DsgUpdate>("dsg", 1);
  if (config.publish_mesh) {
    mesh_pub_ = nh.create_publisher<MeshMsg>("dsg_mesh", 1);
  }
}

DsgSender::DsgSender(ianvs::NodeHandle nh,
                     const std::string& frame_id,
                     const std::string& timer_name,
                     bool publish_mesh,
                     double min_mesh_separation_s,
                     bool serialize_dsg_mesh)
    : DsgSender(Config{frame_id,
                       timer_name,
                       serialize_dsg_mesh,
                       publish_mesh,
                       min_mesh_separation_s,
                       0.0},
                nh) {}

void DsgSender::sendGraph(const DynamicSceneGraph& graph,
                          const rclcpp::Time& stamp) const {
  const uint64_t timestamp_ns = stamp.nanoseconds();
  timing::ScopedTimer timer(config.timer_name, timestamp_ns);

  publishGraph(graph, timestamp_ns);
  publishMesh(graph, timestamp_ns);
}

void DsgSender::publishGraph(const DynamicSceneGraph& graph,
                             uint64_t timestamp_ns) const {
  if (!pub_->get_subscription_count()) {
    return;
  }

  if (last_dsg_time_ns_) {
    std::chrono::nanoseconds diff_ns(timestamp_ns - *last_dsg_time_ns_);
    std::chrono::duration<double> diff_s = diff_ns;
    if (diff_s.count() < config.min_dsg_separation_s) {
      return;
    }
  }

  last_dsg_time_ns_ = timestamp_ns;

  auto msg = std::make_unique<DsgUpdate>();
  msg->header.stamp = rclcpp::Time(timestamp_ns);
  msg->header.frame_id = config.frame_id;
  spark_dsg::io::binary::writeGraph(
      graph, msg->layer_contents, config.serialize_dsg_mesh);
  msg->full_update = true;
  pub_->publish(std::move(msg));
}

void DsgSender::publishMesh(const DynamicSceneGraph& graph,
                            uint64_t timestamp_ns) const {
  if (!config.publish_mesh || !mesh_pub_->get_subscription_count()) {
    return;
  }

  auto mesh = graph.mesh();
  if (!mesh || mesh->empty()) {
    return;
  }

  if (last_mesh_time_ns_) {
    std::chrono::nanoseconds diff_ns(timestamp_ns - *last_mesh_time_ns_);
    std::chrono::duration<double> diff_s = diff_ns;
    if (diff_s.count() < config.min_mesh_separation_s) {
      return;
    }
  }

  last_mesh_time_ns_ = timestamp_ns;

  // TODO(nathan) grab the right robot id
  auto msg = kimera_pgmo::conversions::toMsg(0, *mesh);
  msg->header.stamp = rclcpp::Time(timestamp_ns);
  msg->header.frame_id = config.frame_id;
  mesh_pub_->publish(std::move(msg));
}

DsgReceiver::DsgReceiver(ianvs::NodeHandle nh, bool subscribe_to_mesh)
    : has_update_(false), graph_(nullptr) {
  sub_ = nh.create_subscription<DsgUpdate>("dsg", 1, &DsgReceiver::handleUpdate, this);
  if (subscribe_to_mesh) {
    mesh_sub_ = nh.create_subscription<MeshMsg>(
        "dsg_mesh_updates", 1, &DsgReceiver::handleMesh, this);
  }
}

DsgReceiver::DsgReceiver(ianvs::NodeHandle nh, const LogCallback& log_cb)
    : DsgReceiver(nh) {
  log_callback_.reset(new LogCallback(log_cb));
}

void DsgReceiver::handleUpdate(const DsgUpdate::ConstSharedPtr& msg) {
  const auto timestamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  timing::ScopedTimer timer("receive_dsg", timestamp_ns);
  if (!msg->full_update) {
    throw std::runtime_error("not implemented");
  }

  if (log_callback_) {
    (*log_callback_)(msg->header.stamp, msg->layer_contents.size());
  }

  const auto size_bytes = getHumanReadableMemoryString(msg->layer_contents.size());
  VLOG(5) << "Received dsg update message of " << size_bytes;
  try {
    if (!graph_) {
      graph_ = spark_dsg::io::binary::readGraph(msg->layer_contents);
    } else {
      spark_dsg::io::binary::updateGraph(*graph_, msg->layer_contents);
    }
    has_update_ = true;
  } catch (const std::exception& e) {
    LOG(ERROR) << "Received invalid message: " << e.what();
    return;
  }

  if (mesh_) {
    graph_->setMesh(mesh_);
  }
}

void DsgReceiver::handleMesh(const MeshMsg::ConstSharedPtr& msg) {
  if (!msg) {
    return;
  }

  const auto timestamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  timing::ScopedTimer timer("receive_mesh", timestamp_ns);
  if (!mesh_) {
    mesh_ = std::make_shared<Mesh>();
  }

  kimera_pgmo::conversions::fillFromMsg(*msg, *mesh_);

  if (graph_) {
    graph_->setMesh(mesh_);
  }

  has_update_ = true;
}

}  // namespace hydra
