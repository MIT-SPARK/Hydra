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

#include <hydra/common/dsg_types.h>
#include <hydra/utils/display_utilities.h>
#include <hydra/utils/timing_utilities.h>
#include <kimera_pgmo/utils/CommonFunctions.h>
#include <spark_dsg/graph_binary_serialization.h>

namespace hydra {

DsgSender::DsgSender(const ros::NodeHandle& nh,
                     const std::string& timer_name,
                     bool publish_mesh,
                     double min_mesh_separation_s)
    : nh_(nh),
      timer_name_(timer_name),
      publish_mesh_(publish_mesh),
      min_mesh_separation_s_(min_mesh_separation_s) {
  pub_ = nh_.advertise<hydra_msgs::DsgUpdate>("dsg", 1);
  if (publish_mesh_) {
    mesh_pub_ = nh_.advertise<mesh_msgs::TriangleMeshStamped>("dsg_mesh", 1, false);
  }
}

void DsgSender::sendGraph(const DynamicSceneGraph& graph,
                          const ros::Time& stamp) const {
  const uint64_t timestamp_ns = stamp.toNSec();
  timing::ScopedTimer timer(timer_name_, timestamp_ns);

  if (pub_.getNumSubscribers()) {
    hydra_msgs::DsgUpdate msg;
    msg.header.stamp = stamp;
    spark_dsg::writeGraph(graph, msg.layer_contents);
    msg.full_update = true;
    pub_.publish(msg);
  }

  if (!publish_mesh_ || !mesh_pub_.getNumSubscribers()) {
    return;
  }

  if (graph.isMeshEmpty()) {
    return;
  }

  if (last_mesh_time_ns_) {
    std::chrono::nanoseconds diff_ns(timestamp_ns - *last_mesh_time_ns_);
    std::chrono::duration<double> diff_s = diff_ns;
    if (diff_s.count() < min_mesh_separation_s_) {
      return;
    }
  }

  last_mesh_time_ns_ = timestamp_ns;

  mesh_msgs::TriangleMeshStamped msg;
  msg.header.stamp.fromNSec(timestamp_ns);
  msg.header.frame_id = "world";
  msg.mesh = kimera_pgmo::PolygonMeshToTriangleMeshMsg(*graph.getMeshVertices(),
                                                       *graph.getMeshFaces());
  mesh_pub_.publish(msg);
}

DsgReceiver::DsgReceiver(const ros::NodeHandle& nh)
    : nh_(nh), has_update_(false), graph_(nullptr) {
  sub_ = nh_.subscribe("dsg", 1, &DsgReceiver::handleUpdate, this);
  mesh_sub_ = nh_.subscribe("dsg_mesh_updates", 1, &DsgReceiver::handleMesh, this);
}

DsgReceiver::DsgReceiver(const ros::NodeHandle& nh, const LogCallback& log_cb)
    : DsgReceiver(nh) {
  log_callback_.reset(new LogCallback(log_cb));
}

void DsgReceiver::handleUpdate(const hydra_msgs::DsgUpdate::ConstPtr& msg) {
  timing::ScopedTimer timer("receive_dsg", msg->header.stamp.toNSec());
  if (!msg->full_update) {
    throw std::runtime_error("not implemented");
  }

  if (log_callback_) {
    (*log_callback_)(msg->header.stamp, msg->layer_contents.size());
  }

  const auto size_bytes = getHumanReadableMemoryString(msg->layer_contents.size());
  ROS_INFO_STREAM("Received dsg update message of " << size_bytes);
  try {
    if (!graph_) {
      graph_ = spark_dsg::readGraph(msg->layer_contents);
    } else {
      spark_dsg::updateGraph(*graph_, msg->layer_contents, true);
    }
    has_update_ = true;
  } catch (const std::exception& e) {
    ROS_FATAL_STREAM("Received invalid message: " << e.what());
    ros::shutdown();
    return;
  }

  if (mesh_) {
    graph_->setMeshDirectly(*mesh_);
  }
}

void DsgReceiver::handleMesh(const mesh_msgs::TriangleMeshStamped::ConstPtr& msg) {
  timing::ScopedTimer timer("receive_mesh", msg->header.stamp.toNSec());
  if (!mesh_) {
    mesh_.reset(new pcl::PolygonMesh());
  }

  *mesh_ = kimera_pgmo::TriangleMeshMsgToPolygonMesh(msg->mesh);

  if (graph_) {
    graph_->setMeshDirectly(*mesh_);
  }

  has_update_ = true;
}

}  // namespace hydra
