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
#include "hydra_utils/dsg_streaming_interface.h"
#include "hydra_utils/display_utils.h"
#include "hydra_utils/dsg_types.h"
#include "hydra_utils/timing_utilities.h"

#include <kimera_pgmo/utils/CommonFunctions.h>
#include <spark_dsg/graph_binary_serialization.h>

namespace hydra {

DsgSender::DsgSender(const ros::NodeHandle& nh) : nh_(nh) {
  pub_ = nh_.advertise<hydra_msgs::DsgUpdate>("dsg", 1);
}

void DsgSender::sendGraph(DynamicSceneGraph& graph, const ros::Time& stamp) const {
  timing::ScopedTimer timer("publish_dsg", stamp.toNSec());
  if (!pub_.getNumSubscribers()) {
    return;
  }

  hydra_msgs::DsgUpdate msg;
  msg.header.stamp = stamp;
  spark_dsg::writeGraph(graph, msg.layer_contents);

  msg.deleted_nodes = graph.getRemovedNodes(true);

  const auto deleted_edges = graph.getRemovedEdges(true);
  msg.deleted_edges.reserve(2 * deleted_edges.size());
  for (const auto& e : deleted_edges) {
    msg.deleted_edges.push_back(e.k1);
    msg.deleted_edges.push_back(e.k2);
  }

  msg.full_update = true;
  pub_.publish(msg);
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

  const auto size_bytes =
      hydra_utils::getHumanReadableMemoryString(msg->layer_contents.size());
  ROS_INFO_STREAM("Received dsg update message of " << size_bytes);
  try {
    if (!graph_) {
      graph_ = spark_dsg::readGraph(msg->layer_contents);
    } else {
      spark_dsg::updateGraph(*graph_, msg->layer_contents);
      for (const auto& node : msg->deleted_nodes) {
        graph_->removeNode(node);
      }

      for (size_t i = 0; i < msg->deleted_edges.size(); i += 2) {
        graph_->removeEdge(msg->deleted_edges[i], msg->deleted_edges[i + 1]);
      }
    }
    has_update_ = true;
  } catch (const std::exception&) {
    ROS_FATAL_STREAM("Received invalid message!");
    ros::shutdown();
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
