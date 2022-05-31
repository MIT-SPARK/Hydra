#include "hydra_utils/dsg_streaming_interface.h"
#include "hydra_utils/display_utils.h"
#include "hydra_utils/timing_utilities.h"

#include <kimera_dsg/graph_binary_serialization.h>
#include <kimera_pgmo/utils/CommonFunctions.h>

namespace hydra {

DsgSender::DsgSender(const ros::NodeHandle& nh) : nh_(nh) {
  pub_ = nh_.advertise<hydra_msgs::DsgUpdate>("dsg", 1);
}

void DsgSender::sendGraph(kimera::DynamicSceneGraph& graph,
                          const ros::Time& stamp) const {
  timing::ScopedTimer timer("publish_dsg", stamp.toNSec());
  if (!pub_.getNumSubscribers()) {
    return;
  }

  hydra_msgs::DsgUpdate msg;
  msg.header.stamp = stamp;
  kimera::writeGraph(graph, msg.layer_contents);

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
      graph_ = kimera::readGraph(msg->layer_contents);
    } else {
      kimera::updateGraph(*graph_, msg->layer_contents);
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
