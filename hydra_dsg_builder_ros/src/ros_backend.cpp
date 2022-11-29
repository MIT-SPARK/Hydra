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
#include "hydra_dsg_builder_ros/ros_backend.h"

#include <hydra_utils/ros_utilities.h>

namespace hydra {

using kimera_pgmo::DeformationGraph;
using mesh_msgs::TriangleMeshStamped;
using message_filters::Subscriber;
using visualization_msgs::Marker;

RosBackend::RosBackend(const ros::NodeHandle& nh,
                       const RobotPrefixConfig& prefix,
                       const SharedDsgInfo::Ptr& dsg,
                       const SharedDsgInfo::Ptr& backend_dsg,
                       const SharedModuleState::Ptr& state)
    : DsgBackend(prefix,
                 load_config<incremental::DsgBackendConfig>(nh),
                 load_config<kimera_pgmo::KimeraPgmoConfig>(nh, "pgmo"),
                 dsg,
                 backend_dsg,
                 state),
      nh_(nh) {
  pose_graph_sub_ = nh_.subscribe(
      "pose_graph_incremental", 10000, &RosBackend::poseGraphCallback, this);

  mesh_sub_.reset(new Subscriber<KimeraPgmoMesh>(nh_, "pgmo/ful_mesh", 1));
  deformation_graph_sub_.reset(
      new Subscriber<PoseGraph>(nh_, "pgmo/mesh_graph_incremental", 100));
  sync_.reset(new Sync(Policy(10), *mesh_sub_, *deformation_graph_sub_));
  sync_->registerCallback(boost::bind(&RosBackend::inputCallback, this, _1, _2));
}

RosBackend::~RosBackend() {}

void RosBackend::inputCallback(const KimeraPgmoMesh::ConstPtr& mesh,
                               const PoseGraph::ConstPtr& deformation_graph) {
  latest_mesh_msg_ = mesh;
  have_new_mesh_ = true;

  incremental::BackendInput::Ptr input(new incremental::BackendInput());
  input->deformation_graph = deformation_graph;
  input->timestamp_ns = mesh->header.stamp.toNSec();
  input->pose_graphs = pose_graph_queue_;
  pose_graph_queue_.clear();

  state_->backend_queue.push(input);
}

void RosBackend::poseGraphCallback(const PoseGraph::ConstPtr& msg) {
  pose_graph_queue_.push_back(msg);
}

const pcl::PolygonMesh* RosBackend::getLatestMesh() {
  if (!latest_mesh_msg_) {
    return nullptr;
  }

  latest_mesh_.reset(new pcl::PolygonMesh());
  mesh_vertex_stamps_.reset(new std::vector<ros::Time>());
  mesh_vertex_graph_inds_.reset(new std::vector<int>());
  *latest_mesh_ = kimera_pgmo::PgmoMeshMsgToPolygonMesh(
      *latest_mesh_msg_, mesh_vertex_stamps_.get(), mesh_vertex_graph_inds_.get());
  return latest_mesh_.get();
}

RosBackendVisualizer::RosBackendVisualizer(const ros::NodeHandle& nh,
                                           const DsgBackendConfig& config)
    : nh_(nh), config_(config) {
  mesh_mesh_edges_pub_ =
      nh_.advertise<Marker>("pgmo/deformation_graph_mesh_mesh", 10, false);
  pose_mesh_edges_pub_ =
      nh_.advertise<Marker>("pgmo/deformation_graph_pose_mesh", 10, false);
  mesh_pub_ = nh_.advertise<TriangleMeshStamped>("pgmo/optimized_mesh", 1, false);
  pose_graph_pub_ = nh_.advertise<PoseGraph>("pgmo/pose_graph", 10, false);

  dsg_sender_.reset(new hydra::DsgSender(nh_));
  if (config_.use_zmq_interface) {
    zmq_sender_.reset(
        new spark_dsg::ZmqSender(config_.zmq_send_url, config_.zmq_num_threads));
  }
}

void RosBackendVisualizer::publishOutputs(const DynamicSceneGraph& graph,
                                          const pcl::PolygonMesh& mesh,
                                          const DeformationGraph& dgraph,
                                          size_t timestamp_ns) const {
  ros::Time stamp;
  stamp.fromNSec(timestamp_ns);

  // TODO(nathan) consider serializing to bytes before sending
  dsg_sender_->sendGraph(graph, stamp);
  if (config_.use_zmq_interface) {
    zmq_sender_->send(graph);
  }

  if (mesh_pub_.getNumSubscribers() > 0) {
    publishMesh(mesh, timestamp_ns);
  }

  // if (pose_graph_pub_.getNumSubscribers() > 0) {
  // publishPoseGraph(dgraph);
  //}

  if (mesh_mesh_edges_pub_.getNumSubscribers() > 0 ||
      pose_mesh_edges_pub_.getNumSubscribers() > 0) {
    publishDeformationGraphViz(dgraph, timestamp_ns);
  }
}

void RosBackendVisualizer::publishMesh(const pcl::PolygonMesh& mesh,
                                       size_t timestamp_ns) const {
  if (mesh.cloud.height * mesh.cloud.width == 0) {
    return;
  }

  mesh_msgs::TriangleMeshStamped msg;
  msg.header.stamp.fromNSec(timestamp_ns);
  msg.mesh = kimera_pgmo::PolygonMeshToTriangleMeshMsg(mesh);
  mesh_pub_.publish(msg);
}

void RosBackendVisualizer::publishPoseGraph(const DeformationGraph& dgraph) const {
  // TODO(nathan) grab from somewhere else
  std::map<size_t, std::vector<ros::Time>> id_timestamps;
  // id_timestamps[prefix_.id] = timestamps_;
  const auto& pose_graph = dgraph.getPoseGraph(id_timestamps);
  pose_graph_pub_.publish(*pose_graph);
}

void RosBackendVisualizer::publishDeformationGraphViz(const DeformationGraph& dgraph,
                                                      size_t timestamp_ns) const {
  ros::Time stamp;
  stamp.fromNSec(timestamp_ns);

  Marker mm_edges_msg;
  Marker pm_edges_msg;
  kimera_pgmo::fillDeformationGraphMarkers(dgraph, stamp, mm_edges_msg, pm_edges_msg);

  if (!mm_edges_msg.points.empty()) {
    mesh_mesh_edges_pub_.publish(mm_edges_msg);
  }
  if (!pm_edges_msg.points.empty()) {
    pose_mesh_edges_pub_.publish(pm_edges_msg);
  }
}

}  // namespace hydra