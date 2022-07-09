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
#include "hydra_dsg_builder/ros_frontend.h"

namespace hydra {

ROSFrontend() {
  ros::NodeHandle mesh_nh(nh_, config_.mesh_ns);

  config_ = load_config<DsgFrontendConfig>(nh);

  pose_graph_sub_ = nh_.subscribe(
      "pose_graph_incremental", 100, &DsgFrontend::handleLatestPoseGraph, this);

  active_places_sub_ =
      nh_.subscribe("active_places", 5, &DsgFrontend::handleActivePlaces, this);

  tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));

  mesh_frontend_ros_queue_.reset(new ros::CallbackQueue());

  mesh_sub_ = nh_.subscribe("voxblox_mesh", 5, &DsgFrontend::handleLatestMesh, this);

  if (config_.enable_active_mesh_pub) {
    active_mesh_vertex_pub_ =
        nh_.advertise<MeshVertexCloud>("active_mesh_vertices", 1, true);
  }

  if (config_.enable_segmented_mesh_pub) {
    segmented_mesh_vertices_pub_.reset(
        new ObjectCloudPublishers("object_mesh_vertices", nh_));
  }
}

ROSFrontend::~ROSFrontend() { segmented_mesh_vertices_pub_.reset(); }

void ROSFrontend::handleActivePlaces(const PlacesLayerMsg::ConstPtr& msg) {
  std::unique_lock<std::mutex> queue_lock(places_queue_mutex_);
  places_queue_.push(msg);
}

void ROSFrontend::handleLatestMesh(const hydra_msgs::ActiveMesh::ConstPtr& msg) {
  {  // start mesh frontend critical section
    std::unique_lock<std::mutex> mesh_lock(mesh_frontend_mutex_);
    if (mesh_queue_.size() < config_.mesh_queue_size) {
      mesh_queue_.push(msg);
      return;
    }
  }  // end mesh frontend critical section

  ROS_WARN_STREAM("[DSG Frontend] Dropping mesh update @ "
                  << msg->header.stamp.toSec() << " [s] (" << msg->header.stamp.toNSec()
                  << " [ns])");
}

void ROSFrontend::publishActiveVertices(const MeshVertexCloud& vertices,
                                        const std::vector<size_t>& indices,
                                        const LabelIndices&) const {
  MeshVertexCloud::Ptr active_cloud(new MeshVertexCloud());
  active_cloud->reserve(indices.size());
  for (const auto idx : indices) {
    active_cloud->push_back(vertices.at(idx));
  }

  active_cloud->header.frame_id = "world";
  pcl_conversions::toPCL(ros::Time::now(), active_cloud->header.stamp);
  active_mesh_vertex_pub_.publish(active_cloud);
}

void ROSFrontend::publishObjectClouds(const MeshVertexCloud& vertices,
                                      const std::vector<size_t>&,
                                      const LabelIndices& label_indices) const {
  for (const auto& label_index_pair : label_indices) {
    MeshVertexCloud label_cloud;
    label_cloud.reserve(label_index_pair.second.size());
    for (const auto idx : label_index_pair.second) {
      label_cloud.push_back(vertices.at(idx));
    }

    label_cloud.header.frame_id = "world";
    pcl_conversions::toPCL(ros::Time::now(), label_cloud.header.stamp);
    segmented_mesh_vertices_pub_->publish(label_index_pair.first, label_cloud);
  }
}

std::optional<Eigen::Vector3d> DsgFrontend::getLatestPose() {
  if (!config_.prune_mesh_indices) {
    return std::nullopt;
  }

  geometry_msgs::TransformStamped msg;
  try {
    msg = tf_buffer_.lookupTransform("world", config_.sensor_frame, ros::Time(0));
  } catch (tf2::TransformException& ex) {
    LOG_FIRST_N(WARNING, 3) << "failed to look up transform to " << config_.sensor_frame
                            << " @ " << last_mesh_timestamp_ << ": " << ex.what()
                            << " Not filtering indices.";
    return std::nullopt;
  }

  return Eigen::Vector3d(msg.transform.translation.x,
                         msg.transform.translation.y,
                         msg.transform.translation.z);
}

}  // namespace hydra
