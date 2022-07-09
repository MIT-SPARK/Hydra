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
#include <hydra_utils/semantic_ros_publishers.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

using ObjectCloudPublishers =
    SemanticRosPublishers<uint8_t, MeshSegmenter::MeshVertexCloud>;

struct ROSFrontend {
  ROSFrontend();

  ~ROSFrontend() { segmented_mesh_vertices_pub_.reset(); }

  void publishActiveVertices(const MeshSegmenter::MeshVertexCloud& vertices,
                             const std::vector<size_t>& indices,
                             const MeshSegmenter::LabelIndices&) const;

  void publishObjectClouds(const MeshSegmenter::MeshVertexCloud& vertices,
                           const std::vector<size_t>&,
                           const MeshSegmenter::LabelIndices& label_indices) const;

  void handleActivePlaces(const PlacesLayerMsg::ConstPtr& msg);

  void handleLatestMesh(const hydra_msgs::ActiveMesh::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Subscriber mesh_sub_;
  std::unique_ptr<ros::CallbackQueue> mesh_frontend_ros_queue_;
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  ros::Subscriber active_places_sub_;
  ros::Publisher active_mesh_vertex_pub_;
  std::unique_ptr<ObjectCloudPublishers> segmented_mesh_vertices_pub_;
  ros::Subscriber pose_graph_sub_;
};
