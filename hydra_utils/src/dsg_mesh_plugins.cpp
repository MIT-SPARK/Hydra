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
#include "hydra_utils/dsg_mesh_plugins.h"

#include <kimera_pgmo/utils/CommonFunctions.h>
#include <tf2_eigen/tf2_eigen.h>

#include <mesh_msgs/TriangleMesh.h>
#include <mesh_msgs/TriangleMeshStamped.h>
#include <visualization_msgs/Marker.h>
#include <voxblox_msgs/Mesh.h>

namespace hydra {

using visualization_msgs::Marker;

struct BestIndex {
  bool valid;
  voxblox::BlockIndex index;
};

inline voxblox::Point getPoint(const pcl::PointXYZRGBA& point) {
  voxblox::Point vox_point;
  vox_point << point.x, point.y, point.z;
  return vox_point;
}

inline voxblox::BlockIndex getBlockIndex(const voxblox::Point& point) {
  voxblox::BlockIndex index;
  index << std::floor(point.x()), std::floor(point.y()), std::floor(point.z());
  return index;
}

inline bool vectorHasNegativeElement(const voxblox::Point& point) {
  return (point.array() < 0.0).any();
}

BestIndex getBestBlockIndex(const pcl::PointCloud<pcl::PointXYZRGBA>& cloud,
                            const std::vector<uint32_t>& indices) {
  const voxblox::Point point1 = getPoint(cloud.at(indices.at(0)));
  const voxblox::Point point2 = getPoint(cloud.at(indices.at(1)));
  const voxblox::Point point3 = getPoint(cloud.at(indices.at(2)));

  const voxblox::BlockIndex index1 = getBlockIndex(point1);
  const voxblox::BlockIndex index2 = getBlockIndex(point2);
  const voxblox::BlockIndex index3 = getBlockIndex(point3);

  // TODO(nathan) this is ugly, fix
  voxblox::BlockIndex best_index;
  best_index << std::min(index1.x(), std::min(index2.x(), index3.x())),
      std::min(index1.y(), std::min(index2.y(), index3.y())),
      std::min(index1.z(), std::min(index2.z(), index3.z()));

  const voxblox::Point norm1 = (point1 - best_index.cast<float>()) / 2.0;
  if (norm1.squaredNorm() > 1.0 || (norm1.array() < 0).any()) {
    LOG(ERROR) << "best index: " << best_index.transpose() << " is invalid";
    return {false, voxblox::BlockIndex::Zero()};
  }

  const voxblox::Point norm2 = (point2 - best_index.cast<float>()) / 2.0;
  if (norm2.squaredNorm() > 1.0 || (norm2.array() < 0).any()) {
    LOG(ERROR) << "best index: " << best_index.transpose() << " is invalid";
    return {false, voxblox::BlockIndex::Zero()};
  }

  const voxblox::Point norm3 = (point3 - best_index.cast<float>()) / 2.0;
  if (norm3.squaredNorm() > 1.0 || (norm3.array() < 0).any()) {
    LOG(ERROR) << "best index: " << best_index.transpose() << " is invalid";
    return {false, voxblox::BlockIndex::Zero()};
  }

  return {true, best_index};
}

inline geometry_msgs::Point pointFromPcl(const pcl::PointXYZRGBA& point) {
  geometry_msgs::Point msg;
  msg.x = point.x;
  msg.y = point.y;
  msg.z = point.z;
  return msg;
}

inline std_msgs::ColorRGBA colorFromPcl(const pcl::PointXYZRGBA& point) {
  std_msgs::ColorRGBA msg;
  msg.r = static_cast<float>(point.r) / 255.0f;
  msg.g = static_cast<float>(point.g) / 255.0f;
  msg.b = static_cast<float>(point.b) / 255.0f;
  msg.a = 1.0;
  return msg;
}

RvizMeshPlugin::RvizMeshPlugin(const ros::NodeHandle& nh, const std::string& name)
    : DsgVisualizerPlugin(nh, name), name_(name), published_mesh_(false) {
  ros::NodeHandle temp(nh);
  mesh_pub_ = temp.advertise<Marker>("mesh", 1, true);
}

void RvizMeshPlugin::draw(const std_msgs::Header& header,
                          const DynamicSceneGraph& graph) {
  if (!graph.hasMesh()) {
    ROS_WARN("Attempting to visualize unitialized mesh");
    return;
  }

  Marker msg;
  msg.header = header;
  msg.ns = name_;
  msg.id = 0;
  msg.type = Marker::TRIANGLE_LIST;
  msg.action = Marker::ADD;

  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  tf2::convert(origin, msg.pose.position);
  tf2::convert(Eigen::Quaterniond::Identity(), msg.pose.orientation);

  msg.scale.x = 1.0;
  msg.scale.y = 1.0;
  msg.scale.z = 1.0;

  const auto& vertices = *graph.getMeshVertices();
  const auto& faces = *graph.getMeshFaces();

  for (const auto& face : faces) {
    for (const auto& vertex_idx : face.vertices) {
      const auto& point = vertices.at(vertex_idx);
      msg.points.push_back(pointFromPcl(point));
      msg.colors.push_back(colorFromPcl(point));
    }
  }

  mesh_pub_.publish(msg);
  published_mesh_ = true;
}

void RvizMeshPlugin::reset(const std_msgs::Header& header, const DynamicSceneGraph&) {
  Marker msg;
  msg.header = header;
  msg.ns = name_;
  msg.id = 0;
  msg.action = Marker::DELETE;

  mesh_pub_.publish(msg);
}

PgmoMeshPlugin::PgmoMeshPlugin(const ros::NodeHandle& nh, const std::string& name)
    : DsgVisualizerPlugin(nh, name) {
  // namespacing gives us a reasonable topic
  mesh_pub_ = nh_.advertise<mesh_msgs::TriangleMeshStamped>("", 1, true);
}

void PgmoMeshPlugin::draw(const std_msgs::Header& header,
                          const DynamicSceneGraph& graph) {
  if (!graph.hasMesh()) {
    return;
  }

  if (!graph.getMeshVertices()->size()) {
    return;
  }

  mesh_msgs::TriangleMeshStamped msg;
  msg.header = header;

  // vertices and meshes are guaranteed to not be null (from hasMesh)
  msg.mesh = kimera_pgmo::PolygonMeshToTriangleMeshMsg(*graph.getMeshVertices(),
                                                       *graph.getMeshFaces());
  mesh_pub_.publish(msg);
}

void PgmoMeshPlugin::reset(const std_msgs::Header& header, const DynamicSceneGraph&) {
  mesh_msgs::TriangleMeshStamped msg;
  msg.header = header;
  mesh_pub_.publish(msg);
}

VoxbloxMeshPlugin::VoxbloxMeshPlugin(const ros::NodeHandle& nh, const std::string& name)
    : DsgVisualizerPlugin(nh, name) {
  // namespacing gives us a reasonable topic
  mesh_pub_ = nh_.advertise<voxblox_msgs::Mesh>("", 1, true);
}

void VoxbloxMeshPlugin::draw(const std_msgs::Header& header,
                             const DynamicSceneGraph& graph) {
  if (!graph.hasMesh()) {
    return;
  }

  voxblox_msgs::Mesh msg;
  msg.header = header;
  msg.block_edge_length = 1.0;

  voxblox::AnyIndexHashMapType<size_t>::type block_to_index;

  const auto& vertices = *graph.getMeshVertices();
  const auto& faces = *graph.getMeshFaces();

  curr_blocks_.clear();

  for (const auto& face : faces) {
    const auto block_idx_result = getBestBlockIndex(vertices, face.vertices);
    if (!block_idx_result.valid) {
      continue;
    }

    const auto block_idx = block_idx_result.index;
    if (!block_to_index.count(block_idx)) {
      block_to_index[block_idx] = msg.mesh_blocks.size();
      voxblox_msgs::MeshBlock block_msg;
      block_msg.index[0] = block_idx.x();
      block_msg.index[1] = block_idx.y();
      block_msg.index[2] = block_idx.z();
      msg.mesh_blocks.push_back(block_msg);
      curr_blocks_.push_back(block_idx);
    }

    auto& block = msg.mesh_blocks.at(block_to_index.at(block_idx));
    const voxblox::Point block_pos = block_idx.cast<float>();

    for (const auto& vertex_idx : face.vertices) {
      const auto& point = vertices.at(vertex_idx);
      voxblox::Point vox_point;
      vox_point << point.x, point.y, point.z;
      const voxblox::Point normalized_point = (vox_point - block_pos) / 2.0f;

      const uint16_t max_value = std::numeric_limits<uint16_t>::max();
      block.x.push_back(max_value * normalized_point.x());
      block.y.push_back(max_value * normalized_point.y());
      block.z.push_back(max_value * normalized_point.z());
      block.r.push_back(point.r);
      block.g.push_back(point.g);
      block.b.push_back(point.b);
    }
  }

  mesh_pub_.publish(msg);
}

void VoxbloxMeshPlugin::reset(const std_msgs::Header& header,
                              const DynamicSceneGraph&) {
  if (curr_blocks_.empty()) {
    return;
  }

  voxblox_msgs::Mesh msg;
  msg.header = header;
  msg.block_edge_length = 1.0;

  for (const auto& block_idx : curr_blocks_) {
    voxblox_msgs::MeshBlock block_msg;
    block_msg.index[0] = block_idx.x();
    block_msg.index[1] = block_idx.y();
    block_msg.index[2] = block_idx.z();
    msg.mesh_blocks.push_back(block_msg);
  }

  curr_blocks_.clear();
  mesh_pub_.publish(msg);
}

}  // namespace hydra
