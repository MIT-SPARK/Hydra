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
#include "hydra_dsg_builder/incremental_mesh_segmenter.h"

#include <spark_dsg/bounding_box_extraction.h>
#include <hydra_utils/timing_utilities.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <glog/logging.h>

namespace hydra {
namespace incremental {

using kimera::HashableColor;
using Clusters = MeshSegmenter::Clusters;
using LabelClusters = MeshSegmenter::LabelClusters;
using LabelIndices = MeshSegmenter::LabelIndices;

std::ostream& operator<<(std::ostream& out, const std::set<uint8_t>& labels) {
  out << "[";
  auto iter = labels.begin();
  while (iter != labels.end()) {
    out << static_cast<int>(*iter);
    ++iter;
    if (iter != labels.end()) {
      out << ", ";
    }
  }
  out << "]";
  return out;
}

std::ostream& operator<<(std::ostream& out, const HashableColor& color) {
  return out << "[" << static_cast<int>(color.r) << ", " << static_cast<int>(color.g)
             << ", " << static_cast<int>(color.b) << ", " << static_cast<int>(color.a)
             << "]";
}

bool objectsMatch(const Cluster& cluster, const SceneGraphNode& node) {
  pcl::PointXYZ centroid;
  cluster.centroid.get(centroid);

  Eigen::Vector3f point;
  point << centroid.x, centroid.y, centroid.z;

  return node.attributes<ObjectNodeAttributes>().bounding_box.isInside(point);
}

std::set<uint8_t> readSemanticLabels(const ros::NodeHandle& nh,
                                     const std::string& param_name) {
  std::vector<int> labels;
  nh.getParam(param_name, labels);

  std::set<uint8_t> actual_labels;
  for (const auto& label : labels) {
    if (label < 0 || label > 255) {
      ROS_WARN_STREAM("Encountered label " << label << " outside range [0, 255] for "
                                           << param_name
                                           << ". Excluding from detection");
      continue;
    }
    actual_labels.insert(static_cast<uint8_t>(label));
  }

  return actual_labels;
}

// TODO(nathan) allow for different node symbol prefix
MeshSegmenter::MeshSegmenter(
    const ros::NodeHandle& nh,
    const kimera::SemanticIntegratorBase::SemanticConfig& config,
    const MeshVertexCloud::Ptr& full_mesh_vertices)
    : nh_(nh),
      full_mesh_vertices_(full_mesh_vertices),
      next_node_id_('O', 0),
      active_object_horizon_s_(10.0),
      active_index_horizon_m_(5.0),
      cluster_tolerance_(0.25),
      enable_active_mesh_pub_(false),
      enable_segmented_mesh_pub_(false),
      semantic_config_(config) {
  // TODO(nathan) make a config
  nh_.getParam("active_object_horizon_s", active_object_horizon_s_);
  nh_.getParam("active_index_horizon_m", active_index_horizon_m_);
  nh_.getParam("cluster_tolerance", cluster_tolerance_);
  int min_cluster_size = 25;
  nh_.getParam("min_cluster_size", min_cluster_size);
  min_cluster_size_ = static_cast<size_t>(min_cluster_size);
  int max_cluster_size = 100000;
  nh_.getParam("max_cluster_size", max_cluster_size);
  max_cluster_size_ = static_cast<size_t>(max_cluster_size);
  nh_.getParam("enable_active_mesh_pub", enable_active_mesh_pub_);
  nh_.getParam("enable_segmented_mesh_pub", enable_segmented_mesh_pub_);

  double object_detection_period_s = 0.5;
  nh_.getParam("object_detection_period_s", object_detection_period_s);

  object_labels_ = readSemanticLabels(nh_, "object_labels");
  for (const auto& label : object_labels_) {
    active_objects_[label] = std::set<NodeId>();
  }

  bool use_oriented_bounding_boxes = false;
  nh_.getParam("use_oriented_bounding_boxes", use_oriented_bounding_boxes);
  bounding_box_type_ =
      use_oriented_bounding_boxes ? BoundingBox::Type::RAABB : BoundingBox::Type::AABB;

  CHECK(semantic_config_.semantic_label_to_color_);

  if (enable_active_mesh_pub_) {
    active_mesh_vertex_pub_ =
        nh_.advertise<MeshVertexCloud>("active_mesh_vertices", 1, true);
  }

  if (enable_segmented_mesh_pub_) {
    segmented_mesh_vertices_pub_.reset(
        new ObjectCloudPublishers("object_mesh_vertices", nh_));
  }
}

MeshSegmenter::~MeshSegmenter() {
  // handle this before node handle disappears
  segmented_mesh_vertices_pub_.reset();
}

using KdTreeT = pcl::search::KdTree<pcl::PointXYZRGBA>;

Clusters MeshSegmenter::findClusters(const MeshVertexCloud::Ptr& cloud,
                                     const std::vector<size_t>& indices) const {
  CHECK(cloud);

  // in general, this is unsafe, but PCL doesn't offer us any alternative
  pcl::IndicesPtr cloud_indices(new std::vector<int>(indices.begin(), indices.end()));

  KdTreeT::Ptr tree(new KdTreeT());
  tree->setInputCloud(cloud, cloud_indices);

  pcl::EuclideanClusterExtraction<Cluster::PointT> estimator;
  estimator.setClusterTolerance(cluster_tolerance_);
  estimator.setMinClusterSize(min_cluster_size_);
  estimator.setMaxClusterSize(max_cluster_size_);
  estimator.setSearchMethod(tree);
  estimator.setInputCloud(cloud);
  estimator.setIndices(cloud_indices);

  std::vector<pcl::PointIndices> cluster_indices;
  estimator.extract(cluster_indices);

  Clusters clusters;
  clusters.resize(cluster_indices.size());
  for (size_t k = 0; k < clusters.size(); ++k) {
    clusters.at(k).indices = cluster_indices.at(k);
    clusters.at(k).cloud.reset(new MeshVertexCloud());

    const auto& object_indices = cluster_indices.at(k).indices;
    clusters.at(k).cloud->resize(object_indices.size());

    for (size_t i = 0; i < object_indices.size(); ++i) {
      const auto& cp = cloud->at(object_indices.at(i));
      clusters.at(k).cloud->at(i) = cp;
      clusters.at(k).centroid.add(pcl::PointXYZ(cp.x, cp.y, cp.z));
    }
  }

  return clusters;
}

LabelClusters MeshSegmenter::findNewObjectClusters(
    const std::vector<size_t>& active_indices) const {
  LabelClusters object_clusters;

  if (active_indices.empty()) {
    VLOG(3) << "[Object Detection] No active indices in mesh";
    return object_clusters;
  }

  LabelIndices label_indices = getLabelIndices(active_indices);
  if (label_indices.empty()) {
    VLOG(3) << "[Object Detection] No object vertices found";
    return object_clusters;
  }
  publishObjectClouds(label_indices);

  VLOG(3) << "[Object Detection] Detecting objects";
  for (const auto label : object_labels_) {
    if (!label_indices.count(label)) {
      continue;
    }

    if (label_indices.at(label).size() < min_cluster_size_) {
      continue;
    }

    const auto clusters = findClusters(full_mesh_vertices_, label_indices.at(label));

    VLOG(3) << "[Object Detection]  - Found " << clusters.size() << " objects of label "
            << static_cast<int>(label);
    object_clusters.insert({label, clusters});
  }
  return object_clusters;
}

LabelClusters MeshSegmenter::detectObjects(const std::vector<size_t>& frontend_indices,
                                           const std::optional<Eigen::Vector3d>& pos) {
  std::vector<size_t> active_indices;
  if (!pos) {
    active_indices = frontend_indices;
  } else {
    active_indices.reserve(frontend_indices.size());
    const Eigen::Vector3d root_pos = *pos;
    for (const size_t idx : frontend_indices) {
      const auto& p = full_mesh_vertices_->at(idx);
      const Eigen::Vector3d vertex_pos(p.x, p.y, p.z);
      if ((vertex_pos - root_pos).norm() < active_index_horizon_m_) {
        active_indices.push_back(idx);
      }
    }
  }
  VLOG(1) << "active indices: " << frontend_indices.size()
          << " used: " << active_indices.size();

  publishActiveVertices(active_indices);

  return findNewObjectClusters(active_indices);
}

void MeshSegmenter::pruneObjectsToCheckForPlaces(const DynamicSceneGraph& graph) {
  std::list<NodeId> to_remove;
  for (const auto& object_id : objects_to_check_for_places_) {
    if (!graph.hasNode(object_id)) {
      LOG(ERROR) << "Missing node " << NodeSymbol(object_id).getLabel();
      to_remove.push_back(object_id);
      continue;
    }

    if (graph.getNode(object_id).value().get().hasParent()) {
      to_remove.push_back(object_id);
    }
  }

  for (const auto& node_id : to_remove) {
    objects_to_check_for_places_.erase(node_id);
  }
}

void MeshSegmenter::archiveOldObjects(const DynamicSceneGraph& graph,
                                      uint64_t latest_timestamp) {
  for (const auto& label : object_labels_) {
    std::list<NodeId> removed_nodes;
    for (const auto& object_node : active_objects_.at(label)) {
      if (!graph.hasNode(object_node)) {
        removed_nodes.push_back(object_node);
      }

      if (latest_timestamp - active_object_timestamps_.at(object_node) >
          static_cast<uint64_t>(active_object_horizon_s_ * 1e9)) {
        removed_nodes.push_back(object_node);
      }
    }

    for (const auto& node_id : removed_nodes) {
      active_objects_[label].erase(node_id);
      active_object_timestamps_.erase(node_id);
    }
  }
}

LabelIndices MeshSegmenter::getLabelIndices(const std::vector<size_t>& indices) const {
  LabelIndices label_indices;

  std::set<uint8_t> seen_labels;
  for (const auto idx : indices) {
    if (idx >= full_mesh_vertices_->size()) {
      LOG(ERROR) << "Invalid indice: " << idx << "(out of "
                 << full_mesh_vertices_->size() << ")";
      continue;
    }
    const pcl::PointXYZRGBA& point = full_mesh_vertices_->at(idx);
    const HashableColor color(point.r, point.g, point.b, 255);

    const uint8_t label =
        semantic_config_.semantic_label_to_color_->getSemanticLabelFromColor(color);
    seen_labels.insert(label);

    if (!object_labels_.count(label)) {
      continue;
    }

    if (!label_indices.count(label)) {
      label_indices[label] = std::vector<size_t>();
    }

    label_indices[label].push_back(idx);
  }

  VLOG(3) << "[Object Detection] Seen labels: " << seen_labels;

  return label_indices;
}

void MeshSegmenter::updateGraph(DynamicSceneGraph& graph,
                                const LabelClusters& clusters,
                                uint64_t timestamp) {
  archiveOldObjects(graph, timestamp);

  for (const auto& label_clusters : clusters) {
    for (const auto& cluster : label_clusters.second) {
      bool matches_prev_object = false;
      std::vector<NodeId> nodes_not_in_graph;
      for (const auto& prev_node_id : active_objects_.at(label_clusters.first)) {
        const SceneGraphNode& prev_node = graph.getNode(prev_node_id).value();
        if (objectsMatch(cluster, prev_node)) {
          updateObjectInGraph(graph, cluster, prev_node, timestamp);
          matches_prev_object = true;
          break;
        }
      }

      if (!matches_prev_object) {
        addObjectToGraph(graph, cluster, label_clusters.first, timestamp);
      }
    }

    auto to_check = active_objects_[label_clusters.first];
    for (const auto& node_id : to_check) {
      if (!graph.hasNode(node_id)) {
        continue;
      }

      const auto& node =
          graph.getNode(node_id).value().get().attributes<SemanticNodeAttributes>();

      for (const auto& other_id : to_check) {
        if (node_id == other_id) {
          continue;
        }

        if (!graph.hasNode(other_id)) {
          continue;
        }

        const auto& other =
            graph.getNode(other_id).value().get().attributes<SemanticNodeAttributes>();

        if (node.bounding_box.isInside(other.position) ||
            other.bounding_box.isInside(node.position)) {
          if (node.bounding_box.volume() >= other.bounding_box.volume()) {
            graph.removeNode(other_id);
            active_objects_[label_clusters.first].erase(other_id);
            active_object_timestamps_.erase(other_id);
            objects_to_check_for_places_.erase(other_id);
          } else {
            graph.removeNode(node_id);
            active_objects_[label_clusters.first].erase(node_id);
            active_object_timestamps_.erase(node_id);
            objects_to_check_for_places_.erase(node_id);
          }
        }
      }
    }
  }
}

void MeshSegmenter::updateObjectInGraph(DynamicSceneGraph& graph,
                                        const Cluster& cluster,
                                        const SceneGraphNode& node,
                                        uint64_t timestamp) {
  active_object_timestamps_.at(node.id) = timestamp;

  for (const auto& idx : cluster.indices.indices) {
    graph.insertMeshEdge(node.id, idx, true);
  }

  auto new_box = bounding_box::extract(cluster.cloud, bounding_box_type_);
  ObjectNodeAttributes& attrs = node.attributes<ObjectNodeAttributes>();
  if (attrs.bounding_box.volume() >= new_box.volume()) {
    return;  // prefer the largest detection
  }

  objects_to_check_for_places_.insert(node.id);

  // if we have a more complete detection, update centroid and box
  pcl::PointXYZ centroid;
  cluster.centroid.get(centroid);
  attrs.position << centroid.x, centroid.y, centroid.z;
  attrs.bounding_box = new_box;
}

void MeshSegmenter::addObjectToGraph(DynamicSceneGraph& graph,
                                     const Cluster& cluster,
                                     uint8_t label,
                                     uint64_t timestamp) {
  CHECK(!cluster.cloud->empty());

  ObjectNodeAttributes::Ptr attrs = std::make_unique<ObjectNodeAttributes>();
  attrs->semantic_label = label;
  attrs->name = NodeSymbol(next_node_id_).getLabel();
  attrs->bounding_box = bounding_box::extract(cluster.cloud, bounding_box_type_);

  const pcl::PointXYZRGBA& point = cluster.cloud->at(0);
  attrs->color << point.r, point.g, point.b;

  pcl::PointXYZ centroid;
  cluster.centroid.get(centroid);
  attrs->position << centroid.x, centroid.y, centroid.z;

  graph.emplaceNode(DsgLayers::OBJECTS, next_node_id_, std::move(attrs));

  active_objects_.at(label).insert(next_node_id_);
  active_object_timestamps_[next_node_id_] = timestamp;
  objects_to_check_for_places_.insert(next_node_id_);

  for (const auto& idx : cluster.indices.indices) {
    graph.insertMeshEdge(next_node_id_, idx, true);
  }

  ++next_node_id_;
}

void MeshSegmenter::publishActiveVertices(const std::vector<size_t>& indices) const {
  if (!enable_active_mesh_pub_) {
    return;
  }

  MeshVertexCloud::Ptr active_cloud(new MeshVertexCloud());
  active_cloud->reserve(indices.size());
  for (const auto idx : indices) {
    active_cloud->push_back(full_mesh_vertices_->at(idx));
  }

  active_cloud->header.frame_id = "world";
  pcl_conversions::toPCL(ros::Time::now(), active_cloud->header.stamp);
  active_mesh_vertex_pub_.publish(active_cloud);
}

void MeshSegmenter::publishObjectClouds(const LabelIndices& label_indices) const {
  if (!enable_segmented_mesh_pub_) {
    return;
  }

  for (const auto& label_index_pair : label_indices) {
    MeshVertexCloud label_cloud;
    label_cloud.reserve(label_index_pair.second.size());
    for (const auto idx : label_index_pair.second) {
      label_cloud.push_back(full_mesh_vertices_->at(idx));
    }

    label_cloud.header.frame_id = "world";
    pcl_conversions::toPCL(ros::Time::now(), label_cloud.header.stamp);
    segmented_mesh_vertices_pub_->publish(label_index_pair.first, label_cloud);
  }
}

}  // namespace incremental
}  // namespace hydra
