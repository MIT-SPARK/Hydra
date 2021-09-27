#include "kimera_dsg_builder/incremental_mesh_segmenter.h"

#include <kimera_semantics_ros/ros_params.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <glog/logging.h>

namespace kimera {
namespace incremental {

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

bool objectsMatch(const Cluster<pcl::PointXYZRGBA>& cluster,
                  const SceneGraphNode& node) {
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
MeshSegmenter::MeshSegmenter(const ros::NodeHandle& nh,
                             const MeshVertexCloud::Ptr& full_mesh_vertices)
    : nh_(nh),
      full_mesh_vertices_(full_mesh_vertices),
      next_node_id_('O', 0),
      active_object_horizon_s_(10.0),
      enable_active_mesh_pub_(false),
      enable_segmented_mesh_pub_(false) {
  nh_.getParam("active_object_horizon_s", active_object_horizon_s_);
  nh_.getParam("enable_active_mesh_pub", enable_active_mesh_pub_);
  nh_.getParam("enable_segmented_mesh_pub", enable_segmented_mesh_pub_);

  double object_detection_period_s = 0.5;
  nh_.getParam("object_detection_period_s", object_detection_period_s);

  object_labels_ = readSemanticLabels(nh_, "object_labels");
  for (const auto& label : object_labels_) {
    active_objects_[label] = std::set<NodeId>();
  }

  // TODO(nathan) look at CGAL for better oriented bounding volumnes (min volume
  // ellipse)
  bool use_oriented_bounding_boxes = false;
  nh_.getParam("use_oriented_bounding_boxes", use_oriented_bounding_boxes);
  bounding_box_type_ =
      use_oriented_bounding_boxes ? BoundingBox::Type::RAABB : BoundingBox::Type::AABB;

  semantic_config_ = getSemanticTsdfIntegratorConfigFromRosParam(nh_);
  CHECK(semantic_config_.semantic_label_to_color_);

  object_finder_.reset(new ObjectFinder(ObjectFinderType::kRegionGrowing));

  if (enable_active_mesh_pub_) {
    active_mesh_vertex_pub_ =
        nh_.advertise<MeshVertexCloud>("active_mesh_vertices", 1, true);
  }

  if (enable_segmented_mesh_pub_) {
    segmented_mesh_vertices_pub_.reset(
        new ObjectCloudPublishers("object_mesh_vertices", nh_));
  }

  rqt_callback_ = boost::bind(&MeshSegmenter::objectFinderConfigCb, this, _1, _2);
  rqt_server_.setCallback(rqt_callback_);
}

bool MeshSegmenter::detectObjects(const SharedDsgInfo::Ptr& dsg,
                                  const std::vector<size_t>& active_indices) {
  const double latest_timestamp = ros::Time::now().toSec();
  archiveOldObjects(*dsg->graph, latest_timestamp);

  publishActiveVertices(active_indices);

  if (active_indices.empty()) {
    LOG(INFO) << "[Object Detection] No active indices in mesh";
    return false;
  }

  LabelIndices label_indices = getLabelIndices(active_indices);
  if (label_indices.empty()) {
    VLOG(3) << "[Object Detection] No object vertices found";
    return false;
  }

  publishObjectClouds(label_indices);

  VLOG(3) << "[Object Detection] Detecting objects";
  for (const auto label : object_labels_) {
    if (!label_indices.count(label)) {
      continue;
    }

    ObjectClusters clusters =
        object_finder_->findObjects(full_mesh_vertices_, label_indices.at(label));

    VLOG(3) << "[Object Detection]  - Found " << clusters.size() << " objects of label "
            << static_cast<int>(label);

    {  // start graph update critical section
      std::unique_lock<std::mutex> lock(dsg->mutex);
      updateGraph(*dsg->graph, clusters, label, latest_timestamp);
    }  // end graph update critical section
  }

  return true;
}

void MeshSegmenter::pruneObjectsToCheckForPlaces(const DynamicSceneGraph& graph) {
  std::list<NodeId> to_remove;
  for (const auto& object_id : objects_to_check_for_places_) {
    if (!graph.hasNode(object_id)) {
      LOG(ERROR) << "Missing node " << NodeSymbol(object_id).getLabel();
      to_remove.push_back(object_id);
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
                                      double latest_timestamp) {
  std::list<NodeId> removed_nodes;
  for (const auto id_time_pair : active_object_timestamps_) {
    if (latest_timestamp - id_time_pair.second > active_object_horizon_s_) {
      const NodeId curr_id = id_time_pair.first;
      removed_nodes.push_back(curr_id);
      uint8_t label = graph.getNode(curr_id)
                          .value()
                          .get()
                          .attributes<SemanticNodeAttributes>()
                          .semantic_label;
      active_objects_[label].erase(curr_id);
    }
  }

  for (const auto node_id : removed_nodes) {
    active_object_timestamps_.erase(node_id);
  }
}

LabelIndices MeshSegmenter::getLabelIndices(const std::vector<size_t>& indices) const {
  LabelIndices label_indices;

  std::set<uint8_t> seen_labels;
  for (const auto idx : indices) {
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
                                const ObjectClusters& clusters,
                                uint8_t label,
                                double timestamp) {
  for (const auto& cluster : clusters) {
    bool matches_prev_object = false;
    for (const auto& prev_node_id : active_objects_.at(label)) {
      const SceneGraphNode& prev_node = graph.getNode(prev_node_id).value();
      if (objectsMatch(cluster, prev_node)) {
        updateObjectInGraph(graph, cluster, prev_node, timestamp);
        matches_prev_object = true;
        break;
      }
    }

    if (!matches_prev_object) {
      addObjectToGraph(graph, cluster, label, timestamp);
    }
  }
}

void MeshSegmenter::updateObjectInGraph(DynamicSceneGraph& graph,
                                        const ObjectCluster& cluster,
                                        const SceneGraphNode& node,
                                        double timestamp) {
  active_object_timestamps_.at(node.id) = timestamp;

  for (const auto& idx : cluster.indices.indices) {
    graph.insertMeshEdge(node.id, idx);
  }

  auto new_box = BoundingBox::extract(cluster.cloud, bounding_box_type_);
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
                                     const ObjectCluster& cluster,
                                     uint8_t label,
                                     double timestamp) {
  CHECK(!cluster.cloud->empty());

  ObjectNodeAttributes::Ptr attrs = std::make_unique<ObjectNodeAttributes>();
  attrs->semantic_label = label;
  attrs->name = NodeSymbol(next_node_id_).getLabel();
  attrs->bounding_box = BoundingBox::extract(cluster.cloud, bounding_box_type_);

  const pcl::PointXYZRGBA& point = cluster.cloud->at(0);
  attrs->color << point.r, point.g, point.b;

  pcl::PointXYZ centroid;
  cluster.centroid.get(centroid);
  attrs->position << centroid.x, centroid.y, centroid.z;

  graph.emplaceNode(KimeraDsgLayers::OBJECTS, next_node_id_, std::move(attrs));

  active_objects_.at(label).insert(next_node_id_);
  active_object_timestamps_[next_node_id_] = timestamp;
  objects_to_check_for_places_.insert(next_node_id_);

  for (const auto& idx : cluster.indices.indices) {
    graph.insertMeshEdge(next_node_id_, idx);
  }

  ++next_node_id_;
}

void MeshSegmenter::objectFinderConfigCb(DsgBuilderConfig& config, uint32_t) {
  VLOG(3) << "Updating Object Finder params.";

  object_finder_->updateClusterEstimator(
      static_cast<ObjectFinderType>(config.object_finder_type));

  EuclideanClusteringParams ec_params;
  ec_params.cluster_tolerance = config.cluster_tolerance;
  ec_params.max_cluster_size = config.ec_max_cluster_size;
  ec_params.min_cluster_size = config.ec_min_cluster_size;
  object_finder_->setEuclideanClusterParams(ec_params);

  RegionGrowingClusteringParams rg_params;
  rg_params.curvature_threshold = config.curvature_threshold;
  rg_params.max_cluster_size = config.rg_max_cluster_size;
  rg_params.min_cluster_size = config.rg_min_cluster_size;
  rg_params.normal_estimator_neighbour_size = config.normal_estimator_neighbour_size;
  rg_params.number_of_neighbours = config.number_of_neighbours;
  rg_params.smoothness_threshold = config.smoothness_threshold;
  object_finder_->setRegionGrowingParams(rg_params);

  VLOG(3) << "Object finder: " << *object_finder_;
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
}  // namespace kimera
