#pragma once
#include "hydra_dsg_builder/incremental_types.h"

#include <hydra_utils/semantic_ros_publishers.h>
#include <kimera_dsg/dynamic_scene_graph.h>
#include <kimera_dsg/node_symbol.h>
#include <kimera_semantics/semantic_integrator_base.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include <memory>
#include <mutex>

namespace hydra {
namespace incremental {

struct Cluster {
  using PointT = pcl::PointXYZRGBA;
  using CloudT = pcl::PointCloud<PointT>;
  using CentroidT = pcl::CentroidPoint<pcl::PointXYZ>;
  CentroidT centroid;
  CloudT::Ptr cloud;
  pcl::PointIndices indices;
};

class MeshSegmenter {
 public:
  using LabelIndices = std::map<uint8_t, std::vector<size_t>>;
  using MeshVertexCloud = Cluster::CloudT;
  using ObjectCloudPublishers = SemanticRosPublishers<uint8_t, MeshVertexCloud>;
  using KdTreeT = pcl::search::KdTree<Cluster::PointT>;
  using Clusters = std::vector<Cluster>;
  using LabelClusters = std::map<uint8_t, Clusters>;

  explicit MeshSegmenter(const ros::NodeHandle& nh,
                         const MeshVertexCloud::Ptr& active_vertices);

  virtual ~MeshSegmenter();

  LabelClusters detectObjects(const std::vector<size_t>& active_indices,
                              const std::optional<Eigen::Vector3d>& pos);

  inline std::unordered_set<NodeId> getObjectsToCheckForPlaces() const {
    return objects_to_check_for_places_;
  }

  void pruneObjectsToCheckForPlaces(const DynamicSceneGraph& graph);

  void updateGraph(DynamicSceneGraph& graph,
                   const LabelClusters& clusters,
                   uint64_t timestamp);

 private:
  LabelClusters findNewObjectClusters(const std::vector<size_t>& active_indices) const;

  Clusters findClusters(const MeshVertexCloud::Ptr& cloud,
                        const std::vector<size_t>& indices) const;

  void archiveOldObjects(const DynamicSceneGraph& graph, uint64_t latest_timestamp);

  LabelIndices getLabelIndices(const std::vector<size_t>& indices) const;

  void addObjectToGraph(DynamicSceneGraph& graph,
                        const Cluster& cluster,
                        uint8_t label,
                        uint64_t timestamp);

  void updateObjectInGraph(DynamicSceneGraph& graph,
                           const Cluster& cluster,
                           const SceneGraphNode& node,
                           uint64_t timestamp);

  void publishActiveVertices(const std::vector<size_t>& indices) const;

  void publishObjectClouds(const LabelIndices& label_indices) const;

 private:
  ros::NodeHandle nh_;

  MeshVertexCloud::Ptr full_mesh_vertices_;

  NodeSymbol next_node_id_;
  double active_object_horizon_s_;
  double active_index_horizon_m_;
  double cluster_tolerance_;  // maxium radius
  size_t min_cluster_size_;
  size_t max_cluster_size_;
  std::map<uint8_t, std::set<NodeId>> active_objects_;
  std::map<NodeId, uint64_t> active_object_timestamps_;
  std::unordered_set<NodeId> objects_to_check_for_places_;

  std::set<uint8_t> object_labels_;
  bool enable_active_mesh_pub_;
  bool enable_segmented_mesh_pub_;

  BoundingBox::Type bounding_box_type_;

  ros::Publisher active_mesh_vertex_pub_;
  std::unique_ptr<ObjectCloudPublishers> segmented_mesh_vertices_pub_;

  // TODO(nathan) think about replacing this
  kimera::SemanticIntegratorBase::SemanticConfig semantic_config_;
};

}  // namespace incremental
}  // namespace hydra
