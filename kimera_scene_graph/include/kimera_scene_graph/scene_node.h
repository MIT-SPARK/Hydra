#pragma once

#include <map>
#include <unordered_map>
#include <vector>

#include <glog/logging.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "kimera_scene_graph/common.h"
#include "kimera_semantics/common.h"

namespace kimera {

typedef ColoredPointCloud NodePcl;
typedef std::unordered_map<NodeId, ros::Publisher> NodePclPublishers;

// Forward declare what a SceneNode is.
class SceneNode;
typedef std::unordered_map<NodeId, SceneNode*> NodeParents;
typedef std::unordered_map<NodeId, SceneNode*> NodeSiblings;
typedef std::unordered_map<NodeId, SceneNode*> NodeChildren;

typedef uint8_t InstanceId;

struct NodeAttributes {
  Timestamp timestamp_;
  NodePosition position_;
  NodeOrientation orientation_;
  SemanticLabel semantic_label_;
  InstanceId instance_id_;
  // 3D points associated to this Node.
  NodePcl::Ptr pcl_;
};

struct NodeFamily {
  NodeParents parents_;
  NodeSiblings siblings_;
  NodeChildren children_;
};

struct SceneNode {
  NodeId id_;
  NodeAttributes attributes_;
  NodeFamily family_;
};

typedef std::unordered_map<NodeId, SceneNode> NodeIdMap;
typedef std::pair<NodeId, NodeId> Edge;
typedef std::vector<Edge> EdgeList;
class SceneGraph {
 public:
  SceneGraph(const ros::NodeHandle& nh,
             const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      semantic_instance_centroid_pub_(),
      node_pcl_publishers_(),
      edge_pub_() {
    // Params
    nh_private_.param("world_frame", world_frame_, world_frame_);

    // Publishers
    semantic_instance_centroid_pub_ = nh_private_.advertise<ColoredPointCloud>(
        "semantic_instance_centroid_pub", 1, true);
    edge_pub_ = nh_private_.advertise<visualization_msgs::Marker>(
      "edges_pub", 1, true);
  }
  virtual ~SceneGraph() = default;

 public:
  /**
   * @brief getSemanticInstanceById
   * @param id: instance id of the semantic instance that needs to be
   * retrieved.
   * @param semantic_instance: retrieved semantic instance.
   * @return False if the id could not be found, true otherwise.
   */
  bool getSceneNodeById(const NodeId& id, SceneNode* scene_node) const {
    CHECK_NOTNULL(scene_node);
    const auto& it = database_.find(id);
    if (it == database_.end()) {
      return false;
    } else {
      *scene_node = it->second;
      return true;
    }
  }

  /**
   * @brief addSemanticInstance to the database, but overrides if there was
   * already an instance in the database.
   * @param semantic_instance
   */
  inline void addSceneNode(const SceneNode& scene_node) {
    database_[scene_node.id_] = scene_node;
  }

  /**
   * @brief addSemanticInstanceSafely adds the semantic instance to the
   * database only if there was no previously an instance with the same id
   * in the database.
   * @param semantic_instance
   * @return true if the semantic instance was not in the database and the
   * semantic instance has been added to the database, false
   * otherwise.
   */
  inline bool addSemanticInstanceSafely(const SceneNode& scene_node) {
    const auto& it = database_.find(scene_node.id_);
    if (it == database_.end()) {
      return false;
    } else {
      it->second = scene_node;
      return true;
    }
  }

  /**
   * @brief addEdge Adds an edge to the graph
   * @param id_1 id of the first node
   * @param id_2 id of the second node
   */
  inline void addEdge(const NodeId& id_1, const NodeId& id_2) {
    Edge edge;
    edge.first = id_1;
    edge.second = id_2;
    edges_.push_back(edge);
  }

  inline EdgeList getEdges() const { return edges_; }

  // TODO(Toni): delete pls
  // inline NodeIdMap getFullDatabase() {
  //   return database_;
  // }

  inline void clear() {
    database_.clear();
    edges_.clear();
  }

  inline void visualize() {
    visualizeImpl();
  }

private:
  virtual void visualizeImpl() {
    PointCloud centroid_pointcloud;
    for (const auto& it: database_) {
      LOG(INFO) << "Publish centroid for SceneNode with id: " << it.first;
      // TODO(Toni): Color the centroids with Semantic label and add instance id!
      // Publish centroids
      const NodePosition& object_position = it.second.attributes_.position_;
      centroid_pointcloud.push_back(object_position);

      // Publish centroids' associated pointclouds.
      // TODO(Toni): do not create a pcl publisher for each object just add edges
      // between pcls and centroid, the user can then directly see what pcl
      // is associated to each centroid.
      const NodeId& node_id = it.second.id_;
      const auto& pub_it = node_pcl_publishers_.find(node_id);
      if (pub_it == node_pcl_publishers_.end()) {
        // Didn't find a ROS publisher for this Node Pointcloud, add one.
        node_pcl_publishers_[node_id] = nh_private_.advertise<NodePcl>(
            "node_pcl_" + node_id, 1, true);
      }
      // Publish centroid's pointcloud
      NodePcl& object_pcl = *it.second.attributes_.pcl_;
      object_pcl.header.frame_id = world_frame_;
      node_pcl_publishers_.at(node_id).publish(object_pcl);

      // Publish edges from centroid to its associated pointcloud
      visualizeLinesFromPointToPointCloud(object_position, object_pcl);
    }
    // Publish centroids positions
    centroid_pointcloud.header.frame_id = world_frame_;
    semantic_instance_centroid_pub_.publish(centroid_pointcloud);
  }

  void visualizeLinesFromPointToPointCloud(
      const NodePosition& position,
      const NodePcl& pcl) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.header.stamp = ros::Time();

    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    static int marker_id = 1u;
    marker.id = ++marker_id;
    marker.ns = "node_edges";

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    geometry_msgs::Point center_point;
    center_point.x = position.x;
    center_point.y = position.y;
    center_point.z = position.z;

    const size_t& n_lines = pcl.size();
    marker.points.resize(2u * n_lines);
    marker.colors.resize(2u * n_lines);
    for (size_t i = 0u; i < n_lines; ++i) {
      const NodePcl::PointType& point = pcl.at(i);
      geometry_msgs::Point vtx;
      vtx.x = point.x;
      vtx.y = point.y;
      vtx.z = point.z;

      std_msgs::ColorRGBA color;
      color.r = point.r;
      color.g = point.g;
      color.b = point.b;
      color.a = point.a;

      marker.colors[2u * i] = color;
      marker.colors[(2u * i) + 1u] = color;

      marker.points[2u * i] = center_point;
      marker.points[(2u * i) + 1u] = vtx;
    }

    edge_pub_.publish(marker);
  }

 protected:
  NodeIdMap database_;
  EdgeList edges_;

  // ROS related things
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  std::string world_frame_;
  ros::Publisher semantic_instance_centroid_pub_;
  NodePclPublishers node_pcl_publishers_;
  ros::Publisher edge_pub_;
};

class SceneGraphVisualizer {
 public:
  SceneGraphVisualizer() = default;
  ~SceneGraphVisualizer() = default;

 public:
  static void drawSceneGraph(const SceneGraph& semantic_instance_database) {
    const EdgeList& edges = semantic_instance_database.getEdges();

    for (const auto& edge : edges) {
      const NodeId& id_1 = edge.first;
      const NodeId& id_2 = edge.second;
      SceneNode node_1;
      SceneNode node_2;
      semantic_instance_database.getSceneNodeById(id_1, &node_1);
      semantic_instance_database.getSceneNodeById(id_2, &node_2);
      // Draw edge (color depends on node types).
      // TODO(Toni)
    }
  }
};

}  // namespace kimera
