#pragma once

#include <map>
#include <unordered_map>
#include <vector>

#include <glog/logging.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <kimera_semantics/color.h>
#include <kimera_semantics/common.h>

#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/semantic_ros_publishers.h"

namespace kimera {

typedef ColoredPointCloud NodePcl;

// Forward declare what a SceneNode is.
class SceneNode;

typedef std::unordered_map<NodeId, SceneNode*> NodeParents;
typedef std::unordered_map<NodeId, SceneNode*> NodeSiblings;
typedef std::unordered_map<NodeId, SceneNode*> NodeChildren;

typedef uint8_t InstanceId;
typedef Eigen::Vector3i NodeColor;


struct NodeAttributes {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Timestamp timestamp_;
  NodePosition position_;
  NodeOrientation orientation_;
  NodeColor color_;
  SemanticLabel semantic_label_;
  InstanceId instance_id_;
  // 3D points associated to this Node.
  NodePcl::Ptr pcl_;
};

struct NodeFamily {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  NodeParents parents_;
  NodeSiblings siblings_;
  NodeChildren children_;
};

struct SceneNode {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  NodeId id_;
  NodeAttributes attributes_;
  NodeFamily family_;
};

typedef std::unordered_map<NodeId, SceneNode> NodeIdMap;
typedef std::pair<NodeId, NodeId> Edge;
typedef std::vector<Edge> EdgeList;

class SceneGraph {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SceneGraph(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : nh_(nh),
        nh_private_(nh_private),
        semantic_instance_centroid_pub_(),
        text_markers_pub_(),
        node_pcl_publishers_("centroid", nh_private),
        edge_pub_() {
    // Params
    nh_private_.param("world_frame", world_frame_, world_frame_);

    // Publishers
    semantic_instance_centroid_pub_ = nh_private_.advertise<ColoredPointCloud>(
        "semantic_instance_centroid", 1, true);
    text_markers_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
        "instance_ids", 1, true);
    edge_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
        "edges", 1, true);
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

  // Visualization
  inline void visualize() { visualizeImpl(); }

  inline void updateEdgeAlpha(const float& alpha) { edge_alpha_ = alpha; }

 private:
  virtual void visualizeImpl() {
    ColoredPointCloud centroid_pointcloud;
    visualization_msgs::MarkerArray line_assoc_markers;
    visualization_msgs::MarkerArray text_markers;
    for (const auto& it : database_) {
      LOG(INFO) << "Publish centroid for SceneNode with id: " << it.first;
      // TODO(Toni): Color the centroids with Semantic label and add instance
      // id! Publish centroids
      const NodeId& node_id = it.second.id_;
      const SemanticLabel& node_label = it.second.attributes_.semantic_label_;
      const InstanceId& node_instance_id = it.second.attributes_.instance_id_;
      NodePosition object_position = it.second.attributes_.position_;
      const NodeColor& node_color = it.second.attributes_.color_;

      // Shift centroid in z, for scene-graph visualization
      object_position.z += getSemanticZLevel(node_label);

      ColorPoint colored_centroid;
      colored_centroid.x = object_position.x;
      colored_centroid.y = object_position.y;
      colored_centroid.z = object_position.z;
      colored_centroid.r = node_color.x();
      colored_centroid.g = node_color.y();
      colored_centroid.b = node_color.z();
      centroid_pointcloud.push_back(colored_centroid);

      // Publish text on top of each centroid
      text_markers.markers.push_back(getTextMarker(
          "centroid_ids", "Id: " + std::to_string(node_instance_id)));

      // Publish edges from centroid to its associated pointcloud
      if (it.second.attributes_.pcl_) {
        NodePcl object_pcl = *it.second.attributes_.pcl_;
        static constexpr float kPclShift = -5.0;
        for (auto& it : object_pcl.points)
          it.z = getSemanticZLevel(node_label) + kPclShift;
        const auto& marker =
            getLinesFromPointToPointCloud(object_position, object_pcl);
        line_assoc_markers.markers.push_back(marker);
      }
    }
    // Publish centroids positions
    centroid_pointcloud.header.frame_id = world_frame_;
    semantic_instance_centroid_pub_.publish(centroid_pointcloud);
    text_markers_pub_.publish(text_markers);
    edge_pub_.publish(line_assoc_markers);
  }

  float getSemanticZLevel(const SemanticLabel& semantic_label) {
    // Display each layer 0.5 meters apart.
    static constexpr float kStepZ = 0.5;
    switch (semantic_label) {
      case kRoomSemanticLabel:
        return 20 * kStepZ;
      default:
        return 0.0;
    }
  }

  visualization_msgs::Marker getTextMarker(const std::string& marker_namespace,
                                           const std::string& marker_text) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = marker_namespace;
    static int marker_id = 0;
    marker.id = ++marker_id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 1.0;
    marker.pose.position.z = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.text = marker_text;

    marker.scale.z = 0.1;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    return marker;
  }

  visualization_msgs::Marker getLinesFromPointToPointCloud(
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

    marker.color.a = edge_alpha_;
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
      color.a = edge_alpha_;

      marker.colors[2u * i] = color;
      marker.colors[(2u * i) + 1u] = color;

      marker.points[2u * i] = center_point;
      marker.points[(2u * i) + 1u] = vtx;
    }

    // REMOVE
    marker.text = "Does this work?";
    LOG(WARNING) << "Using edge alpha: " << edge_alpha_;

    return marker;
  }

 protected:
  NodeIdMap database_;
  EdgeList edges_;

  // ROS related things
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  std::string world_frame_;
  ros::Publisher semantic_instance_centroid_pub_;
  SemanticRosPublishers<NodeId, NodePcl> node_pcl_publishers_;
  ros::Publisher edge_pub_;
  ros::Publisher text_markers_pub_;

  // Visualization params.
  float edge_alpha_ = 0.4;
};

class SceneGraphVisualizer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
