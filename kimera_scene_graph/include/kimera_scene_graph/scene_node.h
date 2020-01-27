#pragma once

#include <map>
#include <unordered_map>
#include <vector>

#include <glog/logging.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include <kimera_semantics/color.h>
#include <kimera_semantics/common.h>

#include <pcl_ros/point_cloud.h>

#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/semantic_ros_publishers.h"

namespace kimera {

namespace rvt = rviz_visual_tools;

typedef ColorPointCloud NodePcl;

// Forward declare what a SceneNode is.
class SceneNode;
typedef std::unordered_map<NodeId, SceneNode*> NodeParents;
typedef std::unordered_map<NodeId, SceneNode*> NodeSiblings;
typedef std::unordered_map<NodeId, SceneNode*> NodeChildren;

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
  BoundingBox<ColorPoint> bounding_box_;

  std::string print() const {
    std::stringstream ss;
    // clang-format off
    ss << "Attributes: \n"
        << "Timestamp : " << timestamp_ << '\n'
        << "Position : " << position_ << '\n'
        << "Orientation : " << orientation_ << '\n'
        << "Color : " << color_ << '\n'
        << "Semantic Label: " << std::to_string(semantic_label_) << '\n'
        << "Instance Id: " << std::to_string(instance_id_) << '\n'
        << "Pcl size: " << pcl_ ? std::to_string(pcl_->size()) : "no pcl attached...";
    // clang-format on
    return ss.str();
  }
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

  std::string print() const {
    std::stringstream ss;
    ss << "Node " << id_ << ":\n" << attributes_.print();
    return ss.str();
  }
};

typedef std::unordered_map<NodeId, SceneNode> NodeIdMap;
typedef std::pair<NodeId, NodeId> Edge;
typedef std::vector<Edge> EdgeList;
class SceneGraph {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SceneGraph(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  virtual ~SceneGraph() = default;

 public:
  /**
   * @brief getSemanticInstanceById
   * @param id: instance id of the semantic instance that needs to be
   * retrieved.
   * @param semantic_instance: retrieved semantic instance.
   * @return False if the id could not be found, true otherwise.
   */
  bool getSceneNodeById(const NodeId& id, SceneNode* scene_node) const;

  /**
   * @brief getAllSceneNodes returns a vector of pointers to the internals of the
   * database that scene graph saves. Everything is returned as const& so the
   * user cannot recklessly modify the database...
   * @param scene_nodes
   */
  inline void getAllSceneNodes(std::vector<const SceneNode*>* scene_nodes) {
    CHECK_NOTNULL(scene_nodes);
    scene_nodes->resize(database_.size());
    size_t i = 0;
    for (const auto& kv : database_) {
      scene_nodes->at(i) = &kv.second;
      ++i;
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
  bool addSemanticInstanceSafely(const SceneNode& scene_node);

  /**
   * @brief addEdge Adds an edge to the graph
   * @param id_1 id of the first node
   * @param id_2 id of the second node
   */
  void addEdge(const NodeId& id_1, const NodeId& id_2);

  inline EdgeList getEdges() const { return edges_; }

  void getSemanticLayer(const SemanticLabel& label,
                        NodeIdMap* semantic_layer) const;

  inline void clear() {
    database_.clear();
    edges_.clear();
  }

  // Visualization
  inline void visualize() { visualizeImpl(); }

  inline void updateEdgeAlpha(const float& alpha) { edge_alpha_ = alpha; }

  // Getters
  inline float getLayerStepZ() const { return layer_step_z_; }

  // TODO(Toni): should be private!
  visualization_msgs::Marker getLineFromPointToPoint(
      const NodePosition& p1,
      const NodePosition& p2,
      const NodeColor& color,
      const float& edge_scale,
      const std::string& marker_namespace) const;

 protected:
  virtual void visualizeImpl() const;
  void displayCentroids() const;
  void displayEdges() const;

  // Getters of visualization properties depending on the semantic label.
  std::string getSemanticLabelString(const SemanticLabel& semantic_label) const;
  float getSemanticZLevel(const SemanticLabel& semantic_label) const;
  float getSemanticPclEdgeScale(const SemanticLabel& semantic_label) const;
  float getSemanticPclEdgeAlpha(const SemanticLabel& semantic_label) const;
  float getSemanticCentroidScale(const SemanticLabel& semantic_label) const;
  float getSemanticCentroidAlpha(const SemanticLabel& semantic_label) const;
  size_t getSemanticDropoutRatio(const SemanticLabel& node_label) const;

  // Visualization marker generators
  visualization_msgs::Marker getCentroidMarker(
      const SceneNode& scene_node) const;

  bool getBoundingBoxMarker(const SceneNode& scene_node,
                            visualization_msgs::Marker* marker) const;

  visualization_msgs::Marker getTextMarker(const SceneNode& scene_node) const;
  visualization_msgs::Marker getTextMarker(
      const NodePosition& node_position,
      const std::string& marker_namespace,
      const std::string& marker_text) const;

  visualization_msgs::Marker getLinesFromPointToPointCloud(
      const NodePosition& position,
      const NodeColor& color,
      const NodePcl& pcl,
      const std::string& marker_namespace,
      const float& edge_scale,
      const float& edge_alpha,
      const size_t& dropout_lines = 1u) const;

  bool getNodeCentroidToPclLineMarker(const SceneNode& node,
                                      visualization_msgs::Marker* marker) const;

  // Interactive marker generators
  visualization_msgs::InteractiveMarkerControl& makeBoxControl(
      visualization_msgs::InteractiveMarker& msg);

  visualization_msgs::Marker makeBox(
      visualization_msgs::InteractiveMarker& msg);

  // Convenience functions
  ColorPoint getColorPointFromNode(const SceneNode& node) const;

  void getDefaultMsgPose(geometry_msgs::Pose* pose) const;
  void getDefaultMsgHeader(std_msgs::Header* header) const;

 protected:
  /// Contains the scene nodes and its attributes
  NodeIdMap database_;

  // TODO(Toni): store all edges, including skeleton ones here, but tag edge
  // by their type.
  /// List of edges between scene nodes (these edges are currently only for the
  /// intra-class dependencies, the inter-class is done with pcls)
  EdgeList edges_;

  // ROS related things
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  std::string world_frame_;
  ros::Publisher semantic_instance_centroid_pub_;
  ros::Publisher bounding_box_pub_;
  SemanticRosPublishers<NodeId, NodePcl> node_pcl_publishers_;
  ros::Publisher edges_centroid_pcl_pub_;
  ros::Publisher edges_node_node_pub_;
  ros::Publisher text_markers_pub_;

  // Create an interactive marker server on the topic namespace simple_marker
  interactive_markers::InteractiveMarkerServer server;

  // Visualization params.
  float edge_alpha_ = 0.1;
  float edge_scale_ = 0.01;

  // Step in Z axis for the different layers of the scene graph
  float layer_step_z_ = 15;

  // For visualizing cuboid wireframes in rviz
  rvt::RvizVisualToolsPtr visual_tools_;
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
