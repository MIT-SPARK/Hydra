#pragma once

#include <string>
#include <vector>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include <kimera_semantics/macros.h>

#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/scene_graph.h"
#include "kimera_scene_graph/scene_graph_edge.h"
#include "kimera_scene_graph/scene_graph_layer.h"
#include "kimera_scene_graph/scene_graph_node.h"

namespace kimera {

namespace rvt = rviz_visual_tools;

class SceneGraphVisualizer {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(SceneGraphVisualizer);
  KIMERA_POINTER_TYPEDEFS(SceneGraphVisualizer);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SceneGraphVisualizer(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private);
  virtual ~SceneGraphVisualizer() = default;

  // Visualization
  inline void visualize(const SceneGraph& scene_graph) const {
    visualizeImpl(scene_graph);
  }

  inline void updateEdgeAlpha(const float& alpha) { edge_alpha_ = alpha; }

  inline float getLayerStepZ() const { return layer_step_z_; }

  // Should be private
  visualization_msgs::Marker getLineFromPointToPoint(
      const NodePosition& p1,
      const NodePosition& p2,
      const NodeColor& color,
      const float& edge_scale,
      const std::string& marker_namespace) const;

 protected:
  virtual void visualizeImpl(const SceneGraph& scene_graph) const;
  bool displayCentroids(const SceneGraph& scene_graph) const;
  void displayInterLayerEdges(const SceneGraph& scene_graph) const;
  void displayIntraLayerEdges(const SceneGraph& scene_graph) const;
  visualization_msgs::Marker getMarkerFromSceneGraphEdge(
      const SceneGraph& scene_graph,
      const SceneGraphEdge& edge) const;

  // Visualization marker generators
  visualization_msgs::Marker getCentroidMarker(
      const SceneGraphNode& scene_node) const;

  bool getBoundingBoxMarker(const SceneGraphNode& scene_node,
                            visualization_msgs::Marker* marker) const;

  visualization_msgs::Marker getTextMarker(
      const SceneGraphNode& scene_node) const;
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
  bool getNodeCentroidToPclLineMarker(const SceneGraphNode& node,
                                      visualization_msgs::Marker* marker) const;

  // Interactive marker generators
  visualization_msgs::InteractiveMarkerControl& makeBoxControl(
      visualization_msgs::InteractiveMarker& msg);
  visualization_msgs::Marker makeBox(
      visualization_msgs::InteractiveMarker& msg);

  // Convenience functions
  ColorPoint getColorPointFromNode(const SceneGraphNode& node) const;
  void getDefaultMsgPose(geometry_msgs::Pose* pose) const;
  void getDefaultMsgHeader(std_msgs::Header* header) const;

  // Getters of visualization properties depending on the semantic label.
  std::string getSemanticLabelString(const SemanticLabel& semantic_label) const;
  float getLayerZLevel(const LayerId& layer_id) const;
  float getSemanticPclEdgeScale(const SemanticLabel& semantic_label) const;
  float getSemanticPclEdgeAlpha(const SemanticLabel& semantic_label) const;
  float getSemanticCentroidScale(const SemanticLabel& semantic_label) const;
  float getLayerIdCentroidAlpha(const LayerId& layer_id) const;
  size_t getSemanticDropoutRatio(const SemanticLabel& node_label) const;

 protected:
  // ROS handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // ROS publishers
  ros::Publisher semantic_instance_centroid_pub_;
  ros::Publisher bounding_box_pub_;
  ros::Publisher edges_centroid_pcl_pub_;
  ros::Publisher edges_node_node_pub_;
  ros::Publisher text_markers_pub_;
  SemanticRosPublishers<NodeId, NodePcl> node_pcl_publishers_;

  // Reference frame id
  std::string world_frame_;

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

}  // namespace kimera
