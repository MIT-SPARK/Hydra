#pragma once

#include <map>
#include <vector>
#include <string>

#include <glog/logging.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include <kimera_semantics/color.h>
#include <kimera_semantics/common.h>
#include <kimera_semantics/macros.h>

#include <voxblox_skeleton/skeleton.h>

#include <pcl_ros/point_cloud.h>

#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/semantic_ros_publishers.h"
#include "kimera_scene_graph/utils/voxblox_to_pcl.h"

#include "kimera_scene_graph/scene_graph_node.h"
#include "kimera_scene_graph/scene_graph_edge.h"
#include "kimera_scene_graph/scene_graph_layer.h"

namespace kimera {

namespace rvt = rviz_visual_tools;

class SceneGraph {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(SceneGraph);
  KIMERA_POINTER_TYPEDEFS(SceneGraph);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SceneGraph(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  virtual ~SceneGraph() = default;

 public:
  /**
   * @brief getSceneNodeById
   * @param layer_id: Unique Id of the layer in the scene graph where the node
   * belongs
   * @param node_id: Unique instance id of the node that needs to
   * be retrieved.
   * @param scene_node: retrieved node in the scene graph.
   * @return False if the node could not be found, true otherwise.
   */
  bool getNode(const LayerId& layer_id,
               const NodeId& node_id,
               SceneGraphNode* scene_node) const;

  /**
   * @brief getAllSceneNodes Returns all copies, potentially very slow.
   * @param scene_nodes
   */
  inline void getAllSceneNodes(std::vector<SceneGraphNode>* scene_nodes) const {
    CHECK_NOTNULL(scene_nodes);
    scene_nodes->clear();
    for (const std::pair<LayerId, SceneGraphLayer>& kv_layers : database_) {
      // We need to call the getNodeIdMapMutable to ensure we point to the
      // internals and not scope-limited copies.
      for (const std::pair<NodeId, SceneGraphNode>& kv_nodes :
           kv_layers.second.getNodeIdMap()) {
        scene_nodes->push_back(kv_nodes.second);
      }
    }
    CHECK_EQ(scene_nodes->size(), getNumberOfUniqueSceneNodes());
  }

  inline EdgeIdMap getInterLayerEdgesMap() const {
    return inter_layer_edge_map_;
  }

  inline SceneGraphEdge getInterLayerEdge(const EdgeId& edge_id) const {
    CHECK(hasInterLayerEdge(edge_id));
    return inter_layer_edge_map_.at(edge_id);
  }
  inline SceneGraphEdge& getInterLayerEdgeMutable(const EdgeId& edge_id) {
    CHECK(hasInterLayerEdge(edge_id));
    return inter_layer_edge_map_.at(edge_id);
  }

  /**
   * @brief getLayer Get a layer of the scene graph, but without copying.
   * Handle with care as this may lead to a dangling reference if not careful.
   * @param layer_id Id of the desired layer. If the layer does not exist this
   * will throw!
   * @return Const reference to a Layer (if you want to modify the layer
   * use getLayerMutable).
   */
  const SceneGraphLayer& getLayer(const LayerId& layer_id) const;
  /**
   * @brief getLayer COPIES the layer. If you want to modify the layer of the
   * scene graph, use getLayerMutable.
   * @param layer_id
   * @param layer
   * @return
   */
  bool getLayerSafe(const LayerId& layer_id, SceneGraphLayer* layer) const;
  /**
   * @brief getLayerMutable Provides a reference to an internal layer of
   * the scene graph for update/modification (mind that the scene graph layer
   * is not threadsafe). If you only need read access, use getLayer instead.
   * @param layer_id Id of the layer in the scene graph to retrieve. If this
   * layer does not exist, the function returns a Nullptr, so always check the
   * return is a valid pointer.
   * @return A pointer to the underlying layer, nullptr if the layer does not
   * exist
   */
  SceneGraphLayer* getLayerMutable(const LayerId& layer_id);

  /**
   * @brief getNumberOfUniqueSceneNodes
   * @return
   */
  inline size_t getNumberOfUniqueSceneNodes() const {
    size_t total_scene_nodes = 0u;
    for (const std::pair<LayerId, SceneGraphLayer>& key_value_layers :
         database_) {
      size_t scene_nodes_per_layer = key_value_layers.second.getNumberOfNodes();
      total_scene_nodes += scene_nodes_per_layer;
    }
    return total_scene_nodes;
  }

  /**
   * @brief addSceneNode to the database, but throws if there was
   * already an instance in the database.
   * @param scene_node
   */
  inline void addSceneNode(const SceneGraphNode& scene_node) {
    LayerId layer_id = scene_node.layer_id_;
    if (!hasLayer(layer_id)) {
      LOG(INFO) << "Creating new layer with id: " << to_underlying(layer_id);
      database_.insert(std::make_pair(layer_id, SceneGraphLayer(layer_id)));
    }
    SceneGraphLayer& layer = database_.at(layer_id);
    CHECK(layer.addNode(scene_node)) << "Node with id: " << scene_node.node_id_
                                     << " is already in graph...";
  }

  /**
   * @brief addEdge it can be an inter or an intra layer edge. If it is an intra
   * layer edge (edge inside a layer), we call the layer's implementation of
   * addIntraLayerEdge.
   * @param[in/out] scene_graph_edge The edge to be added, its edge_id_ member
   * will be
   * updated.
   */
  void addEdge(SceneGraphEdge* scene_graph_edge);

  // `Has' queries
  inline bool hasLayer(const LayerId& layer_id) const {
    return database_.find(layer_id) != database_.end();
  }
  inline bool hasNode(const LayerId& layer_id, const NodeId& node_id) const {
    bool has_node = false;
    if (hasLayer(layer_id)) {
      const SceneGraphLayer& layer = database_.at(layer_id);
      has_node = layer.hasNode(node_id);
    }
    return has_node;
  }
  inline bool hasInterLayerEdge(const EdgeId& edge_id) const {
    return inter_layer_edge_map_.find(edge_id) != inter_layer_edge_map_.end();
  }

  inline void clear() {
    next_inter_layer_edge_id_ = 0;
    database_.clear();
    inter_layer_edge_map_.clear();
  }

  // Visualization
  inline void visualize() const { visualizeImpl(); }

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
  bool displayCentroids() const;
  void displayInterLayerEdges() const;
  void displayIntraLayerEdges() const;
  visualization_msgs::Marker getMarkerFromSceneGraphEdge(
      const SceneGraphEdge& edge) const;

  // Getters of visualization properties depending on the semantic label.
  std::string getSemanticLabelString(const SemanticLabel& semantic_label) const;
  float getLayerZLevel(const LayerId& layer_id) const;
  float getSemanticPclEdgeScale(const SemanticLabel& semantic_label) const;
  float getSemanticPclEdgeAlpha(const SemanticLabel& semantic_label) const;
  float getSemanticCentroidScale(const SemanticLabel& semantic_label) const;
  float getLayerIdCentroidAlpha(const LayerId& layer_id) const;
  size_t getSemanticDropoutRatio(const SemanticLabel& node_label) const;

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

  void addInterLayerEdge(SceneGraphEdge* scene_graph_edge);

  inline bool isParent(const LayerId& start, const LayerId& end) const {
    return start > end;
  }

  inline bool isChild(const LayerId& start, const LayerId& end) const {
    return start < end;
  }

  inline bool isSibling(const LayerId& start, const LayerId& end) const {
    return start == end;
  }

 protected:
  /// Contains the scene nodes and its attributes
  LayerIdMap database_;

  /// Map of edges between scene graph layers (inter-layer). The intra-layer
  /// edges are stored on a per layer basis
  EdgeId next_inter_layer_edge_id_ = 0;
  EdgeIdMap inter_layer_edge_map_;

  // TODO(TONI): move all of this visualization/ROS out of scene graph in
  // scene graph visualizer...
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

}  // namespace kimera
