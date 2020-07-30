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

#include <voxblox_skeleton/skeleton.h>

#include <pcl_ros/point_cloud.h>

#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/semantic_ros_publishers.h"
#include "kimera_scene_graph/utils/voxblox_to_pcl.h"

namespace kimera {

namespace rvt = rviz_visual_tools;

typedef ColorPointCloud NodePcl;

// Forward declare what a SceneNode is.
class SceneGraphNode;

struct NodeAttributes {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Timestamp timestamp_ = 0u;
  NodePosition position_ = NodePosition();
  NodeOrientation orientation_ = NodeOrientation();
  NodeColor color_ = NodeColor();
  SemanticLabel semantic_label_ = 0u;
  // This is what will be shown as label in the scene graph visualization
  NodeName name_ = "";
  // 3D points associated to this Node.
  NodePcl::Ptr pcl_ = nullptr;
  BoundingBox<ColorPoint> bounding_box_ = BoundingBox<ColorPoint>();

  std::string print() const {
    std::stringstream ss;
    // clang-format off
    ss << "Attributes: \n"
        << "Timestamp : " << std::to_string(timestamp_) << '\n'
        << "Position : " << position_ << '\n'
        << "Orientation : " << orientation_ << '\n'
        << "Color : " << color_ << '\n'
        << "Semantic Label: " << std::to_string(semantic_label_) << '\n'
        << "Name: " << name_ << '\n'
        // << "Pcl size: " << pcl_ ? std::to_string(pcl_->size()) : "no pcl attached..."
           ;
    // clang-format on
    return ss.str();
  }
};

struct SceneGraphEdge {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @brief isInterLayerEdge
   * @return returns true if this edge is between layers (inter-layer)
   * or between the nodes in a layer (intra-layer).
   */
  inline bool isInterLayerEdge() const {
    return start_layer_id_ != end_layer_id_;
  }
  inline bool isEdgeValid() const {
    return edge_id_ != -1 && !isSelfEdge() &&
           start_layer_id_ != LayerId::kInvalidLayerId &&
           end_layer_id_ != LayerId::kInvalidLayerId;
  }
  inline bool isSelfEdge() const {
    return (start_layer_id_ == end_layer_id_) &&
           (start_node_id_ == end_node_id_);
  }
  bool swapDirection() {
    CHECK(isEdgeValid());
    std::swap(start_layer_id_, end_layer_id_);
    std::swap(start_node_id_, end_node_id_);
  }
  std::string print() const {
    std::stringstream ss;
    // clang-format off
    ss << "SceneGraphEdge: \n"
       << "EdgeId : " << edge_id_ << '\n'
       << "Start Layer Id : " << to_underlying(start_layer_id_) << '\n'
       << "Start Node Id : " << start_node_id_ << '\n'
       << "End Layer Id : " << to_underlying(end_layer_id_) << '\n'
       << "End Node Id : " << end_node_id_;
    // clang-format on
    return ss.str();
  }

 public:
  // These could be protected if we add a ctor and a friend class for SceneGraph
  EdgeId edge_id_ = -1;
  LayerId start_layer_id_ = LayerId::kInvalidLayerId;
  NodeId start_node_id_ = -1;
  LayerId end_layer_id_ = LayerId::kInvalidLayerId;
  NodeId end_node_id_ = -1;
};
typedef std::map<EdgeId, SceneGraphEdge> EdgeIdMap;

inline std::vector<EdgeId> getEdgeIdsInEdgeIdMap(const EdgeIdMap& edge_id_map) {
  std::vector<EdgeId> edge_ids;
  for (const std::pair<EdgeId, SceneGraphEdge>& edge : edge_id_map) {
    edge_ids.push_back(edge.first);
  }
  return edge_ids;
}

struct SceneGraphNode {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  inline bool hasParent() const { return parent_edge_.isEdgeValid(); }
  inline bool hasSiblings() const { return !siblings_edge_map_.empty(); }
  inline bool hasSiblingEdge(const EdgeId& edge_id) const {
    return siblings_edge_map_.find(edge_id) != siblings_edge_map_.end();
  }

  bool checkSiblingEdgeMap() const {
    for (const std::pair<EdgeId, SceneGraphEdge>& edge_pair :
         siblings_edge_map_) {
      const SceneGraphEdge& edge = edge_pair.second;
      if (edge.start_node_id_ != node_id_ && edge.end_node_id_ != node_id_) {
        // The current edge is not connected to this node...
        return false;
      } else {
        CHECK_EQ(edge.start_node_id_, edge.end_node_id_);
        if (edge.start_node_id_ == node_id_ &&
            edge.start_layer_id_ != layer_id_) {
          // The current edge start node is this node, but the layers do not
          // match...
          return false;
        }
        if (edge.end_node_id_ == node_id_ && edge.end_layer_id_ != layer_id_) {
          // The current edge end node is this node, but the layers do not
          // match...
          return false;
        }
      }
    }
    return true;
  }

  std::string print() const {
    std::stringstream ss;
    ss << " - Node Id " << std::to_string(node_id_) << '\n'
       << " - Layer Id " << to_underlying(layer_id_) << '\n'
       << attributes_.print();
    return ss.str();
  }

 public:
  //! Properties of the node
  NodeAttributes attributes_ = NodeAttributes();

 public:
  // Try to keep these protected as they are critical to be updated
  // appropriately
  //! Ids
  // TODO(Toni): perhaps init the ids to 0u so we know for sure if the user
  // updated these fields appropriately and didn't just forget...
  NodeId node_id_ = -1;
  LayerId layer_id_ = LayerId::kInvalidLayerId;

  //! Neighborhood of the node: this contains only intra layer edges!
  //! This list should have no duplicated edges.
  EdgeIdMap siblings_edge_map_;
  //! Each node only has one parent
  SceneGraphEdge parent_edge_;
  //! Each node has perhaps many children
  EdgeIdMap children_edge_map_;

  // Give SceneGraph and Layer access to private members of Node
  // friend class SceneGraph;
  // friend class SceneGraphLayer;
};
typedef std::map<NodeId, SceneGraphNode> NodeIdMap;

class SceneGraphLayer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SceneGraphLayer(const LayerId& layer_id) : layer_id_(layer_id) {}
  virtual ~SceneGraphLayer() = default;

 public:
  //! Getters
  inline LayerId getLayerId() const { return layer_id_; }

  /// Copies, but at least you are safe
  inline NodeIdMap getNodeIdMap() const { return node_map_; }
  inline EdgeIdMap getEdgeIdMap() const { return intra_layer_edge_map_; }

  /// Doesn't copy, but be careful, it can lead to dangling references.
  /// We return a const bcs you are not supposed to modify these maps directly.
  inline const NodeIdMap& getNodeIdMapMutable() { return node_map_; }
  inline const EdgeIdMap& getEdgeIdMapMutable() {
    return intra_layer_edge_map_;
  }

  inline SceneGraphNode getNode(const NodeId& node_id) const {
    CHECK(hasNode(node_id));
    return node_map_.at(node_id);
  }
  inline SceneGraphEdge getIntraLayerEdge(const EdgeId& edge_id) const {
    CHECK(hasEdge(edge_id));
    return intra_layer_edge_map_.at(edge_id);
  }
  // Not threadsafe
  inline SceneGraphNode& getNodeMutable(const NodeId& node_id) {
    CHECK(hasNode(node_id));
    return node_map_.at(node_id);
  }
  // Not threadsafe
  inline SceneGraphEdge& getIntraLayerEdgeMutable(const EdgeId& edge_id) {
    CHECK(hasEdge(edge_id));
    return intra_layer_edge_map_.at(edge_id);
  }

  inline size_t getNumberOfNodes() const { return node_map_.size(); }
  inline size_t getNumberOfEdges() const {
    return intra_layer_edge_map_.size();
  }

  //! Setters
  inline bool setLayerId(const LayerId& layer_id) { layer_id_ = layer_id; }

  //! Checkers
  bool hasNode(const NodeId& node_id) const {
    return node_map_.find(node_id) != node_map_.end();
  }
  bool hasEdge(const EdgeId& edge_id) const {
    return intra_layer_edge_map_.find(edge_id) != intra_layer_edge_map_.end();
  }

  //! Utils
  void convertLayerToSkeleton(vxb::SparseSkeletonGraph* skeleton) const {
    CHECK_NOTNULL(skeleton);
    skeleton->clear();
    // Keep track of ids btw vxb and scene graph
    std::map<NodeId, int64_t> node_id_to_vxb_id;

    // Add vertices
    vxb::SkeletonVertex skeleton_vtx;
    for (const std::pair<NodeId, SceneGraphNode>& vtx : node_map_) {
      CHECK_EQ(vtx.first, vtx.second.node_id_);
      skeleton_vtx.room_id = vtx.second.parent_edge_.start_node_id_;
      // This is the ESDF distance (set to random).
      skeleton_vtx.distance = 1u;
      // Set all to be in the same subgraph... Perhaps we should be able to
      // set different subgraph ids depending on whether the node is isolated.
      skeleton_vtx.subgraph_id = 1u;
      // No need to update this, it will be automatically done once we add the
      // edges (and we will not have inter-layer edges).
      DCHECK_EQ(skeleton_vtx.edge_list.size(), 0u);
      skeleton_vtx.point = pclPointToVxbPoint(vtx.second.attributes_.position_);
      // Vxb sets the vertex id, so it will be overwritten...
      // skeleton_vtx.vertex_id = vtx.second.node_id_;
      const auto& vxb_id = skeleton->addVertex(skeleton_vtx);
      node_id_to_vxb_id[vtx.second.node_id_] = vxb_id;
    }
    CHECK_EQ(skeleton->getVertexMap().size(), getNumberOfNodes())
        << "The skeleton and the scene graph layer should have the same "
           "number of nodes!";

    // Add edges
    // TODO(Toni): perhaps check for self-edges?
    vxb::SkeletonEdge skeleton_edge;
    for (const std::pair<EdgeId, SceneGraphEdge>& edge :
         intra_layer_edge_map_) {
      CHECK_EQ(edge.first, edge.second.edge_id_);
      CHECK(edge.second.start_layer_id_ == layer_id_);
      CHECK(edge.second.end_layer_id_ == layer_id_);
      // Vxb sets the edge id, so it will be overwritten...
      // skeleton_edge.edge_id = edge.second.node_id_;
      skeleton_edge.start_vertex =
          node_id_to_vxb_id.at(edge.second.start_node_id_);
      skeleton_edge.end_vertex = node_id_to_vxb_id.at(edge.second.end_node_id_);
      skeleton_edge.start_distance = 1u;
      skeleton_edge.end_distance = 1u;
      // These ar filled when calling add Edge
      // skeleton_edge.start_point;
      // skeleton_edge.end_point;
      skeleton->addEdge(skeleton_edge);
    }
    CHECK_EQ(skeleton->getEdgeMap().size(), getNumberOfEdges())
        << "The skeleton and the scene graph layer should have the same "
           "number of edges!";
  }

  ColorPointCloud::Ptr convertLayerToPcl(
      std::map<int, NodeId>* cloud_to_graph_ids,
      std::vector<NodeId>* vertex_ids) const {
    ColorPointCloud::Ptr skeleton_graph_cloud(new ColorPointCloud);
    // Get a list of all vertices
    skeleton_graph_cloud->resize(getNumberOfNodes());
    size_t i = 0u;
    for (const std::pair<NodeId, SceneGraphNode>& node : node_map_) {
      CHECK_EQ(node.first, node.second.node_id_);
      CHECK(node.second.layer_id_ == layer_id_);
      const Point& position = node.second.attributes_.position_;
      ColorPoint point;
      point.x = position.x;
      point.y = position.y;
      point.z = position.z;
      skeleton_graph_cloud->at(i) = point;
      if (cloud_to_graph_ids) (*cloud_to_graph_ids)[i] = node.first;
      if (vertex_ids) vertex_ids->push_back(node.first);
      ++i;
    }
    CHECK_EQ(i, getNumberOfNodes());
    return skeleton_graph_cloud;
  }

 protected:
  //! Protected adders
  /**
   * @brief addNode This is protected because it should not be called by the
   * user directly (rather use the addNode of the scene graph!).
   * @param node SceneGraph node to be added
   * @return True if the node id was not there, false otherwise
   */
  bool addNode(const SceneGraphNode& node) {
    if (hasNode(node.node_id_)) {
      return false;
    } else {
      node_map_[node.node_id_] = node;
      return true;
    }
  }

  /**
   * @brief addIntraLayerEdge This is protected because it should not be called
   * by the user directly (rather use the addEdge of the scene graph!).
   * @param edge Edge to be added, the edge itself is updated with an edge id
   * given by the layer it is in.
   */
  void addIntraLayerEdge(SceneGraphEdge* edge) {
    CHECK_NOTNULL(edge);
    CHECK(!edge->isInterLayerEdge())
        << "Use addIntraLayerEdge on a layer only for "
           "intra-layer edges. Use addInterEdge on the "
           "scene-graph for inter-layer edges.";

    int64_t edge_id = next_intra_layer_edge_id_++;
    CHECK(!hasEdge(edge_id)) << "Adding an already existing edge...";

    // Update edge id
    edge->edge_id_ = edge_id;

    // Then, update intra layer edge map
    intra_layer_edge_map_[edge_id] = *edge;

    // Update node's edges maps
    CHECK(edge->start_layer_id_ == layer_id_);
    CHECK(hasNode(edge->start_node_id_));
    SceneGraphNode& start_vertex = node_map_[edge->start_node_id_];
    start_vertex.siblings_edge_map_[edge_id] = *edge;

    CHECK(edge->end_layer_id_ == layer_id_);
    CHECK(hasNode(edge->end_node_id_));
    SceneGraphNode& end_vertex = node_map_[edge->end_node_id_];
    end_vertex.siblings_edge_map_[edge_id] = *edge;

    CHECK(edge->isEdgeValid()) << edge->print();
  }

 protected:
  LayerId layer_id_ = LayerId::kInvalidLayerId;

  NodeIdMap node_map_;

  EdgeId next_intra_layer_edge_id_ = 0u;
  EdgeIdMap intra_layer_edge_map_;

  // Give SceneGraph access to private members of Layer
  friend class SceneGraph;
};
//! A map from layer id to a SceneGraphLayer representing
//! the whole scene graph as a collection of layers.
typedef std::map<LayerId, SceneGraphLayer> LayerIdMap;

class SceneGraph {
 public:
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
};
}  // namespace kimera
