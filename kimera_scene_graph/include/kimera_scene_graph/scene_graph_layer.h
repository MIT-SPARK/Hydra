#pragma once

#include <map>
#include <vector>

// For serialization
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <glog/logging.h>

#include <kimera_semantics/color.h>
#include <kimera_semantics/common.h>

#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/scene_graph_edge.h"
#include "kimera_scene_graph/scene_graph_node.h"

namespace kimera {

struct BaseSceneGraphLayer {
  public:
    virtual ~BaseSceneGraphLayer() = default;
};

class SceneGraphLayer {
 public:
  //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SceneGraphLayer(); // for serialization in a map...
  SceneGraphLayer(const LayerId& layer_id);
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
  inline void setLayerId(const LayerId& layer_id) { layer_id_ = layer_id; }

  //! Checkers
  inline bool hasNode(const NodeId& node_id) const {
    return node_map_.find(node_id) != node_map_.end();
  }
  inline bool hasEdge(const EdgeId& edge_id) const {
    return intra_layer_edge_map_.find(edge_id) != intra_layer_edge_map_.end();
  }

  //! Utils
  ColorPointCloud::Ptr convertLayerToPcl(
      std::map<int, NodeId>* cloud_to_graph_ids = nullptr,
      std::vector<NodeId>* vertex_ids = nullptr) const;

  /**
   * @brief findNearestSceneGraphNode Finds the nearest scene graph
   * node to a query 3D point (query_point).
   * Internally, it converts the scene graph layer to a pointcloud and builds
   * a kd-tree out of it to answer NN queries.
   * @param[in] query_point 3D point for which we want to find its nearest
   * neighbor in the given layer_id.
   * @param[out] nearest_scene_graph_node The actual scene-graph node (or nodes)
   * that are the nearest to the query_point.
   * @param[in] k_nearest_neighbors Number of nearest neighbors to find
   * @return True if we found a nearest neighbor, false otherwise.
   */
  bool findNearestSceneGraphNode(
      const NodePosition& query_point,
      std::vector<SceneGraphNode>* nearest_scene_graph_nodes,
      const size_t& k_nearest_neighbors = 1u);

 protected:
  //! Protected adders
  /**
   * @brief addNode This is protected because it should not be called by the
   * user directly (rather use the addNode of the scene graph!).
   * @param node SceneGraph node to be added
   * @return True if the node id was not there, false otherwise
   */
  bool addNode(const SceneGraphNode& node);

  /**
   * @brief addIntraLayerEdge This is protected because it should not be called
   * by the user directly (rather use the addEdge of the scene graph!).
   * @param edge Edge to be added, the edge itself is updated with an edge id
   * given by the layer it is in.
   */
  void addIntraLayerEdge(SceneGraphEdge* edge);

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar& BOOST_SERIALIZATION_NVP(layer_id_);
    ar& BOOST_SERIALIZATION_NVP(node_map_);
    ar& BOOST_SERIALIZATION_NVP(next_intra_layer_edge_id_);
    ar& BOOST_SERIALIZATION_NVP(intra_layer_edge_map_);
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

}  // namespace kimera
