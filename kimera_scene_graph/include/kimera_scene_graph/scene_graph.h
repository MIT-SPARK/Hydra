#pragma once

#include <map>
#include <vector>

// For serialization
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <glog/logging.h>

#include <kimera_semantics/common.h>
#include <kimera_semantics/macros.h>

#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/utils/voxblox_to_pcl.h"

#include "kimera_scene_graph/scene_graph_edge.h"
#include "kimera_scene_graph/scene_graph_layer.h"
#include "kimera_scene_graph/scene_graph_node.h"

namespace kimera {

class BaseSceneGraph{
  public:
    virtual ~BaseSceneGraph() = default;
};

class SceneGraph {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(SceneGraph);
  KIMERA_POINTER_TYPEDEFS(SceneGraph);
  //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SceneGraph();
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

  inline LayerIdMap getDatabase() const {
    return database_;
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
    CHECK(layer_id != LayerId::kInvalidLayerId);
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

  /**
   * @brief findNearestSceneGraphNodeInLayer Finds the nearest scene graph
   * node in a given layer (layer_id) to a query 3D point (query_point).
   * Internally, it builds a kd-tree for the scene-graph layer requested,
   * and queries the query_point to get the nearest node of the layer.
   * @param[in] query_point 3D point for which we want to find its nearest
   * neighbor in the given layer_id.
   * @param[in] layer_id Layer id where to find the nearest scene-graph node
   * neighbor of the query_point.
   * @param[out] nearest_scene_graph_node The actual scene-graph node that is
   * the nearest to the query_point.
   * @param[in] n_nearest_neighbors Find up to n nearest neighbors instead of
   * one
   * @return True if we found a nearest neighbor, false otherwise.
   */
  bool findNearestSceneGraphNodeInLayer(
      const NodePosition& query_point,
      const LayerId& layer_id,
      std::vector<SceneGraphNode>* nearest_scene_graph_node,
      const size_t& n_nearest_neighbors = 1u);

  inline void clear() {
    next_inter_layer_edge_id_ = 0;
    database_.clear();
    inter_layer_edge_map_.clear();
  }

  // For serialization (save/load) of the scene-graph
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar& BOOST_SERIALIZATION_NVP(database_);
    ar& BOOST_SERIALIZATION_NVP(next_inter_layer_edge_id_);
    ar& BOOST_SERIALIZATION_NVP(inter_layer_edge_map_);
  }

 protected:
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

  friend class SceneGraphVisualizer;
};

}  // namespace kimera
