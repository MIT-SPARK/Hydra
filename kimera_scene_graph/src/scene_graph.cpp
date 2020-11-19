#include "kimera_scene_graph/scene_graph.h"

#include <map>
#include <unordered_map>
#include <vector>

#include <glog/logging.h>

#include <kimera_semantics/common.h>

#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/semantic_ros_publishers.h"
#include "kimera_scene_graph/utils/kimera_to_voxblox.h"

namespace kimera {

SceneGraph::SceneGraph()
    : database_(), next_inter_layer_edge_id_(0), inter_layer_edge_map_() {}

bool SceneGraph::getNode(const LayerId& layer_id,
                         const NodeId& node_id,
                         SceneGraphNode* scene_node) const {
  CHECK_NOTNULL(scene_node);
  const auto& layer_it = database_.find(layer_id);
  bool found = false;
  if (layer_it != database_.end()) {
    const SceneGraphLayer& layer = layer_it->second;
    *scene_node = layer.getNode(node_id);
    found = true;
  } else {
    LOG(ERROR) << "Requested non-found Scene Node: \n"
               << "- LayerId: " << to_underlying(layer_id) << '\n'
               << "- NodeId: " << node_id;
  }
  return found;
}

void SceneGraph::addEdge(SceneGraphEdge* scene_graph_edge) {
  CHECK(hasLayer(scene_graph_edge->start_layer_id_));
  CHECK(hasLayer(scene_graph_edge->end_layer_id_));
  CHECK(hasNode(scene_graph_edge->start_layer_id_,
                scene_graph_edge->start_node_id_));
  CHECK(
      hasNode(scene_graph_edge->end_layer_id_, scene_graph_edge->end_node_id_));

  if (isSibling(scene_graph_edge->start_layer_id_,
                scene_graph_edge->end_layer_id_)) {
    CHECK(scene_graph_edge->start_layer_id_ == scene_graph_edge->end_layer_id_);
    SceneGraphLayer& layer = database_.at(scene_graph_edge->end_layer_id_);
    layer.addIntraLayerEdge(scene_graph_edge);
  } else {
    addInterLayerEdge(scene_graph_edge);
  }
  CHECK(scene_graph_edge->isEdgeValid());
}

void SceneGraph::addInterLayerEdge(SceneGraphEdge* scene_graph_edge) {
  CHECK(scene_graph_edge->isInterLayerEdge());

  int64_t inter_layer_edge_id = next_inter_layer_edge_id_++;

  CHECK(!hasInterLayerEdge(inter_layer_edge_id))
      << "Adding an already existing inter layer edge with id: "
      << inter_layer_edge_id;

  // First, update the edge's id
  scene_graph_edge->edge_id_ = inter_layer_edge_id;

  // Update node's edges maps
  SceneGraphNode& start_vertex =
      database_.at(scene_graph_edge->start_layer_id_)
          .getNodeMutable(scene_graph_edge->start_node_id_);
  SceneGraphNode& end_vertex =
      database_.at(scene_graph_edge->end_layer_id_)
          .getNodeMutable(scene_graph_edge->end_node_id_);

  // Update scene nodes edge maps (and potentially fix the edge).
  if (isChild(scene_graph_edge->start_layer_id_,
              scene_graph_edge->end_layer_id_)) {
    LOG(WARNING) << "Inter-layer edge with weird family tree. Fixing it...";
    // Modify edge, such that start is the parent and end is the child!
    SceneGraphEdge fixed_scene_graph_edge = *scene_graph_edge;
    fixed_scene_graph_edge.swapDirection();
    end_vertex.children_edge_map_[inter_layer_edge_id] = fixed_scene_graph_edge;
    start_vertex.parent_edge_ = fixed_scene_graph_edge;
    // Update edge with the fix.
    *scene_graph_edge = fixed_scene_graph_edge;
  } else {
    CHECK(isParent(scene_graph_edge->start_layer_id_,
                   scene_graph_edge->end_layer_id_));
    start_vertex.children_edge_map_[inter_layer_edge_id] = *scene_graph_edge;
    end_vertex.parent_edge_ = *scene_graph_edge;
  }

  // Then, add to scene-graph the (potentially fixed) edge.
  inter_layer_edge_map_[inter_layer_edge_id] = *scene_graph_edge;
}

const SceneGraphLayer& SceneGraph::getLayer(const LayerId& layer_id) const {
  LayerIdMap::const_iterator it = database_.find(layer_id);
  CHECK(it != database_.end());
  return it->second;
}

bool SceneGraph::getLayerSafe(const LayerId& layer_id,
                              SceneGraphLayer* layer) const {
  CHECK_NOTNULL(layer);
  LayerIdMap::const_iterator it = database_.find(layer_id);
  bool found = false;
  if (it != database_.end()) {
    *layer = it->second;
    found = true;
  } else {
    found = false;
  }
  return found;
}

SceneGraphLayer* SceneGraph::getLayerMutable(const LayerId& layer_id) {
  LayerIdMap::iterator it = database_.find(layer_id);
  if (it != database_.end()) {
    return &(it->second);
  } else {
    return nullptr;
  }
}

bool SceneGraph::findNearestSceneGraphNodeInLayer(
    const NodePosition& query_point,
    const LayerId& layer_id,
    std::vector<SceneGraphNode>* nearest_scene_graph_node,
    const size_t& n_nearest_neighbors) {
  CHECK_NOTNULL(nearest_scene_graph_node);
  SceneGraphLayer scene_graph_layer;
  getLayerSafe(layer_id, &scene_graph_layer);
  return scene_graph_layer.findNearestSceneGraphNode(query_point,
                                                     nearest_scene_graph_node,
                                                     n_nearest_neighbors);
}

}  // namespace kimera
