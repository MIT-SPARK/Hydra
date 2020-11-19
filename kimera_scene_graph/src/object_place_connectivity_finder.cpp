#include "kimera_scene_graph/object_place_connectivity_finder.h"

#include <glog/logging.h>

#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/scene_graph.h"
#include "kimera_scene_graph/scene_graph_edge.h"
#include "kimera_scene_graph/scene_graph_layer.h"
#include "kimera_scene_graph/scene_graph_node.h"

namespace kimera {

void ObjectPlaceConnectivityFinder::findObjectPlaceConnectivity(SceneGraph* scene_graph) {
  CHECK_NOTNULL(scene_graph);

  SceneGraphLayer objects_layer(LayerId::kObjectsLayerId);
  scene_graph->getLayerSafe(LayerId::kObjectsLayerId, &objects_layer);

  SceneGraphLayer places_layer(LayerId::kPlacesLayerId);
  scene_graph->getLayerSafe(LayerId::kPlacesLayerId, &places_layer);

  // Iterate over the nodes of objects layer
  for (const std::pair<NodeId, SceneGraphNode>& object :
       objects_layer.getNodeIdMap()) {
    const NodeId& object_id = object.first;
    const SceneGraphNode& object_node = object.second;
    if (object_node.hasParent()) {
      // This node already has a parent! This should not happen if we are
      // calling this for the first time unless someone has added a parent
      // already...
      LOG(WARNING) << "Object with a place already! \n" << object_node.print();
      CHECK(object_node.parent_edge_.start_layer_id_ ==
            LayerId::kPlacesLayerId);
      continue;
    }
    VLOG(5) << "Finding place of object with id: " << object_id;

    std::vector<SceneGraphNode> nearest_place;
    places_layer.findNearestSceneGraphNode(object_node.attributes_.position_,
                                           &nearest_place);
    if (nearest_place.size() == 0) {
      LOG(ERROR) << "No place found for object with id: " << object_id;
      continue;
    }

    // Found the nearest place to this id
    const NodeId& place_id = nearest_place.at(0).node_id_;
    linkObjectToPlace(object_id, place_id, scene_graph);
  }
}

void ObjectPlaceConnectivityFinder::linkObjectToPlace(const NodeId& object_id,
                                                      const NodeId& place_id,
                                                      SceneGraph* scene_graph) {
  CHECK_NOTNULL(scene_graph);
  SceneGraphEdge scene_graph_edge;
  scene_graph_edge.start_node_id_ = place_id;
  scene_graph_edge.start_layer_id_ = LayerId::kPlacesLayerId;
  scene_graph_edge.end_node_id_ = object_id;
  scene_graph_edge.end_layer_id_ = LayerId::kObjectsLayerId;
  scene_graph->addEdge(&scene_graph_edge);
  VLOG(5) << "Added place to object edge: " << scene_graph_edge.print();
}

}  // namespace kimera
