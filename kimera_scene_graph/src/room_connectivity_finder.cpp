#include "kimera_scene_graph/room_connectivity_finder.h"

#include <glog/logging.h>

#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/scene_graph.h"
#include "kimera_scene_graph/scene_graph_edge.h"
#include "kimera_scene_graph/scene_graph_layer.h"
#include "kimera_scene_graph/scene_graph_node.h"

namespace kimera {

void RoomConnectivityFinder::findRoomConnectivity(SceneGraph* scene_graph) {
  CHECK_NOTNULL(scene_graph);
  SceneGraphLayer places_layer(LayerId::kPlacesLayerId);
  scene_graph->getLayerSafe(LayerId::kPlacesLayerId, &places_layer);
  for (const std::pair<EdgeId, SceneGraphEdge>& edge :
       places_layer.getEdgeIdMap()) {
    const SceneGraphNode& start_node =
        places_layer.getNode(edge.second.start_node_id_);
    const SceneGraphNode& end_node =
        places_layer.getNode(edge.second.end_node_id_);

    if (!start_node.hasParent()) {
      // This node does not belong to any room
      // This can happen if the place node has not been segmented
      LOG(WARNING) << "Place without room: " << start_node.print();
      continue;
    }
    CHECK(start_node.parent_edge_.start_layer_id_ == LayerId::kRoomsLayerId);
    NodeId room_id_start_node = start_node.parent_edge_.start_node_id_;

    if (!end_node.hasParent()) {
      // This node does not belong to any room
      // This can happen if the place node has not been segmented
      LOG(WARNING) << "Place without room: " << end_node.print();
      continue;
    }
    CHECK(end_node.parent_edge_.start_layer_id_ == LayerId::kRoomsLayerId);
    NodeId room_id_end_node = end_node.parent_edge_.start_node_id_;

    if (room_id_start_node != room_id_end_node) {
      SceneGraphEdge scene_graph_edge;
      scene_graph_edge.start_layer_id_ = LayerId::kRoomsLayerId;
      scene_graph_edge.start_node_id_ = room_id_start_node;
      scene_graph_edge.end_layer_id_ = LayerId::kRoomsLayerId;
      scene_graph_edge.end_node_id_ = room_id_end_node;
      scene_graph->addEdge(&scene_graph_edge);
      LOG(INFO) << "Added room to room edge: " << scene_graph_edge.print();
    } else {
      VLOG(10) << "This pair of places are in the same room.";
    }
  }
}

}  // namespace kimera
