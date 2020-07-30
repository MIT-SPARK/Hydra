#pragma once

#include <glog/logging.h>

#include <ros/ros.h>

#include <voxblox_skeleton/skeleton.h>

#include <kimera_semantics/common.h>

#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/scene_node.h"

namespace kimera {

class RoomConnectivityFinder {
 public:
  RoomConnectivityFinder() = default;
  ~RoomConnectivityFinder() = default;

 public:
  /// Loop over all edges, if two vertices of different rooms are
  /// connected together, add such connection to the scene-graph
  void findRoomConnectivity(SceneGraph* scene_graph) {
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

 private:
  // bool findParentOfNode(const SceneGraphNode& scene_graph_node,
  //                      NodeId* parent_node_id) {
  //  CHECK_NOTNULL(parent_node_id);
  //  CHECK_EQ(scene_graph_node.layer_id_, LayerId::kPlacesLayerId)
  //      << "We can't find a room id for a node that is not a place.";
  //  EdgeIdMap inter_layer_edges;
  //  scene_graph_node.getInterIntraLayerEdges(&inter_layer_edges, nullptr);
  //  for (const std::pair<EdgeId, SceneGraphEdge>& inter_layer_edge :
  //       scene_graph_node.ch) {
  //    if (inter_layer_edge.second.start_layer_id_ == LayerId::kRoomsLayerId) {
  //      CHECK_NE(inter_layer_edge.second.start_node_id_,
  //               scene_graph_node.node_id_)
  //          << "The parent should not have the same node id.";
  //      *parent_node_id = inter_layer_edge.second.start_node_id_;
  //      return true;
  //    } else if (inter_layer_edge.second.end_layer_id_ ==
  //               LayerId::kRoomsLayerId) {
  //      CHECK_NE(inter_layer_edge.second.end_node_id_,
  //               scene_graph_node.node_id_)
  //          << "This node should be in kPlacesLayer, not in kRoomsLayer";
  //      *parent_node_id = inter_layer_edge.second.end_node_id_;
  //      return true;
  //    } else {
  //      // If any of the other layers is neither the rooms or the objects,
  //      // we have a problem
  //      CHECK(
  //          inter_layer_edge.second.end_layer_id_ == LayerId::kPlacesLayerId
  //          ||
  //          inter_layer_edge.second.end_layer_id_ ==
  //          LayerId::kObjectsLayerId);
  //      CHECK(inter_layer_edge.second.start_layer_id_ ==
  //                LayerId::kPlacesLayerId ||
  //            inter_layer_edge.second.start_layer_id_ ==
  //                LayerId::kObjectsLayerId);
  //    }
  //  }
  //  return false;
  //}
};

}  // namespace kimera
