#include "kimera_scene_graph/building_finder.h"

#include <string>

#include "kimera_scene_graph/common.h"

namespace kimera {

void BuildingFinder::findBuildings(SceneGraph* scene_graph) {
  CHECK_NOTNULL(scene_graph);
  SceneGraphNode building_instance;
  building_instance.attributes_.semantic_label_ = kBuildingSemanticLabel;
  building_instance.attributes_.color_ = kBuildingColor;
  // We only have one building now.
  building_instance.attributes_.name_ =
      std::to_string(kBuildingSemanticLabel) +
      building_instance.attributes_.name_;
  building_instance.node_id_ = 1u;
  building_instance.layer_id_ = LayerId::kBuildingsLayerId;
  SceneGraphLayer room_layer(LayerId::kRoomsLayerId);
  CHECK(scene_graph->getLayerSafe(LayerId::kRoomsLayerId, &room_layer))
      << "Room layer is not present in the scene-graph...";
  //  TODO(Toni): assumes we only have one building, ideally get the sub-graph
  // spanned by the inter-class room traversability edges.
  Centroid building_centroid;
  ColorPointCloud::Ptr building_rooms_cloud(new ColorPointCloud);
  for (const auto& room_it : room_layer.getNodeIdMap()) {
    const SceneGraphNode& room_node = room_it.second;
    const NodePosition& room_position = room_node.attributes_.position_;
    // Calculate the building's centroid: centroid of rooms' centroids.
    building_centroid.add(room_position);

    // Link this room to the building node pcl wise
    ColorPoint room_point;
    room_point.x = room_position.x;
    room_point.y = room_position.y;
    room_point.z = room_position.z;
    room_point.r = room_node.attributes_.color_[0];
    room_point.g = room_node.attributes_.color_[1];
    room_point.b = room_node.attributes_.color_[2];
    building_rooms_cloud->points.push_back(room_point);
  }
  building_centroid.get(building_instance.attributes_.position_);
  // I guess this could be calculated after the fact, once the scene graph is
  // built
  building_instance.attributes_.pcl_ = building_rooms_cloud;
  scene_graph->addSceneNode(building_instance);

  // Adding building-to-room edges: assumes all rooms are in the building.
  for (const auto& room_it : room_layer.getNodeIdMap()) {
    const SceneGraphNode& room_node = room_it.second;
    CHECK(room_node.layer_id_ == LayerId::kRoomsLayerId);
    SceneGraphEdge scene_graph_edge;
    scene_graph_edge.start_layer_id_ = building_instance.layer_id_;
    scene_graph_edge.start_node_id_ = building_instance.node_id_;
    scene_graph_edge.end_layer_id_ = room_node.layer_id_;
    scene_graph_edge.end_node_id_ = room_node.node_id_;
    scene_graph->addEdge(&scene_graph_edge);
  }
}

}  // namespace kimera
