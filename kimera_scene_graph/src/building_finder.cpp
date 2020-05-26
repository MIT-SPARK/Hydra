#include "kimera_scene_graph/building_finder.h"

#include <string>

namespace kimera {

void BuildingFinder::findBuildings(const SceneGraph& scene_graph,
                                   SceneNode* building_instance) {
  CHECK_NOTNULL(building_instance);
  building_instance->attributes_.semantic_label_ = kBuildingSemanticLabel;
  building_instance->attributes_.color_ = kBuildingColor;
  // We only have one building now.
  building_instance->attributes_.instance_id_ = 1u;
  building_instance->id_ =
      std::to_string(kBuildingSemanticLabel) +
      std::to_string(building_instance->attributes_.instance_id_);
  NodeIdMap room_layer;
  scene_graph.getSemanticLayer(kRoomSemanticLabel, &room_layer);
  //  TODO(Toni): assumes we only have one building, ideally get the sub-graph
  // spanned by the inter-class room traversability edges.
  Centroid building_centroid;
  ColorPointCloud::Ptr building_rooms_cloud(new ColorPointCloud);
  building_rooms_cloud->resize(room_layer.size());
  size_t room_idx = 0;
  for (const auto& room : room_layer) {
    const NodePosition& room_position = room.second.attributes_.position_;
    // Calculate the building's centroid: centroid of rooms' centroids.
    building_centroid.add(room_position);

    // Link this room to the building node
    ColorPoint room_point;
    room_point.x = room_position.x;
    room_point.y = room_position.y;
    room_point.z = room_position.z;
    room_point.r = kRoomColor.x();
    room_point.g = kRoomColor.y();
    room_point.b = kRoomColor.z();
    building_rooms_cloud->points.at(room_idx) = room_point;
    ++room_idx;
  }
  building_centroid.get(building_instance->attributes_.position_);
  building_instance->attributes_.pcl_ = building_rooms_cloud;
}

}
