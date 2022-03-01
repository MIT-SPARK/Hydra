#pragma once
#include "kimera_batch_dsg/pcl_types.h"

#include <kimera_dsg/dynamic_scene_graph.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/search/kdtree.h>
#include <map>

namespace kimera {

struct RoomHull {
  pcl::CropHull<ColorPoint>::Ptr cropper;
  pcl::KdTreeFLANN<ColorPoint>::Ptr hull_kdtree;
};

using RoomHullMap = std::map<NodeId, RoomHull>;

void findRoomConnectivity(DynamicSceneGraph* scene_graph);

void findPlacesRoomConnectivity(DynamicSceneGraph* scene_graph,
                                const RoomHullMap& room_hulls,
                                float esdf_truncation_distance);

void findObjectPlaceConnectivity(DynamicSceneGraph* scene_graph);

}  // namespace kimera
