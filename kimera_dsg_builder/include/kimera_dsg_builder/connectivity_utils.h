#pragma once
#include "kimera_dsg_builder/pcl_types.h"

#include <kimera_dsg/scene_graph.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/search/kdtree.h>
#include <map>

namespace kimera {

struct RoomHull {
  pcl::CropHull<ColorPoint>::Ptr cropper;
  pcl::KdTreeFLANN<ColorPoint>::Ptr hull_kdtree;
};

using RoomHullMap = std::map<NodeId, RoomHull>;

void findRoomConnectivity(SceneGraph* scene_graph);

void findPlacesRoomConnectivity(SceneGraph* scene_graph,
                                const RoomHullMap& room_hulls,
                                float esdf_truncation_distance);

void findObjectPlaceConnectivity(SceneGraph* scene_graph);

}  // namespace kimera
