#pragma once
#include <kimera_dsg/scene_graph.h>

namespace kimera {

void findRoomConnectivity(SceneGraph* scene_graph);

void findPlacesRoomConnectivity(SceneGraph* scene_graph, float esdf_truncation_distance);

void findObjectPlaceConnectivity(SceneGraph* scene_graph);

}  // namespace kimera
