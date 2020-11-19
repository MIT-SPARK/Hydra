#pragma once

#include "kimera_scene_graph/common.h"

namespace kimera {

class SceneGraph;

class ObjectPlaceConnectivityFinder {
 public:
  ObjectPlaceConnectivityFinder();
  ~ObjectPlaceConnectivityFinder() = default;

 public:
  // Finds the nearest place to an object, mind that this function only
  // considers euclidean distance, and not traversable path distance. Hence
  // an object close to a wall might be linked to a place on the other side
  // of the wall.
  void findObjectPlaceConnectivity(SceneGraph* scene_graph);

 protected:
  void linkObjectToPlace(const NodeId& object_id,
                         const NodeId& room_id,
                         SceneGraph* scene_graph);
};

}  // namespace kimera
