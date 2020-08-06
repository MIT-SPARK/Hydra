#pragma once

namespace kimera {

class SceneGraph;

class RoomConnectivityFinder {
 public:
  RoomConnectivityFinder() = default;
  ~RoomConnectivityFinder() = default;

 public:
  /// Loop over all edges, if two vertices of different rooms are
  /// connected together, add such connection to the scene-graph
  void findRoomConnectivity(SceneGraph* scene_graph);
};

}  // namespace kimera
