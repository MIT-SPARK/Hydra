#pragma once

namespace kimera {

class SceneGraph;

class BuildingFinder {
 public:
  BuildingFinder() = default;
  virtual ~BuildingFinder() = default;

  /**
   * @brief findBuildings
   */
  void findBuildings(SceneGraph* scene_graph);
};

}  // namespace kimera
