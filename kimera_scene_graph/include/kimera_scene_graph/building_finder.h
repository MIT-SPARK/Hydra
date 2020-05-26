#pragma once

#include <string>

#include <kimera_semantics/common.h>

#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/scene_node.h"

#include <pcl/common/centroid.h>

namespace kimera {

class BuildingFinder {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  BuildingFinder() = default;
  virtual ~BuildingFinder() = default;

  /**
   * @brief findBuildings
   */
  void findBuildings(const SceneGraph& scene_graph,
                     SceneNode* building_instance);

 private:
};

}  // namespace kimera
