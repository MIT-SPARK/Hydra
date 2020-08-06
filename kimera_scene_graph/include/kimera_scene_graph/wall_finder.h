#pragma once

#include <string>

#include <voxblox/mesh/mesh.h>

#include <kimera_semantics/common.h>

#include "kimera_scene_graph/scene_graph.h"

namespace kimera {

class EnclosingWallFinder {
 public:
  EnclosingWallFinder(const std::string& world_frame);
  ~EnclosingWallFinder() = default;

 public:
  /**
   * @brief findWalls
   * @param walls_mesh[in/out] Mesh of the walls without room labels [in] and
   * with room labels [out].
   * @param[out]
   */
  void findWalls(const vxb::Mesh& walls_mesh,
                 const SceneGraph& scene_graph,
                 vxb::Mesh* segmented_walls_mesh);

 private:
  std::string world_frame_;
};

}  // namespace kimera
