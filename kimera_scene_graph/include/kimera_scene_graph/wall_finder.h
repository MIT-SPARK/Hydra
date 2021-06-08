#pragma once

#include <string>

#include <kimera_dsg/scene_graph.h>
#include <voxblox/mesh/mesh.h>

namespace kimera {

class EnclosingWallFinder {
 public:
  EnclosingWallFinder(const std::string& world_frame);

  ~EnclosingWallFinder() = default;

  /**
   * @brief findWalls
   * @param walls_mesh[in/out] Mesh of the walls without room labels [in] and
   * with room labels [out].
   * @param[out]
   */
  void findWalls(const voxblox::Mesh& walls_mesh,
                 const SceneGraph& scene_graph,
                 voxblox::Mesh* segmented_walls_mesh);

 private:
  std::string world_frame_;
};

}  // namespace kimera
