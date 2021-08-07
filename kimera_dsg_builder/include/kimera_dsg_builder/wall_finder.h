#pragma once
#include "kimera_dsg_builder/pcl_conversion.h"
#include <kimera_dsg/scene_graph.h>

namespace kimera {

pcl::PolygonMesh::Ptr findWalls(const SubMesh& walls_mesh,
                                const SceneGraph& scene_graph);

}  // namespace kimera
