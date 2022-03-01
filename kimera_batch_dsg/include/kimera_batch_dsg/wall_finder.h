#pragma once
#include "kimera_batch_dsg/pcl_conversion.h"
#include <kimera_dsg/dynamic_scene_graph.h>

namespace kimera {

pcl::PolygonMesh::Ptr findWalls(const SubMesh& walls_mesh,
                                const DynamicSceneGraph& scene_graph);

}  // namespace kimera
