#pragma once
#include <kimera_dsg/dynamic_scene_graph.h>
#include <kimera_dsg/node_attributes.h>

namespace kimera {

using NodeColor = SemanticNodeAttributes::ColorVector;

void findBuildings(DynamicSceneGraph* scene_graph, const NodeColor& color);

}  // namespace kimera
