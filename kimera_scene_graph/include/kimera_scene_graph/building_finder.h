#pragma once
#include <kimera_dsg/scene_graph.h>
#include <kimera_dsg/node_attributes.h>

namespace kimera {

using NodeColor = SemanticNodeAttributes::ColorVector;

void findBuildings(SceneGraph* scene_graph, const NodeColor& color);

}  // namespace kimera
