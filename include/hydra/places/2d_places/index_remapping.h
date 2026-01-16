#pragma once
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/scene_graph_layer.h>

namespace kimera_pgmo {
struct MeshOffsetInfo;
}  // namespace kimera_pgmo

namespace hydra {

void remap2dPlaceIndices(spark_dsg::Place2dNodeAttributes& attrs,
                         const kimera_pgmo::MeshOffsetInfo& offsets);

void remap2dPlaceIndices(const spark_dsg::SceneGraphLayer& layer,
                         const kimera_pgmo::MeshOffsetInfo& offsets);

}  // namespace hydra
