#pragma once
#include <spark_dsg/node_attributes.h>

namespace kimera_pgmo {
struct MeshOffsetInfo;
}  // namespace kimera_pgmo

namespace hydra {

void remap2dPlaceIndices(spark_dsg::Place2dNodeAttributes& attrs,
                         const kimera_pgmo::MeshOffsetInfo& offsets);

}  // namespace hydra
