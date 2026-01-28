#include "hydra/places/2d_places/index_remapping.h"

#include <kimera_pgmo/mesh_offset_info.h>
#include <spark_dsg/node_attributes.h>

namespace hydra {
namespace {

inline void remapPlace2dConnections(spark_dsg::Place2dNodeAttributes& attrs,
                                    const kimera_pgmo::MeshOffsetInfo& offsets) {
  kimera_pgmo::MeshOffsetInfo::RemapStats info;
  offsets.remapVertexIndices(attrs.mesh_connections, &info);
  attrs.min_mesh_index = info.min_index;
  attrs.max_mesh_index = info.max_index;
  attrs.has_active_mesh_indices = !info.all_archived;
}

}  // namespace

void remap2dPlaceIndices(spark_dsg::Place2dNodeAttributes& attrs,
                         const kimera_pgmo::MeshOffsetInfo& offsets) {
  remapPlace2dConnections(attrs, offsets);

  auto& indices = attrs.boundary_connections;
  kimera_pgmo::MeshOffsetInfo::RemapStats info;
  indices = offsets.remapVertexIndices(indices, &info);

  const auto prev_boundary = attrs.boundary;
  attrs.boundary.clear();
  attrs.boundary.reserve(prev_boundary.size());
  for (size_t i = 0; i < prev_boundary.size(); ++i) {
    if (info.deleted_indices.count(i)) {
      continue;
    }

    attrs.boundary.push_back(prev_boundary[i]);
  }
}

}  // namespace hydra
