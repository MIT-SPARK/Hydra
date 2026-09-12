#include "hydra/frontend/mesh_update_info.h"

#include "hydra/utils/pgmo_mesh_traits.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

kimera_pgmo::MeshDelta::Ptr updateMesh(kimera_pgmo::DeltaCompression& compression,
                                       const ActiveWindowOutput& input,
                                       spark_dsg::Mesh& mesh,
                                       MeshUpdateInfo& info) {
  auto& offsets = info.offsets;
  info.blocks.clear();
  info.archived_blocks = input.archived;
  timing::ScopedTimer timer(
      "frontend/mesh_archive", input.timestamp_ns, true, 1, false);
  const spatial_hash::IndexSet archived(input.archived.begin(), input.archived.end());
  compression.archiveBlocks(
      [&](const auto& index, const auto&) { return archived.count(index); });

  timer.reset("frontend/mesh_compression");
  kimera_pgmo::HashedIndexMapping mapping;
  const auto& layer = input.map().getMeshLayer();
  auto delta = compression.update(BlockMeshIter(layer), input.timestamp_ns, &mapping);
  timer.reset("frontend/mesh_update");
  delta->updateMesh(mesh, offsets);
  timer.stop();
  timing::ScopedTimer mapping_timer("object/mesh_mapping", input.timestamp_ns);
  for (const auto& block : layer) {
    const auto source = layer.getBlockPtr(block.index);
    auto& entry = info.blocks[source.get()];
    entry.block = source;
    entry.vertices.assign(block.numVertices(), std::nullopt);
    const auto it = mapping.find(block.index);
    if (it != mapping.end()) {
      for (const auto& [local, target] : it->second) {
        entry.vertices.at(local) = offsets.toGlobalVertex(target);
      }
    }
  }
  return delta;
}

}  // namespace hydra
