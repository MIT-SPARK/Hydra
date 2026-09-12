#pragma once

#include <kimera_pgmo/compression/delta_compression.h>

#include "hydra/active_window/active_window_output.h"

namespace hydra {

struct MeshUpdateInfo {
  struct BlockMapping {
    MeshBlock::ConstPtr block;
    //! Local vertex to global compressed mesh index; empty entries were removed.
    std::vector<std::optional<size_t>> vertices;
  };

  kimera_pgmo::MeshOffsetInfo offsets;
  BlockIndices archived_blocks;
  //! Mappings for block instances received in the latest input packet.
  std::map<const MeshBlock*, BlockMapping> blocks;
};

kimera_pgmo::MeshDelta::Ptr updateMesh(kimera_pgmo::DeltaCompression& compression,
                                       const ActiveWindowOutput& input,
                                       spark_dsg::Mesh& mesh,
                                       MeshUpdateInfo& info);

}  // namespace hydra
