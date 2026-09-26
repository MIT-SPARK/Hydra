#include <glog/logging.h>

#include "hydra_multi/common/types.h"

namespace hydra_multi {

void combineMeshes(const std::map<RobotId, Mesh::Ptr>& id_mesh,
                   Mesh::Ptr combined_mesh,
                   RobotIndexMap& vertex_offset) {
  vertex_offset.clear();
  combined_mesh->clear();

  for (const auto& [robot_id, robot_mesh] : id_mesh) {
    vertex_offset[robot_id] = combined_mesh->numVertices();
    if (!robot_mesh) {
      continue;
    }

    CHECK(combined_mesh->append(*robot_mesh));
  }
}

}  // namespace hydra_multi
