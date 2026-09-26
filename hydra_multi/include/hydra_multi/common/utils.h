#pragma once

#include <map>

#include "hydra_multi/common/dsg_types.h"
#include "hydra_multi/common/types.h"

namespace hydra_multi {

void combineMeshes(const std::map<RobotId, Mesh::Ptr>& id_mesh,
                   Mesh::Ptr combined_mesh,
                   RobotIndexMap& vertex_offset);

}  // namespace hydra_multi
