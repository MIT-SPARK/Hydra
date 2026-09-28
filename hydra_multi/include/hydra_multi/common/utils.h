#pragma once

#include <map>

#include "hydra_multi/common/types.h"

namespace hydra_multi {

void combineMeshes(const std::map<RobotId, spark_dsg::Mesh::Ptr>& id_mesh,
                   spark_dsg::Mesh::Ptr combined_mesh,
                   RobotIndexMap& vertex_offset);

}  // namespace hydra_multi
