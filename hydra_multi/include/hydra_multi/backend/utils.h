#pragma once

#include <gtsam/nonlinear/Values.h>
#include <hydra/utils/minimum_spanning_tree.h>
#include <kimera_pgmo/deformation_graph.h>

#include <map>

#include "hydra_multi/common/types.h"

namespace hydra_multi {

bool updateToGlobalFrame(const kimera_pgmo::DeformationGraph& dgraph,
                         const gtsam::Values& global_values,
                         const RobotId robot_id,
                         const gtsam::Pose3& local_T_pose,
                         gtsam::Pose3& global_T_pose);

size_t findClosestStampIdx(const Timestamps& stamps, const Timestamp query_stamp);

std::optional<Timestamp> getTimeNs(const DynamicSceneGraph& graph, gtsam::Symbol key);

void reindexMeshConnections(DynamicSceneGraph& graph,
                            const NodeIdRobotMap& node_to_robot,
                            const RobotIndexMap& mesh_offsets);

}  // namespace hydra_multi
