#include "hydra_multi/backend/utils.h"

#include <glog/logging.h>
#include <hydra/places/2d_places/index_remapping.h>
#include <kimera_pgmo/utils/common_functions.h>

namespace hydra_multi {
namespace {

inline void updateIndices(ObjectNodeAttributes& attrs,
                          const kimera_pgmo::MeshOffsetInfo& offsets) {
  offsets.remapVertexIndices(attrs.mesh_connections);
}

inline void updateIndices(Place2dNodeAttributes& attrs,
                          const kimera_pgmo::MeshOffsetInfo& offsets) {
  hydra::remap2dPlaceIndices(attrs, offsets);
}

template <typename Attrs>
void remapLayerConnections(const SceneGraphLayer& layer,
                           const NodeIdRobotMap& node_to_robot,
                           const RobotIndexMap& mesh_offsets) {
  for (const auto& [node_id, node] : layer.nodes()) {
    auto attrs = node->template tryAttributes<Attrs>();
    if (!attrs) {
      continue;
    }

    // all robots need a current offset
    const auto robot_id = node_to_robot.at(node_id);
    auto iter = mesh_offsets.find(robot_id);
    if (iter == mesh_offsets.end()) {
      LOG(ERROR) << "Robot " << robot_id << " has no mesh offsets!";
      continue;
    }

    updateIndices(*attrs, iter->second);
  }
}

}  // namespace

bool updateToGlobalFrame(const kimera_pgmo::DeformationGraph& dgraph,
                         const gtsam::Values& global_values,
                         const RobotId robot_id,
                         const gtsam::Pose3& local_T_pose,
                         gtsam::Pose3& global_T_pose) {
  const char robot_prefix = kimera_pgmo::GetRobotPrefix(robot_id);
  if (robot_prefix == '\0') {
    LOG(WARNING) << "Robot ID (" << robot_id
                 << ") not assigned valid prefix in updateToGlobalFrame.";
  }

  const gtsam::Symbol first_node(robot_prefix, 0);
  if (!global_values.exists(first_node)) {
    global_T_pose = local_T_pose;
    return false;
  }

  gtsam::Pose3 global_T_robot = global_values.at<gtsam::Pose3>(first_node);
  gtsam::Pose3 local_T_robot = dgraph.getInitialPose(robot_prefix, 0);
  global_T_pose = global_T_robot.compose(local_T_robot.between(local_T_pose));
  return true;
}

size_t findClosestStampIdx(const Timestamps& stamps, const Timestamp query_stamp) {
  auto it = std::lower_bound(stamps.begin(), stamps.end(), query_stamp);
  if (it == stamps.end()) {
    return stamps.size() - 1;
  }
  if (query_stamp - *(it - 1) < *it - query_stamp) {
    it--;
  }
  return it - stamps.begin();
}

std::optional<Timestamp> getTimeNs(const DynamicSceneGraph& graph, gtsam::Symbol key) {
  NodeSymbol node(key.chr(), key.index());
  if (!graph.hasNode(node)) {
    LOG(ERROR) << "Missing node << " << node << "when getting time.";
    return std::nullopt;
  }
  return graph.getNode(node).attributes<AgentNodeAttributes>().timestamp.count();
}

void reindexMeshConnections(DynamicSceneGraph& graph,
                            const NodeIdRobotMap& node_to_robot,
                            const RobotIndexMap& mesh_offsets) {
  const auto objects = graph.findLayer(DsgLayers::OBJECTS);
  if (objects) {
    remapLayerConnections<ObjectNodeAttributes>(*objects, node_to_robot, mesh_offsets);
  }

  const auto places = graph.findLayer(DsgLayers::MESH_PLACES);
  if (places) {
    remapLayerConnections<Place2dNodeAttributes>(*places, node_to_robot, mesh_offsets);
  }
}

}  // namespace hydra_multi
