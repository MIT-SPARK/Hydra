#pragma once

#include <gtsam/geometry/Pose3.h>
#include <hydra/common/message_queue.h>
#include <hydra/common/robot_prefix_config.h>
#include <hydra/common/shared_dsg_info.h>
#include <hydra/common/shared_module_state.h>
#include <hydra/loop_closure/registration_solution.h>
#include <hydra/utils/data_directory.h>
#include <hydra/utils/pgmo_mesh_traits.h>
#include <kimera_pgmo/mesh_delta.h>
#include <kimera_pgmo/mesh_offset_info.h>
#include <spark_dsg/mesh.h>

#include <map>
#include <memory>
#include <unordered_map>

namespace hydra_multi {

using RobotId = size_t;
using Timestamp = uint64_t;

using RobotPrefixConfig = hydra::RobotPrefixConfig;

using IdIdMap = std::unordered_map<spark_dsg::NodeId, spark_dsg::NodeId>;
using NodeIdRobotMap = std::unordered_map<spark_dsg::NodeId, RobotId>;
using RobotIndexMap = std::map<RobotId, kimera_pgmo::MeshOffsetInfo>;
using RobotIdIdMap = std::unordered_map<RobotId, IdIdMap>;

using Timestamps = std::vector<Timestamp>;

// Mesh
using MeshDelta = kimera_pgmo::MeshDelta;

struct MeshData {
  using Ptr = std::shared_ptr<MeshData>;
  MeshData();

  MeshData::Ptr clone() const;
  Timestamp getLatestStamp() const;
  void transform(const Eigen::Isometry3d& tf);

  spark_dsg::Mesh::Ptr mesh;
  hydra::StampedMeshVertices original;
  kimera_pgmo::MeshOffsetInfo offsets;
};

// Scene Graph
using SceneGraphDelta = std::vector<uint8_t>;

// LCs and Transforms
using LoopClosure = hydra::lcd::RegistrationSolution;
using LoopClosures = std::vector<LoopClosure>;
using Transforms = std::vector<gtsam::Pose3>;
using MultiTransforms = std::map<RobotId, std::map<RobotId, Transforms>>;

using hydra::DataDirectory;

}  // namespace hydra_multi
