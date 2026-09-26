#pragma once

#include <gtsam/geometry/Pose3.h>
#include <hydra/common/message_queue.h>
#include <hydra/common/robot_prefix_config.h>
#include <hydra/common/shared_dsg_info.h>
#include <hydra/common/shared_module_state.h>
#include <hydra/loop_closure/registration_solution.h>
#include <hydra/utils/data_directory.h>
#include <kimera_pgmo/mesh_delta.h>
#include <kimera_pgmo/mesh_offset_info.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map>
#include <memory>
#include <unordered_map>

#include "hydra_multi/common/dsg_types.h"

namespace hydra_multi {

using RobotId = size_t;
using Timestamp = uint64_t;

using RobotPrefixConfig = hydra::RobotPrefixConfig;

using IdIdMap = std::unordered_map<NodeId, NodeId>;
using NodeIdRobotMap = std::unordered_map<NodeId, RobotId>;
using RobotIndexMap = std::map<RobotId, kimera_pgmo::MeshOffsetInfo>;
using RobotIdIdMap = std::unordered_map<RobotId, IdIdMap>;

using Timestamps = std::vector<Timestamp>;

// Mesh
using MeshDelta = kimera_pgmo::MeshDelta;
struct MeshData {
  using Ptr = std::shared_ptr<MeshData>;
  MeshData();

  MeshData::Ptr clone() const;

  void transform(const Eigen::Isometry3d& tf);

  Mesh::Ptr mesh;
  kimera_pgmo::MeshOffsetInfo offsets;
  pcl::PointCloud<pcl::PointXYZ>::Ptr original_vertices;
  std::shared_ptr<Timestamps> vertex_stamps;

  Timestamps getStamps() const;
  Timestamp getLatestStamp() const;
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
