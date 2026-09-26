#pragma once

#include <config_utilities/factory.h>
#include <hydra/common/message_queue.h>

#include <Eigen/Dense>
#include <atomic>
#include <filesystem>
#include <memory>
#include <mutex>

#include "hydra_multi/common/dsg_types.h"
#include "hydra_multi/common/types.h"
#include "hydra_multi/operators/deformation_graph_operator.h"
#include "hydra_multi/operators/dynamic_scene_graph_operator.h"
#include "hydra_multi/operators/mesh_operator.h"
#include "hydra_multi/operators/pose_graph_operator.h"

namespace hydra_multi {

struct UnitInterfaceState {
  using Ptr = std::shared_ptr<UnitInterfaceState>;
  UnitInterfaceState();

  void initDsg(DynamicSceneGraph::Ptr dsg);
  void save(const std::filesystem::path& log_dir) const;

  // Fields
  pose_graph_tools::PoseGraph::Ptr mesh_graph_;
  pose_graph_tools::PoseGraph::Ptr pose_graph_;
  MeshData::Ptr mesh_data_;
  DynamicSceneGraph::Ptr dsg_;
  gtsam::Pose3 world_T_robot = gtsam::Pose3(Eigen::MatrixXd::Identity(4, 4));

  // Operators
  DeformationGraphOperator::Ptr mesh_graph_operator_;
  PoseGraphOperator::Ptr pose_graph_operator_;
  MeshOperator::Ptr mesh_operator_;
  DynamicSceneGraphOperator::Ptr dsg_operator_;

  // States
  mutable std::mutex mutex;
  std::atomic<bool> updated;
  std::atomic<bool> rebased;
  std::atomic<Timestamp> stamp;
};

}  // namespace hydra_multi
