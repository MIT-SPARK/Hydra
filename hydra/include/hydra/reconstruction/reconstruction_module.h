/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#pragma once
#include <kimera_semantics/semantic_voxel.h>

#include <Eigen/Geometry>
#include <memory>

#include "hydra/common/input_queue.h"
#include "hydra/common/robot_prefix_config.h"
#include "hydra/places/gvd_voxel.h"
#include "hydra/places/vertex_voxel.h"
#include "hydra/reconstruction/configs.h"
#include "hydra/reconstruction/reconstruction_config.h"
#include "hydra/reconstruction/reconstruction_output.h"

namespace hydra {

namespace places {
// forward declare to avoid include
class GvdIntegrator;
}  // namespace places

class VoxelAwareMeshIntegrator;

struct ReconstructionInput {
  using Ptr = std::shared_ptr<ReconstructionInput>;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  uint64_t timestamp_ns;
  std::list<pose_graph_tools::PoseGraph::ConstPtr> pose_graphs;
  Eigen::Vector3d world_t_body;
  Eigen::Quaterniond world_R_body;
  std::unique_ptr<voxblox::Pointcloud> pointcloud;
  std::unique_ptr<voxblox::Colors> pointcloud_colors;
};

class ReconstructionModule {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using PositionMatrix = Eigen::Matrix<double, 3, Eigen::Dynamic>;
  using ReconstructionInputQueue = InputQueue<ReconstructionInput::Ptr>;
  using OutputQueue = InputQueue<ReconstructionOutput::Ptr>;
  using OutputCallback =
      std::function<void(const ReconstructionModule&, const ReconstructionOutput&)>;

  ReconstructionModule(const RobotPrefixConfig& prefix,
                       const ReconstructionConfig& config,
                       const OutputQueue::Ptr& output_queue);

  virtual ~ReconstructionModule();

  void start();

  void stop();

  void save(const std::string& output_path);

  void spin();

  bool spinOnce();

  void updateGvd();

  // public for external use
  bool spinOnce(const ReconstructionInput& input);

  void addOutputCallback(const OutputCallback& callback);

  // takes in a 3xN matrix
  std::vector<bool> inFreespace(const PositionMatrix& positions,
                                double freespace_distance_m) const;

 protected:
  void update(const ReconstructionInput& msg, bool full_update);

  void updateGvdSpin();

  void showStats() const;

  void addMeshToOutput(ReconstructionOutput& output,
                       const voxblox::BlockIndexList& archived_blocks);

  void addPlacesToOutput(ReconstructionOutput& output);

  void fillPoseGraphNode(pose_graph_tools::PoseGraphNode& node,
                         uint64_t stamp,
                         const Eigen::Affine3d& pose,
                         size_t index) const;

  void makePoseGraphEdge(pose_graph_tools::PoseGraph& graph) const;

  pose_graph_tools::PoseGraph makePoseGraph(uint64_t timestamp_ns,
                                            const Eigen::Affine3d& curr_pose);

  voxblox::Transformation getCameraPose(const ReconstructionInput& msg) const;

  voxblox::BlockIndexList findBlocksToArchive(const voxblox::Point& center) const;

 protected:
  mutable std::mutex tsdf_mutex_;
  mutable std::mutex gvd_mutex_;

  RobotPrefixConfig prefix_;
  ReconstructionConfig config_;

  std::atomic<bool> should_shutdown_{false};
  ReconstructionInputQueue::Ptr queue_;
  OutputQueue gvd_queue_;
  std::unique_ptr<std::thread> spin_thread_;
  std::unique_ptr<std::thread> gvd_thread_;

  std::list<pose_graph_tools::PoseGraph::ConstPtr> pose_graphs_;
  OutputQueue::Ptr output_queue_;

  voxblox::Layer<voxblox::TsdfVoxel>::Ptr tsdf_;
  voxblox::Layer<kimera::SemanticVoxel>::Ptr semantics_;
  voxblox::Layer<places::GvdVoxel>::Ptr gvd_;
  voxblox::Layer<places::VertexVoxel>::Ptr vertices_;
  voxblox::MeshLayer::Ptr mesh_;

  std::unique_ptr<voxblox::TsdfIntegratorBase> tsdf_integrator_;
  std::unique_ptr<VoxelAwareMeshIntegrator> mesh_integrator_;
  std::unique_ptr<places::GvdIntegrator> gvd_integrator_;

  uint64_t prev_time_;
  size_t num_poses_received_;
  Eigen::Affine3d prev_pose_;

  std::list<OutputCallback> output_callbacks_;
};

}  // namespace hydra
