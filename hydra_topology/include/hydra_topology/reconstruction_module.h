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
#include "hydra_topology/reconstruction_config.h"
#include "hydra_topology/reconstruction_output.h"

#include <hydra_topology/configs.h>
#include <hydra_utils/display_utils.h>
#include <hydra_utils/input_queue.h>
#include <hydra_utils/robot_prefix_config.h>
#include <kimera_semantics/semantic_voxel.h>
#include <Eigen/Geometry>

#include <memory>

namespace hydra {

struct ReconstructionInput {
  using Ptr = std::shared_ptr<ReconstructionInput>;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  size_t timestamp_ns;
  pose_graph_tools::PoseGraph::ConstPtr pose_graph;
  Eigen::Vector3d world_t_body;
  Eigen::Quaterniond world_R_body;
  std::unique_ptr<voxblox::Pointcloud> pointcloud;
  std::unique_ptr<voxblox::Colors> pointcloud_colors;
};

// TODO(nathan) consider moving to shared module state
class ReconstructionModule {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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

  // public for external use
  void spinOnce(const ReconstructionInput& input);

  void addOutputCallback(const OutputCallback& callback);

 protected:
  voxblox::BlockIndexList update(const voxblox::Transformation& T_G_C,
                                 const voxblox::Pointcloud& pointcloud,
                                 const voxblox::Colors& colors,
                                 size_t timestamp_ns,
                                 bool full_update);

  void showStats() const;

  void addMeshToOutput(const voxblox::BlockIndexList& archived_blocks,
                       ReconstructionOutput& output,
                       size_t timestamp_ns);

  void addPlacesToOutput(ReconstructionOutput& output, size_t timestamp_ns);

  void fillPoseGraphNode(pose_graph_tools::PoseGraphNode& node,
                         size_t stamp,
                         const Eigen::Affine3d& pose,
                         size_t index) const;

  void makePoseGraphEdge(pose_graph_tools::PoseGraph& graph) const;

  pose_graph_tools::PoseGraph makePoseGraph(size_t timestamp_ns,
                                            const Eigen::Affine3d& curr_pose);

  voxblox::Transformation getCameraPose(const ReconstructionInput& msg) const;

 protected:
  RobotPrefixConfig prefix_;
  ReconstructionConfig config_;

  std::atomic<bool> should_shutdown_{false};
  ReconstructionInputQueue::Ptr queue_;
  std::unique_ptr<std::thread> spin_thread_;

  ReconstructionOutput::Ptr output_;
  OutputQueue::Ptr output_queue_;

  voxblox::Layer<voxblox::TsdfVoxel>::Ptr tsdf_;
  voxblox::Layer<kimera::SemanticVoxel>::Ptr semantics_;
  voxblox::Layer<topology::GvdVoxel>::Ptr gvd_;
  voxblox::MeshLayer::Ptr mesh_;

  std::unique_ptr<voxblox::TsdfIntegratorBase> tsdf_integrator_;
  std::unique_ptr<topology::GvdIntegrator> gvd_integrator_;

  size_t prev_time_;
  size_t num_poses_received_;
  Eigen::Affine3d prev_pose_;

  std::list<OutputCallback> output_callbacks_;
};

}  // namespace hydra
