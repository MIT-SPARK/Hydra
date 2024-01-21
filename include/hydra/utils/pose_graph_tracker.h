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
#include <pose_graph_tools_msgs/PoseGraph.h>

#include <Eigen/Geometry>
#include <memory>

namespace spark_dsg {
// forward declare to avoid include
class DynamicSceneGraph;
}  // namespace spark_dsg

namespace hydra {

struct ReconstructionOutput;
struct BackendInput;
struct RobotPrefixConfig;

class PoseGraphTracker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::unique_ptr<PoseGraphTracker>;

  struct Config {
    bool make_pose_graph = false;
    double min_separation_m = 0.0;
  };

  explicit PoseGraphTracker(const Config& config);

  ~PoseGraphTracker() = default;

  std::optional<size_t> update(const ReconstructionOutput& msg);

  void fillPoseGraphs(BackendInput& output,
                      const std::vector<uint64_t>& new_nodes) const;

  std::vector<uint64_t> addAgentNodes(spark_dsg::DynamicSceneGraph& graph,
                                      const RobotPrefixConfig& prefix) const;

 protected:
  const Config config_;
  pose_graph_tools_msgs::PoseGraph graph_;

  uint64_t prev_time_;
  size_t num_poses_received_;
  Eigen::Isometry3d prev_pose_;
};

void declare_config(PoseGraphTracker::Config& config);

}  // namespace hydra
