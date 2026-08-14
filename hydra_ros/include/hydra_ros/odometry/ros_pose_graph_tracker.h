/* -----------------------------------------------------------------------------
 * Code in this file was originally adapted from Khronos, see:
 *   @misc{schmid2024khronosunifiedapproachspatiotemporal,
 *         title={Khronos: A Unified Approach for Spatio-Temporal Metric-Semantic
 *                SLAM in Dynamic Environments},
 *         author={Lukas Schmid and Marcus Abate and Yun Chang and Luca Carlone},
 *         year={2024},
 *         eprint={2402.13817},
 *         archivePrefix={arXiv},
 *         primaryClass={cs.RO},
 *         url={https://arxiv.org/abs/2402.13817},
 *  }
 *
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

#include <config_utilities/factory.h>
#include <hydra/odometry/pose_graph_tracker.h>
#include <pose_graph_tools_ros/conversions.h>

#include <mutex>

namespace hydra {

/**
 * @brief Odometry input that subscribes to pose graph
 * messages from Kimera or other sources that follow its output types.
 */
class RosPoseGraphTracker : public PoseGraphTracker {
 public:
  struct Config {
    //! ROS namespace
    std::string ns = "~";
    //! Size of the odometry subscriber queue.
    size_t queue_size = 1000;
  } const config;

  explicit RosPoseGraphTracker(const Config& config);
  virtual ~RosPoseGraphTracker() = default;

  PoseGraphPacket update(uint64_t timestamp,
                         const Eigen::Isometry3d& world_T_body) override;

 protected:
  void odomCallback(const pose_graph_tools::PoseGraph& pose_graph);
  void priorCallback(const pose_graph_tools::PoseGraph& pose_graph);

  pose_graph_tools::PoseGraphSubscription odom_sub_;
  pose_graph_tools::PoseGraphSubscription prior_sub_;

  std::mutex mutex_;
  std::vector<pose_graph_tools::PoseGraph> pose_graphs_;
  pose_graph_tools::PoseGraph::ConstPtr external_priors_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<PoseGraphTracker,
                                     RosPoseGraphTracker,
                                     RosPoseGraphTracker::Config>("RosPoseGraphs");
};

void declare_config(RosPoseGraphTracker::Config& config);

}  // namespace hydra
