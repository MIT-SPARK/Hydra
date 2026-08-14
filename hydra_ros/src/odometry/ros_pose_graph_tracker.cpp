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

#include "hydra_ros/odometry/ros_pose_graph_tracker.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <ianvs/node_handle.h>

namespace hydra {

using pose_graph_tools::PoseGraph;

void declare_config(RosPoseGraphTracker::Config& config) {
  using namespace config;
  name("RosPoseGraphTracker");
  field(config.ns, "ns");
  field(config.queue_size, "queue_size");
}

RosPoseGraphTracker::RosPoseGraphTracker(const Config& config)
    : config(config::checkValid(config)) {
  using pose_graph_tools::PoseGraphTypeAdapter;
  auto nh = ianvs::NodeHandle::this_node(config.ns);
  odom_sub_ = nh.create_subscription<PoseGraphTypeAdapter>(
      "pose_graph", config.queue_size, &RosPoseGraphTracker::odomCallback, this);
  prior_sub_ =
      nh.create_subscription<PoseGraphTypeAdapter>("agent_node_measurements",
                                                   config.queue_size,
                                                   &RosPoseGraphTracker::priorCallback,
                                                   this);
}

// Return pose graphs and priors received from Kimera since the last call
PoseGraphPacket RosPoseGraphTracker::update(uint64_t, const Eigen::Isometry3d&) {
  std::unique_lock<std::mutex> lock(mutex_);

  PoseGraphPacket packet;
  packet.pose_graphs = std::move(pose_graphs_);
  // NOTE(lschmid): Move assignments should leave vectors empty but could be
  // implementation dependent so we play it safe.
  pose_graphs_.clear();
  packet.external_priors = external_priors_;
  external_priors_.reset();
  return packet;
}

void RosPoseGraphTracker::odomCallback(const PoseGraph& msg) {
  if (msg.nodes.empty()) {
    LOG(WARNING) << "[RosPoseGraphTracker] Received empty pose graph, skipping!";
    return;
  }

  std::unique_lock<std::mutex> lock(mutex_);
  pose_graphs_.push_back(msg);
}

void RosPoseGraphTracker::priorCallback(const PoseGraph& msg) {
  std::unique_lock<std::mutex> lock(mutex_);
  external_priors_ = std::make_shared<pose_graph_tools::PoseGraph>(msg);
}

}  // namespace hydra
