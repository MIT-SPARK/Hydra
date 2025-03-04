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

#include <gtsam/geometry/Pose3.h>
#include <pose_graph_tools/pose_graph.h>
#include <spark_dsg/dynamic_scene_graph.h>

#include <list>
#include <optional>

#include "hydra/common/message_queue.h"

namespace hydra {

class ExternalLoopClosureReceiver {
 public:
  using Queue = MessageQueue<pose_graph_tools::PoseGraph>;
  using Callback = std::function<void(
      spark_dsg::NodeId to, spark_dsg::NodeId from, const gtsam::Pose3 to_T_from)>;

  struct LookupResult {
    enum class Status {
      UNKNOWN,
      INVALID,
      VALID,
    } const status = Status::UNKNOWN;
    const spark_dsg::NodeId id = 0;
  };

  struct Config {
    //! Layer to use for the pose graph
    std::string layer = spark_dsg::DsgLayers::AGENTS;
    //! Maximum difference in seconds between a loop closure timestamp and the nearest
    //! agent pose. 0 disables checking time differences.
    double max_time_difference = 1.0;
  } const config;

  ExternalLoopClosureReceiver(const Config& config, Queue* const queue);
  void update(const spark_dsg::DynamicSceneGraph& graph, const Callback& callback);
  LookupResult findClosest(const spark_dsg::DynamicSceneGraph& graph,
                           uint64_t timestamp_ns,
                           int robot_id,
                           double max_diff_s = 0.0) const;

 protected:
  Queue* const input_queue_;
  std::list<pose_graph_tools::PoseGraphEdge> loop_closures_;
};

void declare_config(ExternalLoopClosureReceiver::Config& config);

}  // namespace hydra
