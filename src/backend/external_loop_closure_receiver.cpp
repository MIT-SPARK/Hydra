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
#include "hydra/backend/external_loop_closure_receiver.h"

#include <config_utilities/config.h>
#include <glog/logging.h>
#include <kimera_pgmo/utils/common_functions.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>

namespace hydra {

using LookupResult = ExternalLoopClosureReceiver::LookupResult;
using spark_dsg::AgentNodeAttributes;
using spark_dsg::DynamicSceneGraph;
using spark_dsg::NodeSymbol;
using spark_dsg::SceneGraphLayer;
using spark_dsg::SceneGraphNode;

namespace {

inline std::chrono::nanoseconds getAgentTimestamp(const SceneGraphNode& node) {
  return node.attributes<AgentNodeAttributes>().timestamp;
}

inline double convertToSeconds(std::chrono::nanoseconds time_ns) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(time_ns).count();
}

inline std::chrono::nanoseconds getLastStamp(const SceneGraphLayer& layer) {
  auto last = std::max_element(
      layer.nodes().begin(), layer.nodes().end(), [](const auto& lhs, const auto& rhs) {
        return getAgentTimestamp(*lhs.second) < getAgentTimestamp(*rhs.second);
      });
  return last == layer.nodes().end() ? std::chrono::nanoseconds(0)
                                     : getAgentTimestamp(*last->second);
}

}  // namespace

void declare_config(ExternalLoopClosureReceiver::Config& config) {
  using namespace config;
  name("ExternalLoopClosureReceiver::Config");
  field(config.layer, "layer");
  field(config.max_time_difference, "max_time_difference", "s");
  check(config.max_time_difference, GE, 0.0, "max_time_difference");
}

ExternalLoopClosureReceiver::ExternalLoopClosureReceiver(const Config& config,
                                                         Queue* const queue)
    : config(config), input_queue_(queue) {
  LOG_IF(WARNING, !input_queue_) << "External loop closures disabled!";
}

LookupResult ExternalLoopClosureReceiver::findClosest(const DynamicSceneGraph& graph,
                                                      uint64_t stamp_ns,
                                                      int robot_id,
                                                      double max_diff_s) const {
  const auto prefix = kimera_pgmo::GetRobotPrefix(robot_id);
  const auto layer_id = graph.getLayerKey(config.layer)->layer;
  const auto layer = graph.findLayer(layer_id, prefix);
  if (!layer) {
    LOG(WARNING) << "Missing robot " << robot_id << " for external loop closure";
    return {};
  }

  const auto stamp = std::chrono::nanoseconds(stamp_ns);
  const auto last_stamp = getLastStamp(*layer);
  if (stamp > last_stamp) {
    // avoid clearing loop closure before best candidate node can be determined
    return {};
  }

  auto closest =
      std::min_element(layer->nodes().begin(),
                       layer->nodes().end(),
                       [stamp](const auto& lhs, const auto& rhs) {
                         const auto diff_lhs = getAgentTimestamp(*lhs.second) - stamp;
                         const auto diff_rhs = getAgentTimestamp(*rhs.second) - stamp;
                         return std::abs(diff_lhs.count()) < std::abs(diff_rhs.count());
                       });
  if (closest == layer->nodes().end()) {
    VLOG(1) << "No nodes exist for robot " << robot_id << "' when looking up timestamp "
            << stamp_ns << " [ns]";
    return {};
  }

  const NodeSymbol best_id(closest->second->id);
  const auto best_stamp = getAgentTimestamp(*closest->second);
  const auto diff_s = convertToSeconds(best_stamp - stamp);
  VLOG(5) << "Found node " << best_id.str() << " with difference of " << diff_s
          << " [s] for timestamp " << stamp_ns << " [ns]";

  // avoid associating to nodes that are too far away in time
  if (max_diff_s && std::abs(diff_s) >= max_diff_s) {
    LOG(WARNING) << "Nearest node " << best_id.str()
                 << " has too large of a time difference: " << diff_s
                 << " [s] >= " << max_diff_s << " [s]";
    return {LookupResult::Status::INVALID};
  }

  return {LookupResult::Status::VALID, best_id};
}

void ExternalLoopClosureReceiver::update(const DynamicSceneGraph& graph,
                                         const Callback& callback) {
  if (!input_queue_) {
    return;
  }

  while (!input_queue_->empty()) {
    const auto pose_graph = input_queue_->pop();
    for (const auto& edge : pose_graph.edges) {
      loop_closures_.push_back(edge);
    }
  }

  auto iter = loop_closures_.begin();
  while (iter != loop_closures_.end()) {
    const auto& edge = *iter;
    const auto max_diff_s = config.max_time_difference;

    // NOTE(hlim) the pose graph edge IDs contain the loop closure timestamps, not the
    // keyframe IDs
    const auto from_node =
        findClosest(graph, edge.key_from, edge.robot_from, max_diff_s);
    const auto to_node = findClosest(graph, edge.key_to, edge.robot_to, max_diff_s);
    if (from_node.status == LookupResult::Status::INVALID ||
        to_node.status == LookupResult::Status::INVALID) {
      LOG(WARNING) << "Dropped external loop closure: " << edge;
      iter = loop_closures_.erase(iter);
      continue;
    }

    // delay adding loop closure until best nodes can be determined
    if (from_node.status == LookupResult::Status::UNKNOWN ||
        to_node.status == LookupResult::Status::UNKNOWN) {
      ++iter;
      continue;
    }

    // to_id, from_id, to_T_from
    callback(to_node.id, from_node.id, gtsam::Pose3(edge.pose.matrix()));
    iter = loop_closures_.erase(iter);
  }
}

}  // namespace hydra
