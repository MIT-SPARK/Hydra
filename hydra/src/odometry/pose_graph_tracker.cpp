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
#include "hydra/odometry/pose_graph_tracker.h"

#include <glog/logging.h>

#include "hydra/common/dsg_types.h"
#include "hydra/common/robot_prefix_config.h"

namespace hydra {

std::vector<NodeId> PoseGraphPacket::addToGraph(DynamicSceneGraph& graph,
                                                std::optional<int> robot_id) const {
  std::vector<NodeId> new_nodes;

  for (const auto& pose_graph : pose_graphs) {
    for (const auto& node : pose_graph.nodes) {
      if (robot_id && node.robot_id != robot_id.value()) {
        VLOG(1) << "Dropping node for other robot: " << node;
        continue;
      }

      const auto node_prefix = RobotPrefixConfig(node.robot_id);
      const NodeSymbol node_id(node_prefix.key, node.key);
      if (graph.hasNode(node_id)) {
        continue;
      }

      const std::chrono::nanoseconds stamp(node.stamp_ns);
      const Eigen::Vector3d pos = node.pose.translation();
      const Eigen::Quaterniond rot(node.pose.linear());
      VLOG(5) << "Adding agent " << node_id.str() << " @ " << stamp.count() << " [ns]";

      auto attrs = std::make_unique<AgentNodeAttributes>(stamp, rot, pos, node_id);
      const auto key = graph.getLayerKey(DsgLayers::AGENTS);
      if (!key) {
        LOG(ERROR) << "No layer named '" << DsgLayers::AGENTS << "' in graph!";
        continue;
      }

      if (!graph.emplaceNode(key->layer, node_id, std::move(attrs), node_prefix.key)) {
        VLOG(1) << "Failed to add node @ " << stamp.count() << "[ns]";
        continue;
      }

      new_nodes.push_back(node_id);
    }
  }

  // TODO(nathan) technically we could do a single loop, but this ensures
  // that we get *most* edges if something external messages up
  for (const auto& pose_graph : pose_graphs) {
    for (const auto& edge : pose_graph.edges) {
      if (robot_id &&
          (edge.robot_from != robot_id.value() || edge.robot_to != robot_id.value())) {
        VLOG(1) << "Dropping edge for other robot: " << edge;
        continue;
      }

      const auto from_prefix = RobotPrefixConfig(edge.robot_from);
      const auto to_prefix = RobotPrefixConfig(edge.robot_to);
      const NodeSymbol from_id(from_prefix.key, edge.key_from);
      const NodeSymbol to_id(to_prefix.key, edge.key_to);
      // TODO(nathan) save actual info once we add attributes to spark_dsg or somewhere
      // else
      graph.insertEdge(from_id, to_id);
    }
  }

  return new_nodes;
}

void PoseGraphPacket::updateFrom(const PoseGraphPacket& other) {
  timestamp_ns = other.timestamp_ns;
  pose_graphs.insert(
      pose_graphs.end(), other.pose_graphs.begin(), other.pose_graphs.end());
  // TODO(nathan) this is technically bad, but we'll get to it
  external_priors = other.external_priors;
}

}  // namespace hydra
