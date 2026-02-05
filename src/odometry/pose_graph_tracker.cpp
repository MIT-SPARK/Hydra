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

  // assigning semantic labels and timestamps to agent nodes
  // collect all node timestamps for orphan detection
  std::set<uint64_t> node_timestamps;
  for (const auto& pose_graph : pose_graphs) {
    for (const auto& node : pose_graph.nodes) {
      if (robot_id && node.robot_id != robot_id.value()) {
        continue;
      }
      node_timestamps.insert(node.stamp_ns);
    }
  }

  // find orphaned labels,  map to next available node timestamp
  std::map<uint64_t, std::vector<uint32_t>> additional_labels_for_node;

  for (const auto& [timestamp, labels] : semantic_labels_by_timestamp) {
    if (node_timestamps.find(timestamp) == node_timestamps.end()) {
      // labels but no node -> orphaned
      auto next_node_it = node_timestamps.lower_bound(timestamp);
      if (next_node_it != node_timestamps.end()) {
        // assign to next node
        uint64_t next_timestamp = *next_node_it;
        additional_labels_for_node[next_timestamp].insert(
          additional_labels_for_node[next_timestamp].end(),
          labels.begin(),
          labels.end()
        );
        VLOG(5) << "carrying forward " << labels.size() << " orphaned labels from "
                << timestamp << " to " << next_timestamp << " [ns]";
      } else {
        VLOG(5) << "dropping " << labels.size() << " orphaned labels from "
                << timestamp << " [ns] (no subsequent node)";
      }
    }
  }

  // create nodes with both their own and orphaned labels
  for (const auto& pose_graph : pose_graphs) {
    for (const auto& node : pose_graph.nodes) {
      if (robot_id && node.robot_id != robot_id.value()) {
        VLOG(1) << "dropping node for other robot: " << node;
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
      VLOG(5) << "adding agent " << node_id.str() << " @ " << stamp.count() << " [ns]";

      auto attrs = std::make_unique<AgentNodeAttributes>(stamp, rot, pos, node_id);

      // assign semantic labels for this timestamp
      auto labels_iter = semantic_labels_by_timestamp.find(node.stamp_ns);
      if (labels_iter != semantic_labels_by_timestamp.end()) {
        attrs->observed_semantic_labels = labels_iter->second;
        VLOG(5) << "Assigned " << labels_iter->second.size()
                << " semantic labels to agent @ " << node.stamp_ns << " [ns]";
      }

      // assign any orphaned labels carried forward to this node
      auto additional_iter = additional_labels_for_node.find(node.stamp_ns);
      if (additional_iter != additional_labels_for_node.end()) {
        attrs->observed_semantic_labels.insert(
          attrs->observed_semantic_labels.end(),
          additional_iter->second.begin(),
          additional_iter->second.end()
        );
        VLOG(5) << "added " << additional_iter->second.size()
                << " orphaned semantic labels to agent @ " << node.stamp_ns << " [ns]";
      }

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
  // merge semantic labels
  semantic_labels_by_timestamp.insert(
      other.semantic_labels_by_timestamp.begin(),
      other.semantic_labels_by_timestamp.end());
}

}  // namespace hydra
