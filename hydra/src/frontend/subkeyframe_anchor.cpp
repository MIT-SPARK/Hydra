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
#include "hydra/frontend/subkeyframe_anchor.h"

#include <chrono>
#include <limits>

namespace hydra {

using spark_dsg::AgentNodeAttributes;
using spark_dsg::NodeId;
using spark_dsg::SceneGraph;

AnchorWindow::AnchorWindow(uint64_t window_ns) : window_ns_(window_ns) {}

void AnchorWindow::update(const SceneGraph& graph,
                          const std::vector<NodeId>& new_nodes) {
  for (const auto node_id : new_nodes) {
    const auto node = graph.findNode(node_id);
    const auto attrs = node ? node->tryAttributes<AgentNodeAttributes>() : nullptr;
    if (!attrs) {
      continue;
    }

    const auto stamp = static_cast<uint64_t>(attrs->timestamp.count());
    if (!anchors_.empty() && stamp < anchors_.back().second) {
      continue;  // anchors are kept in time order
    }

    anchors_.emplace_back(node_id, stamp);
  }

  if (anchors_.empty()) {
    return;
  }

  const auto newest = anchors_.back().second;
  while (anchors_.front().second + window_ns_ < newest) {
    anchors_.pop_front();
  }
}

std::vector<AnchorCandidate> AnchorWindow::candidates(const SceneGraph& graph) const {
  std::vector<AnchorCandidate> result;
  result.reserve(anchors_.size());
  for (const auto& [node_id, stamp] : anchors_) {
    const auto node = graph.findNode(node_id);
    const auto attrs = node ? node->tryAttributes<AgentNodeAttributes>() : nullptr;
    if (!attrs) {
      continue;
    }

    // the pose is read from the graph so that it reflects the latest agent pose
    Eigen::Isometry3d world_T_anchor = Eigen::Isometry3d::Identity();
    world_T_anchor.translation() = attrs->position;
    world_T_anchor.linear() = attrs->world_R_body.toRotationMatrix();
    result.push_back({node_id, stamp, world_T_anchor});
  }

  return result;
}

std::optional<size_t> selectNearestAnchor(const std::vector<AnchorCandidate>& anchors,
                                          uint64_t subframe_ts_ns,
                                          const Eigen::Vector3d& subframe_position,
                                          double max_dist_m) {
  if (anchors.empty()) {
    return std::nullopt;
  }

  // temporally-nearest anchor (ties resolve to the lowest index)
  size_t best = 0;
  uint64_t best_dt = std::numeric_limits<uint64_t>::max();
  for (size_t i = 0; i < anchors.size(); ++i) {
    const auto anchor_ns = anchors[i].timestamp_ns;
    const uint64_t dt = anchor_ns > subframe_ts_ns ? anchor_ns - subframe_ts_ns
                                                   : subframe_ts_ns - anchor_ns;
    if (dt < best_dt) {
      best_dt = dt;
      best = i;
    }
  }

  const double dist =
      (anchors[best].world_T_anchor.translation() - subframe_position).norm();
  if (dist > max_dist_m) {
    return std::nullopt;
  }

  return best;
}

Eigen::Isometry3d computeRelativeTransform(const Eigen::Isometry3d& world_T_anchor,
                                           const Eigen::Isometry3d& world_T_subframe) {
  return world_T_anchor.inverse() * world_T_subframe;
}

std::unique_ptr<spark_dsg::SubKeyframeNodeAttributes> buildSubKeyframeAttrs(
    spark_dsg::NodeId anchor_id,
    const Eigen::Isometry3d& world_T_anchor,
    const Eigen::Isometry3d& world_T_subframe,
    uint64_t timestamp_ns,
    const std::string& image_folder) {
  auto attrs = std::make_unique<spark_dsg::SubKeyframeNodeAttributes>();
  attrs->anchor_node_id = anchor_id;
  const Eigen::Isometry3d rel =
      computeRelativeTransform(world_T_anchor, world_T_subframe);
  attrs->anchor_t_subframe = rel.translation();
  attrs->anchor_R_subframe = Eigen::Quaterniond(rel.rotation());
  attrs->position = world_T_subframe.translation();
  attrs->image_folder = image_folder;
  attrs->timestamp = std::chrono::nanoseconds(timestamp_ns);
  return attrs;
}

}  // namespace hydra
