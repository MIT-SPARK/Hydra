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

#include <spark_dsg/node_attributes.h>
#include <spark_dsg/scene_graph.h>
#include <spark_dsg/scene_graph_types.h>

#include <Eigen/Geometry>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace hydra {

//! Agent node that a sub-keyframe can be attached to
struct AnchorCandidate {
  spark_dsg::NodeId id;
  uint64_t timestamp_ns;
  Eigen::Isometry3d world_T_anchor;
};

/**
 * @brief Recent agent nodes that sub-keyframes can be anchored to.
 *
 * Sub-keyframe requests are always close to the current time, so only the agent nodes
 * created within a time window of the newest one are kept, which bounds the cost of
 * anchoring independently of the trajectory length.
 */
class AnchorWindow {
 public:
  //! @param window_ns Anchors older than the newest anchor by more than this are
  //! dropped
  explicit AnchorWindow(uint64_t window_ns);

  /**
   * @brief Add new agent nodes and drop anchors outside the window.
   * @param graph Graph containing the nodes
   * @param new_nodes New agent nodes in creation order (nodes without agent attributes
   * are skipped)
   */
  void update(const spark_dsg::SceneGraph& graph,
              const std::vector<spark_dsg::NodeId>& new_nodes);

  //! @brief Anchors in the window with the current poses of their nodes in the graph
  std::vector<AnchorCandidate> candidates(const spark_dsg::SceneGraph& graph) const;

  //! @brief Number of anchors in the window
  size_t size() const { return anchors_.size(); }

 private:
  const uint64_t window_ns_;
  //! Node ids and timestamps of the anchors in creation order
  std::deque<std::pair<spark_dsg::NodeId, uint64_t>> anchors_;
};

/**
 * @brief Select the anchor for a sub-keyframe.
 *
 * Selects the temporally-nearest anchor (which stays within the same pass through an
 * area, even when revisiting it) and accepts it only if it is within max_dist_m of
 * the sub-keyframe, as the error of the rigid anchor_T_subframe transform grows with
 * that distance. Never falls back to a spatially closer but temporally further anchor.
 * @returns Index of the selected anchor if any
 */
std::optional<size_t> selectNearestAnchor(const std::vector<AnchorCandidate>& anchors,
                                          uint64_t subframe_ts_ns,
                                          const Eigen::Vector3d& subframe_position,
                                          double max_dist_m);

//! @brief Compute anchor_T_subframe
Eigen::Isometry3d computeRelativeTransform(const Eigen::Isometry3d& world_T_anchor,
                                           const Eigen::Isometry3d& world_T_subframe);

/**
 * @brief Build the attributes of a sub-keyframe node.
 *
 * The transform relative to the anchor is the source of truth; the world position is
 * seeded from world_T_subframe and refreshed from the anchor by the backend.
 */
std::unique_ptr<spark_dsg::SubKeyframeNodeAttributes> buildSubKeyframeAttrs(
    spark_dsg::NodeId anchor_id,
    const Eigen::Isometry3d& world_T_anchor,
    const Eigen::Isometry3d& world_T_subframe,
    uint64_t timestamp_ns,
    const std::string& image_folder);

}  // namespace hydra
