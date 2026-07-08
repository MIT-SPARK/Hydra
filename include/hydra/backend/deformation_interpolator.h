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

#include <memory>
#include <optional>

#include "hydra/backend/update_functions.h"

namespace hydra {

/**
 * @brief Write the deformed (optimized) version of the odometric source attributes
 * into the destination attributes.
 *
 * Only spatial fields are written (position, bounding box, object orientation), so
 * merged-only state on the destination survives. Everything is derived from the
 * odometric source, never from the destination, so repeated application with the
 * same transform is idempotent.
 */
void applyNodeDeformation(const Eigen::Isometry3d& transform,
                          const NodeAttributes& src,
                          NodeAttributes& dst);

struct NodeCache {
  struct Entry {
    NodeId id;
    uint64_t timestamp;
    //! Odometric position (the unmerged graph is odometric by invariant); doubles
    //! as the pgmo vertex for control-point matching.
    Eigen::Vector3f pos;
    //! Most recent odometric->optimized transform applied to this node, if any.
    //! Lets merge hooks bring attributes rebuilt from odometric constituents into
    //! the optimized frame.
    std::optional<Eigen::Isometry3d> last_transform;
  };

  Entry* add(NodeId node, const NodeAttributes& attrs);

  /**
   * @brief Apply the node's most recent deformation transform to the given
   * attributes (via NodeAttributes::transform).
   * @returns true if a transform was recorded and applied.
   */
  bool applyLastTransform(NodeId node, NodeAttributes& attrs) const;

  std::map<NodeId, Entry> nodes;
};

/**
 * @brief Tool to apply the deformation of the mesh control points to other nodes in the
 * DSG.
 */
class DeformationInterpolator {
 public:
  struct Config {
    //! Number of control points to use for deformation interpolation
    size_t num_control_points = 4;
    //! Timestamp tolerance for control points [s]
    double control_point_tolerance_s = 10.0;
  } const config;

  explicit DeformationInterpolator(const Config& config);

  /**
   * @brief Interpolate the node poses based on the deformation graph, using the
   * temporally closest control points as reference.
   *
   * Reads odometric values from the unmerged scene graph and writes the deformed
   * (optimized) values into the private DSG only; the unmerged graph is never
   * modified.
   *
   * @param unmerged Unmerged (odometric) scene graph to read from.
   * @param dsg Private DSG to update.
   * @param info Update information containing deformation graph.
   * @param view View on the unmerged scene graph selecting all nodes to update.
   */
  void interpolate(const DynamicSceneGraph& unmerged,
                   DynamicSceneGraph& dsg,
                   const UpdateInfo::ConstPtr& info,
                   const LayerView& view) const;

  //! @copydoc DeformationInterpolator::interpolate
  void interpolateNodePositions(const DynamicSceneGraph& unmerged,
                                DynamicSceneGraph& dsg,
                                const UpdateInfo::ConstPtr& info,
                                const LayerView& view) const {
    interpolate(unmerged, dsg, info, view);
  }

  /**
   * @brief Apply the node's most recent deformation transform to the given
   * attributes (e.g. to bring merge-hook output rebuilt from odometric
   * constituents into the optimized frame).
   * @returns true if a transform was recorded and applied.
   */
  bool applyLastTransform(NodeId node, NodeAttributes& attrs) const {
    return cache_.applyLastTransform(node, attrs);
  }

 private:
  mutable NodeCache cache_;
};

void declare_config(DeformationInterpolator::Config& config);

}  // namespace hydra
