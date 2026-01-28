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
#include "hydra/places/2d_places/place_reallocation.h"

namespace hydra::utils {

using spark_dsg::NodeId;
using spark_dsg::Place2dNodeAttributes;
using spark_dsg::SceneGraphLayer;

namespace {

using Filter = std::function<bool(size_t)>;

std::vector<size_t> filterPoints(Place2dNodeAttributes& attrs, const Filter& filter) {
  std::vector<size_t> filtered;
  auto iter = attrs.mesh_connections.begin();
  while (iter != attrs.mesh_connections.end()) {
    if (!filter(*iter)) {
      ++iter;
      continue;
    }

    filtered.push_back(*iter);
    iter = attrs.mesh_connections.erase(iter);
  }

  return filtered;
}

}  // namespace

void reallocateMeshPoints(const spark_dsg::Mesh& mesh,
                          Place2dNodeAttributes& attrs1,
                          Place2dNodeAttributes& attrs2) {
  const Eigen::Vector2f delta =
      (attrs2.position.head(2) - attrs1.position.head(2)).cast<float>();
  const Eigen::Vector2f p_mid = attrs1.position.head(2).cast<float>() + delta / 2.0f;
  const auto p2_new = filterPoints(attrs1, [&](size_t idx) {
    // positive distance -> closer to attrs2
    return (mesh.points.at(idx).head(2) - p_mid).dot(delta) > 0.0f;
  });

  const auto p1_new = filterPoints(attrs2, [&](size_t idx) {
    // negative distance -> closer to attrs1
    return (mesh.points.at(idx).head(2) - p_mid).dot(delta) <= 0.0f;
  });

  attrs1.mesh_connections.insert(
      attrs1.mesh_connections.end(), p1_new.begin(), p1_new.end());
  attrs2.mesh_connections.insert(
      attrs2.mesh_connections.end(), p2_new.begin(), p2_new.end());

  // Say there are active mesh indices if either involved node has them.
  attrs1.has_active_mesh_indices |= attrs2.has_active_mesh_indices;
  attrs2.has_active_mesh_indices |= attrs1.has_active_mesh_indices;
}

inline Place2dNodeAttributes* getAttrs(const SceneGraphLayer& layer,
                                       spark_dsg::NodeId node_id) {
  auto node = layer.findNode(node_id);
  if (!node) {
    return nullptr;
  }

  auto attrs = node->tryAttributes<Place2dNodeAttributes>();
  return (!attrs || attrs->is_active) ? nullptr : attrs;
}

void propagateReallocation(const spark_dsg::Mesh& mesh,
                           const SceneGraphLayer& layer,
                           const std::set<NodeId>& changed_nodes,
                           std::set<NodeId>& seen_nodes) {
  for (const auto node_id : changed_nodes) {
    const auto node = layer.findNode(node_id);
    if (!node) {
      continue;
    }

    auto attrs = node->tryAttributes<Place2dNodeAttributes>();
    if (!attrs || attrs->is_active) {
      continue;
    }

    seen_nodes.insert(node_id);
    for (const auto sibling_id : node->siblings()) {
      const auto& sibling = layer.getNode(sibling_id);
      auto sibling_attrs = sibling.tryAttributes<Place2dNodeAttributes>();
      if (!sibling_attrs || sibling_attrs->is_active) {
        continue;
      }

      seen_nodes.insert(sibling_id);
      reallocateMeshPoints(mesh, *attrs, *sibling_attrs);
    }
  }
}

}  // namespace hydra::utils
