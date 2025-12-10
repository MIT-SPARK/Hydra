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
#include "hydra/backend/backend_utilities.h"

#include <glog/logging.h>
#include <kimera_pgmo/mesh_delta.h>

#include <limits>

namespace hydra::utils {
namespace {

void updatePlace2dMesh(Place2dNodeAttributes& attrs,
                       const kimera_pgmo::MeshDelta& delta,
                       const size_t archived) {
  attrs.pcl_min_index = std::numeric_limits<size_t>::max();
  attrs.pcl_max_index = 0;

  attrs.pcl_mesh_connections = delta.remapIndices(attrs.pcl_mesh_connections, archived);
  for (const auto idx : attrs.pcl_mesh_connections) {
    attrs.pcl_min_index = std::min(attrs.pcl_min_index, idx);
    attrs.pcl_max_index = std::max(attrs.pcl_max_index, idx);
  }

  if (attrs.pcl_max_index < archived) {
    attrs.has_active_mesh_indices = false;
  }
}

void updatePlace2dBoundary(Place2dNodeAttributes& attrs,
                           const kimera_pgmo::MeshDelta& delta,
                           const size_t archived) {
  const auto prev_boundary = attrs.boundary;
  const auto prev_connections = attrs.pcl_boundary_connections;
  attrs.boundary.clear();
  attrs.pcl_boundary_connections.clear();

  const auto& remap = delta.prev_to_curr();
  for (size_t i = 0; i < prev_boundary.size(); ++i) {
    auto global_idx = prev_connections.at(i);
    if (global_idx < archived) {
      attrs.boundary.push_back(prev_boundary.at(i));
      attrs.pcl_boundary_connections.push_back(global_idx);
      continue;
    }

    const auto prev_local = global_idx - archived;
    auto new_idx = remap.find(prev_local);
    if (new_idx == remap.end()) {
      continue;
    }

    // TODO(nathan) this is wrong
    attrs.boundary.push_back(prev_boundary.at(i));
    attrs.pcl_boundary_connections.push_back(new_idx->second + archived);
  }
}

}  // namespace

std::optional<uint64_t> getTimeNs(const DynamicSceneGraph& graph, gtsam::Symbol key) {
  NodeSymbol node(key.chr(), key.index());
  if (!graph.hasNode(node)) {
    LOG(ERROR) << "Missing node << " << node.str() << "when logging loop closure";
    return std::nullopt;
  }

  return graph.getNode(node).attributes<AgentNodeAttributes>().timestamp.count();
}

void updatePlaces2d(SharedDsgInfo::Ptr dsg,
                    kimera_pgmo::MeshDelta& mesh_update,
                    size_t num_archived_vertices) {
  if (!dsg->graph->hasLayer(DsgLayers::MESH_PLACES)) {
    return;
  }

  for (auto& [node_id, node] : dsg->graph->getLayer(DsgLayers::MESH_PLACES).nodes()) {
    auto attrs = node->tryAttributes<spark_dsg::Place2dNodeAttributes>();
    if (!attrs || !attrs->has_active_mesh_indices) {
      continue;
    }

    updatePlace2dMesh(*attrs, mesh_update, num_archived_vertices);
    updatePlace2dBoundary(*attrs, mesh_update, num_archived_vertices);
  }
}

}  // namespace hydra::utils
