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

#include "hydra/frontend/place_2d_split_logic.h"

namespace hydra::utils {

std::optional<uint64_t> getTimeNs(const DynamicSceneGraph& graph, gtsam::Symbol key) {
  NodeSymbol node(key.chr(), key.index());
  if (!graph.hasNode(node)) {
    LOG(ERROR) << "Missing node << " << node.str() << "when logging loop closure";
    return std::nullopt;
  }

  return graph.getNode(node).attributes<AgentNodeAttributes>().timestamp.count();
}

void updatePlaces2d(SharedDsgInfo::Ptr dsg,
                    const kimera_pgmo::MeshDelta& mesh_update,
                    const kimera_pgmo::MeshOffsetInfo& offsets) {
  if (!dsg->graph->hasLayer(DsgLayers::MESH_PLACES)) {
    return;
  }

  for (auto& [node_id, node] : dsg->graph->getLayer(DsgLayers::MESH_PLACES).nodes()) {
    auto attrs = node->tryAttributes<spark_dsg::Place2dNodeAttributes>();
    if (!attrs || !attrs->has_active_mesh_indices) {
      continue;
    }

    remapPlace2dMesh(*attrs, mesh_update, offsets);
  }
}

}  // namespace hydra::utils
