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
#include "hydra_topology/graph_extractor_types.h"

namespace hydra {
namespace topology {

VoxelGraphInfo::VoxelGraphInfo() : is_node(false), is_split_node(false) {}

VoxelGraphInfo::VoxelGraphInfo(NodeId id, bool is_from_split)
    : id(id), is_node(true), is_split_node(is_from_split) {}

EdgeInfo::EdgeInfo(size_t id, NodeId source) : id(id), source(source) {}

EdgeSplitSeed::EdgeSplitSeed(const GlobalIndex& index,
                             double distance_to_edge,
                             size_t edge_id)
    : index(index), distance_to_edge(distance_to_edge), edge_id(edge_id) {}

bool operator<(const EdgeSplitSeed& lhs, const EdgeSplitSeed& rhs) {
  return lhs.distance_to_edge < rhs.distance_to_edge;
}

std::ostream& operator<<(std::ostream& out, const VoxelGraphInfo& info) {
  if (info.is_node) {
    out << "node " << NodeSymbol(info.id).getLabel();
  } else {
    out << "edge " << info.edge_id;
  }

  return out;
}

std::ostream& operator<<(std::ostream& out, const EdgeInfo& info) {
  out << "source: " << NodeSymbol(info.id).getLabel() << ", id: " << info.id
      << ", size: " << info.indices.size()
      << ", connections: " << info.connections.size()
      << ", node connections: " << info.node_connections.size();
  return out;
}

}  // namespace topology
}  // namespace hydra
