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
#include "hydra/eval/graph_utilities.h"

#include <glog/logging.h>

namespace hydra::eval {

DynamicSceneGraph::Ptr mergeGraphs(const std::vector<DynamicSceneGraph::Ptr>& graphs) {
  DynamicSceneGraph::Ptr to_return(new DynamicSceneGraph());

  size_t num_nodes_expected = 0;
  size_t num_edges_expected = 0;

  std::map<LayerId, size_t> layer_counts;
  layer_counts[DsgLayers::OBJECTS] = 0;
  layer_counts[DsgLayers::PLACES] = 0;
  layer_counts[DsgLayers::ROOMS] = 0;
  layer_counts[DsgLayers::BUILDINGS] = 0;

  for (const auto& graph_ptr : graphs) {
    const auto& graph = *graph_ptr;
    num_nodes_expected += graph.numStaticNodes();
    num_edges_expected += graph.numStaticEdges();

    std::map<NodeId, NodeId> node_id_map;
    for (const auto& id_layer_pair : graph.layers()) {
      const auto layer = id_layer_pair.first;
      auto& curr_count = layer_counts.at(layer);

      for (const auto& id_node_pair : id_layer_pair.second->nodes()) {
        NodeSymbol prev_id(id_node_pair.first);
        NodeSymbol new_id(prev_id.category(), curr_count);
        ++curr_count;

        node_id_map[prev_id] = new_id;
        to_return->emplaceNode(
            layer, new_id, id_node_pair.second->attributes().clone());
      }
    }

    // intralayer edges
    for (const auto& id_layer_pair : graph.layers()) {
      for (const auto& id_edge_pair : id_layer_pair.second->edges()) {
        const auto& edge = id_edge_pair.second;
        const auto source_id = node_id_map.at(edge.source);
        const auto target_id = node_id_map.at(edge.target);
        to_return->insertEdge(source_id, target_id, edge.info->clone());
      }
    }

    // interlayer edges
    for (const auto& id_edge_pair : graph.interlayer_edges()) {
      const auto& edge = id_edge_pair.second;
      const auto source_id = node_id_map.at(edge.source);
      const auto target_id = node_id_map.at(edge.target);
      to_return->insertEdge(source_id, target_id, edge.info->clone());
    }
  }

  CHECK_EQ(to_return->numStaticNodes(), num_nodes_expected);
  CHECK_EQ(to_return->numStaticEdges(), num_edges_expected);
  return to_return;
}

}  // namespace hydra::eval
