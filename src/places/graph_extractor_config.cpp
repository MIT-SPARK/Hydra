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
#include "hydra/places/graph_extractor_config.h"

#include <config_utilities/config.h>

namespace hydra::places {

void declare_config(OverlapEdgeConfig& conf) {
  using namespace config;
  name("OverlapEdgeConfig");
  field(conf.num_neighbors_to_check, "num_neighbors_to_check");
  field(conf.min_clearance_m, "min_clearance_m");
}

void declare_config(FreespaceEdgeConfig& conf) {
  using namespace config;
  name("FreespaceEdgeConfig");
  field(conf.max_length_m, "max_length_m");
  field(conf.num_nodes_to_check, "num_nodes_to_check");
  field(conf.num_neighbors_to_find, "num_neighbors_to_find");
  field(conf.min_clearance_m, "min_clearance_m");
}

void declare_config(GraphExtractorConfig& conf) {
  using namespace config;
  name("GraphExtractorConfig");
  field(conf.node_merge_distance_m, "node_merge_distance_m");
  field(conf.add_overlap_edges, "add_overlap_edges");
  if (conf.add_overlap_edges) {
    field(conf.overlap_edges, "overlap_edges");
  }

  field(conf.add_freespace_edges, "add_freespace_edges");
  if (conf.add_freespace_edges) {
    field(conf.freespace_edges, "freespace_edges");
  }
}

void declare_config(CompressionExtractorConfig& conf) {
  using namespace config;
  name("CompressionExtractorConfig");
  base<GraphExtractorConfig>(conf);
  field(conf.compression_distance_m, "compression_distance_m");
  field(conf.validate_graph, "validate_graph");
  field(conf.min_node_distance_m, "min_node_distance_m");
  field(conf.min_edge_distance_m, "min_edge_distance_m");
  field(conf.merge_nearby_nodes, "merge_new_nodes");
  field(conf.add_heuristic_edges, "add_heuristic_edges");
  field(conf.merge_policy, "merge_policy");
}

void declare_config(FloodfillExtractorConfig& conf) {
  using namespace config;
  name("FloodfillExtractorConfig");
  base<GraphExtractorConfig>(conf);
  field(conf.min_extra_basis, "min_extra_basis");
  field(conf.min_vertex_basis, "min_vertex_basis");
  field(conf.merge_new_nodes, "merge_new_nodes");
  field(conf.edge_splitting_merge_nodes, "edge_splitting_merge_nodes");
  field(conf.max_edge_split_iterations, "max_edge_split_iterations");
  field(conf.max_edge_deviation, "max_edge_deviation");
}

}  // namespace hydra::places
