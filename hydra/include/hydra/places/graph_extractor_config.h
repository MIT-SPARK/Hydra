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
#include <cstddef>
#include <cstdint>

namespace hydra {
namespace places {

struct OverlapEdgeConfig {
  size_t num_neighbors_to_check;
  double min_clearance_m;
};

struct FreespaceEdgeConfig {
  double max_length_m;
  size_t num_nodes_to_check;
  size_t num_neighbors_to_find;
  double min_clearance_m;
};

struct CompressionExtractorConfig {
  //! Average resolution of sparse graph
  double compression_distance_m = 0.5;
  //! Validate graph properties
  bool validate_graph = false;
  //! Minimum distance for a GVD node to be considered for compression
  double min_node_distance_m = 0.3;
  //! Minimum distance for a edge to be valid
  double min_edge_distance_m = 0.4;
  //! Distance to consider a merge at
  double node_merge_distance_m = 0.3;
  //! whether to merge nearby neighbors
  bool merge_nearby_nodes = true;
  //! whether or not to add heuristic edges
  bool add_heuristic_edges = false;
};

struct FloodfillExtractorConfig {
  //! Number of basis points for a voxel to be consider for extraction
  uint8_t min_extra_basis = 2;
  //! Number of basis points for a voxel to be automatically labeled a vertex
  uint8_t min_vertex_basis = 3;
  //! Whether or not to merge nodes close together during initial edge extraction
  bool merge_new_nodes = true;
  //! Maximum distance between two nodes to consider a merge
  double node_merge_distance_m = 0.2;
  //! @brief Whether or not to merge nodes close together when splitting edges
  bool edge_splitting_merge_nodes = true;
  //! Number of maximum iterations to run edges splitting (set to 0 to disable)
  size_t max_edge_split_iterations = 5;
  //! Maximum squared voxel distance an edge can be from supporting voxels at any point
  int64_t max_edge_deviation = 4;
};

struct GraphExtractorConfig {
  //! Add edges between nodes that have overlapping free-space regions
  bool add_overlap_edges = true;
  //! configuration for overlap-based edges
  OverlapEdgeConfig overlap_edges;
  //! add edges between disconnected components
  bool add_freespace_edges = true;
  //! configuration for freespace edges
  FreespaceEdgeConfig freespace_edges;
  //! whether or not to use compression-based extraction
  bool use_compression_extractor = false;
  //! Compression-based extraction configuration
  CompressionExtractorConfig compression;
  //! Floodfill-based extraction configuration
  FloodfillExtractorConfig floodfill;
};

template <typename Visitor>
void visit_config(const Visitor& v, OverlapEdgeConfig& config) {
  v.visit("num_neighbors_to_check", config.num_neighbors_to_check);
  v.visit("min_clearance_m", config.min_clearance_m);
}

template <typename Visitor>
void visit_config(const Visitor& v, FreespaceEdgeConfig& config) {
  v.visit("max_length_m", config.max_length_m);
  v.visit("num_nodes_to_check", config.num_nodes_to_check);
  v.visit("num_neighbors_to_find", config.num_neighbors_to_find);
  v.visit("min_clearance_m", config.min_clearance_m);
}

template <typename Visitor>
void visit_config(const Visitor& v, CompressionExtractorConfig& config) {
  v.visit("compression_distance_m", config.compression_distance_m);
  v.visit("validate_graph", config.validate_graph);
  v.visit("min_node_distance_m", config.min_node_distance_m);
  v.visit("min_edge_distance_m", config.min_edge_distance_m);
  v.visit("node_merge_distance_m", config.node_merge_distance_m);
  v.visit("merge_new_nodes", config.merge_nearby_nodes);
  v.visit("add_heuristic_edges", config.add_heuristic_edges);
}

template <typename Visitor>
void visit_config(const Visitor& v, FloodfillExtractorConfig& config) {
  v.visit("min_extra_basis", config.min_extra_basis);
  v.visit("min_vertex_basis", config.min_vertex_basis);
  v.visit("merge_new_nodes", config.merge_new_nodes);
  v.visit("node_merge_distance_m", config.node_merge_distance_m);
  v.visit("edge_splitting_merge_nodes", config.edge_splitting_merge_nodes);
  v.visit("max_edge_split_iterations", config.max_edge_split_iterations);
  v.visit("max_edge_deviation", config.max_edge_deviation);
}

template <typename Visitor>
void visit_config(const Visitor& v, GraphExtractorConfig& config) {
  v.visit("add_overlap_edges", config.add_overlap_edges);
  if (config.add_overlap_edges) {
    v.visit("overlap_edges", config.overlap_edges);
  }

  v.visit("add_freespace_edges", config.add_freespace_edges);
  if (config.add_freespace_edges) {
    v.visit("freespace_edges", config.freespace_edges);
  }

  v.visit("use_compression_extractor", config.use_compression_extractor);
  if (config.use_compression_extractor) {
    v.visit("", config.compression);
  } else {
    v.visit("", config.floodfill);
  }
}

}  // namespace places
}  // namespace hydra
