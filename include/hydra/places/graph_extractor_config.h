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
#include <string>

namespace hydra::places {

struct OverlapEdgeConfig {
  //! Number of nearest neighbors to check for free-space overlap
  size_t num_neighbors_to_check = 4;
  //! Minimum radius to nearest obstacle for the free-space intersection
  double min_clearance_m = 0.4;
};

struct FreespaceEdgeConfig {
  //! Maximum edge length to project
  double max_length_m = 2.0;
  //! Number of nodes to check in a disconnected component for edge candidates
  size_t num_nodes_to_check = 5;
  //! Number of nearest neighbors to find in another disconnected component
  size_t num_neighbors_to_find = 1;
  //! Minimum distance to the nearest obstacle along an edge
  double min_clearance_m = 0.5;
};

struct GraphExtractorConfig {
  //! Add edges between nodes that have overlapping free-space regions
  bool add_overlap_edges = true;
  //! Configuration for overlap-based edges
  OverlapEdgeConfig overlap_edges;
  //! Add edges between disconnected components
  bool add_freespace_edges = true;
  //! Configuration for freespace edges
  FreespaceEdgeConfig freespace_edges;
  //! Maximum distance between two nodes to consider a merge
  double node_merge_distance_m = 0.2;
};

struct CompressionExtractorConfig : public GraphExtractorConfig {
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
  //! Whether to merge nearby neighbors
  bool merge_nearby_nodes = true;
  //! Whether or not to add heuristic edges
  bool add_heuristic_edges = false;
  //! Merge policy
  std::string merge_policy = "basis_points";
};

struct FloodfillExtractorConfig : public GraphExtractorConfig {
  //! Number of basis points for a voxel to be consider for extraction
  uint8_t min_extra_basis = 2;
  //! Number of basis points for a voxel to be automatically labeled a vertex
  uint8_t min_vertex_basis = 3;
  //! Whether or not to merge nodes close together during initial edge extraction
  bool merge_new_nodes = true;
  //! @brief Whether or not to merge nodes close together when splitting edges
  bool edge_splitting_merge_nodes = true;
  //! Number of maximum iterations to run edges splitting (set to 0 to disable)
  size_t max_edge_split_iterations = 5;
  //! Maximum squared voxel distance an edge can be from supporting voxels at any point
  int64_t max_edge_deviation = 4;
};

void declare_config(OverlapEdgeConfig& conf);
void declare_config(GraphExtractorConfig& conf);
void declare_config(CompressionExtractorConfig& conf);
void declare_config(FloodfillExtractorConfig& conf);

}  // namespace hydra::places
