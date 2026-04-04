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
#include <spark_dsg/node_symbol.h>

#include <queue>

#include "hydra/common/partial_graph.h"
#include "hydra/places/gvd_places/compressed_node.h"
#include "hydra/places/gvd_places/gvd_graph.h"
#include "hydra/places/gvd_places/gvd_merge_policies.h"
#include "hydra/places/gvd_places/gvd_voxel.h"
#include "hydra/reconstruction/voxel_types.h"
#include "hydra/utils/logging.h"

namespace hydra::places {

struct GvdParentTracker;

struct IndexVoxelPair {
  GlobalIndex index;
  const GvdVoxel* voxel;
};

using IndexVoxelQueue = std::list<IndexVoxelPair>;

class GraphExtractor {
 public:
  using Ptr = std::shared_ptr<GraphExtractor>;
  using NodeIndexMap = std::unordered_map<spark_dsg::NodeId, GlobalIndex>;

  struct Config : VerbosityConfig {
    //! Node prefix to use
    char prefix = 'p';
    //! Average resolution of sparse graph
    double compression_distance_m = 0.5;
    //! Minimum distance for a GVD node to be considered for compression
    double min_node_distance_m = 0.3;
    //! Minimum distance for a edge to be valid
    double min_edge_distance_m = 0.4;
    //! Whether to merge nearby neighbors
    bool merge_nearby_nodes = true;
    //! Distance to consider a merge at
    double node_merge_distance_m = 0.3;
    //! Merge policy
    std::string merge_policy = "basis_points";
    //! Configuration for overlap-based edges
    struct OverlapEdges {
      //! Turn on overlap-based edges
      bool enable = true;
      //! Minimum radius to nearest obstacle for the free-space intersection
      double min_clearance_m = 0.4;
    } overlap_edges;
    //! Configuration for freespace edges
    struct FreespaceEdges {
      //! Turn on freespace-based edges
      bool enable = true;
      //! Maximum edge length to project
      double max_length_m = 2.0;
      //! Minimum distance to the nearest obstacle along an edge
      double min_clearance_m = 0.5;
    } freespace_edges;
    Config() : VerbosityConfig("[graph_extraction] ") {}
  } const config;

  explicit GraphExtractor(const Config& config);

  virtual ~GraphExtractor();

  void pushIndex(const GlobalIndex& index);

  void clearIndex(const GlobalIndex& index);

  void archiveIndex(const GlobalIndex& index);

  void extract(const GvdLayer& layer, uint64_t timestamp_ns);

  void fillParentInfo(const GvdLayer& gvd, const GvdParentTracker& parents);

  const PartialGraph<spark_dsg::PlaceNodeAttributes>& graph() const { return graph_; }

  const GvdGraph& gvd_graph() const { return gvd_; };

 private:
  void clearArchived();

  void fillSeenVoxels(const GvdLayer& layer,
                      uint64_t timestamp_ns,
                      IndexVoxelQueue& seen_voxels);

  void updateGvdGraph(const GvdLayer& layer,
                      const IndexVoxelQueue& update_info,
                      uint64_t timestamp_ns);

  void assignCompressedNodeAttributes();

  void updateCompressedEdges(const GvdLayer& layer);

  void mergeNearbyNodes();

  void updateOverlapEdges();

  void updateFreespaceEdges(const GvdLayer& layer);

 protected:
  uint64_t next_id_;
  GvdGraph gvd_;
  CompressedGraph compressed_;
  PartialGraph<spark_dsg::PlaceNodeAttributes> graph_;

  NodeIndexMap node_index_map_;
  std::queue<GlobalIndex> modified_voxel_queue_;

  std::set<spark_dsg::EdgeKey> overlap_edges_;
  std::set<spark_dsg::EdgeKey> freespace_edges_;
  std::unordered_set<spark_dsg::NodeId> deleted_nodes_;
  std::vector<spark_dsg::EdgeKey> deleted_edges_;

  GlobalIndexMap<uint64_t> index_id_map_;
  std::unique_ptr<MergePolicy> merge_policy_;

  std::unordered_set<uint64_t> to_archive_;
  std::unordered_set<spark_dsg::NodeId> archived_node_ids_;
};

void declare_config(GraphExtractor::Config& config);

}  // namespace hydra::places
