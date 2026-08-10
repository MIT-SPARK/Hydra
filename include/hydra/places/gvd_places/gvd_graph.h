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
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/scene_graph_types.h>
#include <spatial_hash/neighbor_utils.h>

#include <Eigen/Dense>
#include <cstdint>
#include <list>
#include <map>
#include <optional>
#include <set>
#include <unordered_map>

#include "hydra/places/gvd_places/gvd_merge_policies.h"
#include "hydra/reconstruction/voxel_types.h"

namespace hydra::places {

struct GvdMemberInfo {
  double distance = 0.0;
  uint8_t num_basis_points = 0;
  Eigen::Vector3f position;
  GlobalIndex index;
  std::vector<Point> parents;
  uint64_t last_updated = 0;
};

class GvdGraph {
 public:
  struct Node {
    uint64_t id;
    GvdMemberInfo info;
    std::set<uint64_t> siblings;
    bool archived = false;
  };

  struct CompressedNode {
    explicit CompressedNode(uint64_t node_id);

    bool archived() const;
    std::vector<uint64_t> refs() const;

    uint64_t node_id;
    std::set<uint64_t> siblings;
    std::set<uint64_t> active_refs;
    std::set<uint64_t> archived_refs;
  };

  using Nodes = std::unordered_map<uint64_t, Node>;
  using CompressedNodes = std::map<uint64_t, CompressedNode>;
  using NodeRemapping = std::unordered_map<uint64_t, uint64_t>;

  GvdGraph(float voxel_resolution_m, float compression_resolution_m);

  void add(const GlobalIndex& index,
           double distance,
           uint8_t num_basis_points,
           const std::vector<Point>& parents = {},
           uint64_t timestamp = 0);

  void remove(const GlobalIndexSet& indices);

  void archive(const GlobalIndex& index);

  void dropCompressed(uint64_t compressed_id);

  std::vector<uint64_t> clearArchived();

  const GvdMemberInfo* get(uint64_t uncompressed_id) const;

  const GvdMemberInfo* getCompressed(uint64_t compressed_id,
                                     const MergePolicy& policy) const;

  const Nodes& uncompressed() const;

  const CompressedNodes& compressed() const;

  const NodeRemapping& remapping() const;

  const spatial_hash::IndexGrid voxel_grid;
  const spatial_hash::IndexGrid compression_grid;
  const spatial_hash::NeighborSearch neighbor_search;

 protected:
  void remove(const GlobalIndex& index, std::set<uint64_t>& updated);

  uint64_t next_uncompressed_id();

  uint64_t next_compressed_id();

  Node* add_uncompressed(const GlobalIndex& index,
                         double distance,
                         uint8_t basis,
                         const std::vector<Point>& parents,
                         uint64_t timestamp);

  void add_compressed(const Node& node);

  Node* uncompressed_by_index(const GlobalIndex& index);

  uint64_t assign_to_cluster(const Node& node);

  std::optional<uint64_t> cluster_for_gvd(uint64_t gvd_id) const;

  void merge_clusters(uint64_t target, uint64_t candidate);

  std::optional<uint64_t> drop_uncompressed(const GlobalIndex& index);

  void drop_uncompressed_node(uint64_t gvd_id);

  void drop_compressed_id(uint64_t compressed_id);

 protected:
  uint64_t next_uncompressed_id_;
  Nodes uncompressed_;
  std::list<uint64_t> uncompressed_id_queue_;
  spatial_hash::IndexHashMap<uint64_t> uncompressed_index_map_;

  uint64_t next_compressed_id_;
  CompressedNodes compressed_;
  spatial_hash::IndexHashMap<std::set<uint64_t>> compressed_index_map_;
  std::unordered_map<uint64_t, spatial_hash::Index> compressed_id_map_;

  NodeRemapping compression_map_;
};

}  // namespace hydra::places
