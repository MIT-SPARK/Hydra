#pragma once

#include <kimera_pgmo/hashing.h>
#include <kimera_pgmo/mesh_offset_info.h>
#include <spatial_hash/grid.h>
#include <spatial_hash/hash.h>

#include <optional>
#include <vector>

namespace hydra {

// Correspondence for one compressor update, valid until the next update.
// Indices are local to its MeshDelta; MeshOffsetInfo converts them to graph indices.
struct MeshCorrespondence {
  explicit MeshCorrespondence(double resolution) : grid(resolution) {}

  void clear() {
    if (sources) {
      sources->clear();
    }
    active.clear();
    retained.clear();
  }

  std::optional<size_t> find(const Eigen::Vector3f& point) const {
    const auto it = active.find(grid.toIndex(point));
    return it == active.end() ? std::nullopt : std::optional<size_t>(it->second);
  }

  std::optional<size_t> find(const spatial_hash::BlockIndex& block,
                             size_t vertex,
                             const Eigen::Vector3f& point) const {
    if (!sources) {
      return find(point);
    }
    const auto block_it = sources->find(block);
    if (block_it == sources->end()) {
      return std::nullopt;
    }
    const auto it = block_it->second.find(vertex);
    return it == block_it->second.end() ? std::nullopt
                                        : std::optional<size_t>(it->second);
  }

  //! Source identity correspondence when provided by the compressor. Otherwise,
  //! the voxel compressor resolves current support through compression cells.
  std::optional<kimera_pgmo::HashedIndexMapping> sources;
  spatial_hash::Grid<spatial_hash::LongIndex> grid;
  //! Current vertex for each compression cell, including unchanged support.
  spatial_hash::LongIndexHashMap<size_t> active;
  //! Includes frozen and newly archived vertices that may share a cell with
  //! an active replacement. Used only to retain previously owned connections.
  spatial_hash::LongIndexHashMap<std::vector<size_t>> retained;
};

struct MeshUpdateInfo {
  kimera_pgmo::MeshOffsetInfo offsets;
  //! Borrowed from the compressor; consumed after all input callbacks finish.
  const MeshCorrespondence* correspondence = nullptr;
};

}  // namespace hydra
