#pragma once

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
    active.clear();
    retained.clear();
  }

  std::optional<size_t> find(const Eigen::Vector3f& point) const {
    const auto it = active.find(grid.toIndex(point));
    return it == active.end() ? std::nullopt : std::optional<size_t>(it->second);
  }

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
