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

#include <kimera_pgmo/mesh_delta.h>
#include <kimera_pgmo/utils/vertex_update.h>

#include <functional>
#include <vector>

#include "hydra/frontend/mesh_compressor.h"
#include "hydra/reconstruction/volumetric_map.h"

namespace hydra {

class MeshCompression : public MeshCompressor {
 public:
  using Vertex = kimera_pgmo::traits::Vertex;
  using Traits = kimera_pgmo::traits::VertexTraits;
  using Pos = kimera_pgmo::traits::Pos;
  using Face = kimera_pgmo::traits::Face;
  using ArchivePredicate = std::function<bool(const Vertex&)>;

  struct Config {
    //! Compression resolution
    double resolution = 0.005;
    //! Min observation weight for TSDF
    float min_weight = 1.0e-6f;
    //! Minimum TSDF distance for free-space
    double min_clearance_m = 0.0;
  } const config;

  explicit MeshCompression(double resolution);

  explicit MeshCompression(const Config& config);

  template <typename MergeT = kimera_pgmo::DefaultVertexUpdate>
  kimera_pgmo::MeshDelta::Ptr update(const VolumetricMap& map,
                                     uint64_t timestamp_ns,
                                     const ArchivePredicate& archive = {});

  MeshDeltaPtr update(const ActiveWindowOutput& input,
                      const VolumetricWindow* window) override;

 private:
  struct Entry {
    //! Position of compressed entry
    kimera_pgmo::traits::Pos pos;
    //! Traits of compressed entry
    kimera_pgmo::traits::VertexTraits traits;
    //! Whether or not the entry can be deelted
    bool frozen = false;
  };

  using UpdateCallback = std::function<void(const Pos&, const Traits&, Entry&)>;

  struct CellFace {
    Face vertices;
    GlobalIndex cell;
  };

  struct UpdateState {
    size_t previous_vertices;
    size_t previous_faces;
    std::vector<bool> deleted;
    std::vector<bool> observed;
    GlobalIndexMap<size_t> mutable_cells;
    GlobalIndexSet reobserved_cells;
  };

  GlobalIndex cellIndex(const Eigen::Vector3f& pos) const;

  bool isFree(const VolumetricMap& map, const GlobalIndex& cell) const;

  UpdateState prepare(const VolumetricMap& map) const;

  void integrate(const MeshBlock& block,
                 UpdateState& state,
                 const UpdateCallback& callback);

  void prune(const VolumetricMap& map, UpdateState& state);

  kimera_pgmo::MeshDelta::Ptr makeDelta(const UpdateState& state,
                                        uint64_t timestamp_ns,
                                        const ArchivePredicate& archive);

  std::vector<Entry> vertices_;
  std::vector<CellFace> faces_;
  kimera_pgmo::MeshDelta::TrackingInfo tracking_{1};
};

void declare_config(MeshCompression::Config& config);

template <typename MergeT>
kimera_pgmo::MeshDelta::Ptr MeshCompression::update(const VolumetricMap& map,
                                                    uint64_t stamp,
                                                    const ArchivePredicate& archive) {
  auto state = prepare(map);
  for (const auto& block : map.getMeshLayer()) {
    integrate(block, state, [stamp](const auto& pos, const auto& traits, auto& entry) {
      constexpr static const MergeT merge;
      merge(stamp, pos, traits, entry.pos, entry.traits);
    });
  }

  prune(map, state);
  return makeDelta(state, stamp, archive);
}

}  // namespace hydra
