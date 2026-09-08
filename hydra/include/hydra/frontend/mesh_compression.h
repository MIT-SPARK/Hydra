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

#include <array>
#include <functional>
#include <string>
#include <vector>

#include "hydra/frontend/mesh_compressor.h"
#include "hydra/reconstruction/volumetric_map.h"

namespace hydra {

class MeshCompression : public MeshCompressor {
 public:
  using Vertex = kimera_pgmo::traits::Vertex;
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

  kimera_pgmo::MeshDelta::Ptr update(const VolumetricMap& map,
                                     uint64_t timestamp_ns,
                                     const ArchivePredicate& archive = {});

  MeshDeltaPtr update(const ActiveWindowOutput& input,
                      const VolumetricWindow* window) override;

 private:
  struct Removal {
    GlobalIndex cell;
    std::array<Eigen::Vector3f, 3> points;
    std::string reason;
    std::array<float, 8> distances;
    std::array<float, 8> weights;
  };

  struct Entry {
    Vertex vertex;
    // Frozen boundary vertices support faces already sent for archival. They
    // cannot be cleared or reused by a new observation until fully archived.
    bool frozen = false;
  };

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
    GlobalIndexMap<bool> cleared_cells;
  };

  GlobalIndex compressionCell(const Eigen::Vector3f& pos) const;

  bool isObservedFreeSpace(const VolumetricMap& map, const GlobalIndex& cell) const;

  UpdateState initializeUpdate(const VolumetricMap& map);

  UpdateState prepareUpdate() const;

  std::vector<size_t> integrateVertices(const MeshBlock& block,
                                        uint64_t timestamp_ns,
                                        UpdateState& state);

  void integrateMeshBlock(const MeshBlock& block,
                          uint64_t timestamp_ns,
                          UpdateState& state);

  void removeClearedAndReplacedFaces(const VolumetricMap& map, UpdateState& state);

  void findUnusedVertices(UpdateState& state) const;

  std::vector<bool> findArchivableVertices(const UpdateState& state,
                                           const ArchivePredicate& archive) const;

  std::vector<size_t> appendDeltaVertices(const UpdateState& state,
                                          const std::vector<bool>& archivable,
                                          kimera_pgmo::MeshDelta& delta) const;

  void appendDeltaFaces(const std::vector<bool>& archivable,
                        const std::vector<size_t>& remap,
                        kimera_pgmo::MeshDelta& delta);

  void retainActiveVertices(const UpdateState& state,
                            const std::vector<bool>& archivable);

  void updateTracking(const kimera_pgmo::MeshDelta& delta);

  std::vector<Entry> vertices_;
  std::vector<CellFace> faces_;
  kimera_pgmo::MeshDelta::TrackingInfo tracking_{1};
};

void declare_config(MeshCompression::Config& config);

}  // namespace hydra
