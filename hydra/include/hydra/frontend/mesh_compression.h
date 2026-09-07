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
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

#include "hydra/frontend/mesh_compressor.h"
#include "hydra/reconstruction/volumetric_map.h"

namespace hydra {

/** Spatially compress a mesh, retaining geometry until the TSDF observes free space.
 *
 * Input mesh blocks must supply face_cells provenance from MeshIntegrator. Active
 * triangles are replaced by source cell; clearing requires eight observed free
 * TSDF corners. Shared vertices survive while retained faces need them.
 * Input maps may contain only updated blocks. Missing blocks and zero-weight voxels
 * are unknown, not evidence that a surface disappeared. Archival is specified at
 * each vertex through a window predicate, independently of TSDF block ownership.
 * Archived geometry is immutable, as required by MeshDelta.
 */
class MeshCompression : public MeshCompressor {
 public:
  using Vertex = kimera_pgmo::traits::Vertex;
  using Face = kimera_pgmo::traits::Face;
  using ArchivePredicate = std::function<bool(const Vertex&)>;

  struct Config {
    double resolution = 0.01;
    float min_weight = 1.0e-6f;
    // All eight source-cell corners must exceed this distance to clear a cell.
    // Zero uses the compression resolution as a positive-distance margin.
    double min_clearance_m = 0.0;
    // Ablations for diagnosing loss of valid surfaces.
    bool clear_free_space = true;
    bool replace_reobserved_cells = true;
  };

  explicit MeshCompression(double resolution);
  explicit MeshCompression(const Config& config);

  // The predicate returns true for vertices outside the active window. Without
  // a predicate geometry remains active; absence from a partial map never archives.
  // Vertices reobserved in this update remain active even outside the window.
  kimera_pgmo::MeshDelta::Ptr update(const VolumetricMap& map,
                                     uint64_t timestamp_ns,
                                     const ArchivePredicate& archive = {});

  kimera_pgmo::MeshDelta::Ptr update(const ActiveWindowOutput& input,
                                     const VolumetricWindow* window) override;

  // Diagnostic snapshots are opt-in and excluded from normal benchmark runs.
  void enableDiagnostics(bool enabled);
  void saveDiagnostics(const std::filesystem::path& output) const;

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
  void integrateMeshBlock(const VolumetricMap& map,
                          const MeshBlock& block,
                          uint64_t timestamp_ns,
                          UpdateState& state);
  void removeClearedAndReplacedFaces(const VolumetricMap& map, UpdateState& state);
  void findUnusedVertices(UpdateState& state) const;
  void recordRemovalPoints(const VolumetricMap& map,
                           const GlobalIndex& cell,
                           const std::array<Eigen::Vector3f, 3>& points,
                           const char* reason);
  void recordRemoval(const VolumetricMap& map,
                     const CellFace& face,
                     const char* reason);
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

  std::vector<Eigen::Vector3f> diagnostic_positions_;
  bool diagnostics_enabled_ = false;
  std::vector<Removal> removals_;
  const Config config_;
  std::vector<Entry> vertices_;
  std::vector<CellFace> faces_;
  kimera_pgmo::MeshDelta::TrackingInfo tracking_{1};
};

void declare_config(MeshCompression::Config& config);

}  // namespace hydra
