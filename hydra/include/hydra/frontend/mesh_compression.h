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

#include <functional>
#include <vector>

#include "hydra/reconstruction/volumetric_map.h"

namespace hydra {

/** Spatially compress a mesh, retaining geometry until the TSDF observes free space.
 *
 * Input maps may contain only updated blocks. Missing blocks and zero-weight voxels
 * are unknown, not evidence that a surface disappeared. Archival is specified at
 * each vertex through a window predicate, independently of TSDF block ownership.
 * Archived geometry is immutable, as required by MeshDelta.
 */
class MeshCompression {
 public:
  using Vertex = kimera_pgmo::traits::Vertex;
  using Face = kimera_pgmo::traits::Face;
  using ArchivePredicate = std::function<bool(const Vertex&)>;

  struct Config {
    double resolution = 0.01;
    float min_weight = 1.0e-6f;
    // Minimum positive TSDF distance for clearing. Zero selects a conservative
    // tolerance based on the TSDF voxel size and the compression cell diagonal.
    double min_clearance_m = 0.0;
  };

  explicit MeshCompression(double resolution);
  explicit MeshCompression(const Config& config);

  // The predicate returns true for vertices outside the active window. Without
  // a predicate geometry remains active; absence from a partial map never archives.
  // Vertices reobserved in this update remain active even outside the window.
  kimera_pgmo::MeshDelta::Ptr update(const VolumetricMap& map,
                                     uint64_t timestamp_ns,
                                     const ArchivePredicate& archive = {});

 private:
  struct Entry {
    Vertex vertex;
    // Frozen boundary vertices support faces already sent for archival. They
    // cannot be cleared or reused by a new observation until fully archived.
    bool frozen = false;
  };

  struct UpdateState {
    size_t previous_vertices;
    size_t previous_faces;
    std::vector<bool> deleted;
    std::vector<bool> observed;
    GlobalIndexMap<size_t> mutable_cells;
  };

  GlobalIndex compressionCell(const Eigen::Vector3f& pos) const;
  bool isObservedFreeSpace(const VolumetricMap& map, const Eigen::Vector3f& pos) const;
  UpdateState prepareUpdate(const VolumetricMap& map) const;
  void integrateMeshBlock(const VolumetricMap& map,
                          const MeshBlock& block,
                          uint64_t timestamp_ns,
                          UpdateState& state);
  void markReobservedFrozenVertices(UpdateState& state) const;
  void removeClearedAndReplacedFaces(const UpdateState& state);
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

  const Config config_;
  std::vector<Entry> vertices_;
  std::vector<Face> faces_;
  kimera_pgmo::MeshDelta::TrackingInfo tracking_{1};
};

void declare_config(MeshCompression::Config& config);

}  // namespace hydra
