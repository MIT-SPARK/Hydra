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

#include <spark_dsg/mesh.h>

#include <Eigen/Dense>
#include <array>
#include <limits>
#include <optional>
#include <vector>

#include "hydra/reconstruction/voxel_types.h"

namespace hydra {

struct SdfPoint {
  float distance;
  float weight;
  Eigen::Vector3f pos;
  spark_dsg::Color color;
  std::optional<uint32_t> label;
  const TrackingVoxel* tracking_voxel = nullptr;
};

std::ostream& operator<<(std::ostream& out, const SdfPoint& point);

class MarchingCubes {
 public:
  using EdgePoints = std::array<SdfPoint, 12>;
  using SdfPoints = std::array<SdfPoint, 8>;

  // One index per lattice edge, including the positive boundary halo. Use only
  // with a single output mesh and discard when that mesh is cleared.
  class EdgeCache {
   public:
    explicit EdgeCache(size_t cubes_per_side);
    size_t& index(const Eigen::Vector3i& cube, int edge);

    static constexpr size_t kInvalid = std::numeric_limits<size_t>::max();

   private:
    size_t side_;
    std::array<std::vector<size_t>, 3> indices_;
  };

  static void interpolateEdges(const SdfPoints& points,
                               EdgePoints& edge_points,
                               float min_sdf_difference = 1.0e-6);

  // Append every table triangle and return the number of faces added. With a
  // cache, cube is the block-local integer origin; different lattice edges stay
  // distinct even when their intersections coincide. Without a cache, append
  // independent vertices for each face.
  static size_t meshCube(const SdfPoints& points,
                         spark_dsg::Mesh& mesh,
                         bool compute_normals = true,
                         EdgeCache* cache = nullptr,
                         const Eigen::Vector3i& cube = Eigen::Vector3i::Zero());

  static const int kTriangleTable[256][16];
  static const int kEdgeIndexPairs[12][2];

 private:
  static SdfPoint interpolateEdge(const SdfPoints& points,
                                  int edge,
                                  float min_sdf_difference);
};

}  // namespace hydra
