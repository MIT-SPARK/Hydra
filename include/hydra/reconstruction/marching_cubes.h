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
#include <optional>

#include "hydra/common/common.h"
#include "hydra/reconstruction/voxel_types.h"

namespace hydra {

struct OccupancyVoxel;

struct SdfPoint {
  float distance;
  float weight;
  Eigen::Vector3f pos;
  Color color;
  std::optional<uint32_t> label;
  OccupancyVoxel* vertex_voxel = nullptr;
};

std::ostream& operator<<(std::ostream& out, const SdfPoint& point);

class MarchingCubes {
 public:
  using EdgePoints = std::array<SdfPoint, 12>;
  using EdgeStatus = std::array<uint8_t, 12>;
  using SdfPoints = std::array<SdfPoint, 8>;

  static void interpolateEdges(const SdfPoints& points,
                               EdgePoints& edge_points,
                               EdgeStatus& edge_status,
                               float min_sdf_difference = 1.0e-6);

  static void meshCube(const BlockIndex& block,
                       const SdfPoints& points,
                       spark_dsg::Mesh& mesh,
                       bool compute_normals = true);

  static const int kTriangleTable[256][16];
  static const int kEdgeIndexPairs[12][2];
};

}  // namespace hydra
