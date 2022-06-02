// The original implementation from https://github.com/personalrobotics/OpenChisel and
// subsequent modifications falls under the following license:
//
// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// Modifications are also subject to the following copyright and disclaimer:
//
// Copyright 2022 Massachusetts Institute of Technology.
// All Rights Reserved
//
// Research was sponsored by the United States Air Force Research Laboratory and
// the United States Air Force Artificial Intelligence Accelerator and was
// accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
// and conclusions contained in this document are those of the authors and should
// not be interpreted as representing the official policies, either expressed or
// implied, of the United States Air Force or the U.S. Government. The U.S.
// Government is authorized to reproduce and distribute reprints for Government
// purposes notwithstanding any copyright notation herein.
#include "hydra_topology/voxel_aware_marching_cubes.h"
#include "hydra_topology/gvd_utilities.h"

namespace hydra {
namespace topology {

using voxblox::Point;

static constexpr FloatingPoint kMinSdfDifference = 1e-6;

void interpolateEdges(const PointMatrix& vertex_coords,
                      const SdfMatrix& vertex_sdf,
                      EdgeIndexMatrix& edge_coords,
                      std::vector<uint8_t>& edge_status,
                      const std::vector<GvdVoxel*>& gvd_voxels) {
  // we use the first two bits to denote status
  edge_status = std::vector<uint8_t>(12, 0);
  for (size_t i = 0; i < 12; ++i) {
    const int* pairs = voxblox::MarchingCubes::kEdgeIndexPairs[i];
    const int edge0 = pairs[0];
    const int edge1 = pairs[1];
    const float sdf0 = vertex_sdf(edge0);
    const float sdf1 = vertex_sdf(edge1);

    if (std::signbit(sdf0) == std::signbit(sdf1) && sdf0 != 0.0 && sdf1 != 0.0) {
      continue;  // zero-crossing must be present
    }

    const Point vertex0 = vertex_coords.col(edge0);
    const Point vertex1 = vertex_coords.col(edge1);

    const float sdf_diff = sdf0 - sdf1;
    if (std::abs(sdf_diff) <= kMinSdfDifference) {
      edge_coords.col(i) = Point(0.5f * (vertex0 + vertex1));

      if (gvd_voxels[edge0]) {
        edge_status[i] |= 0x01;
      }

      if (gvd_voxels[edge1]) {
        edge_status[i] |= 0x02;
      }

      continue;
    }

    // t \in [-1, 1] (as 0 \in [sdf0, sdf1])
    const float t = sdf0 / sdf_diff;
    edge_coords.col(i) = Point(vertex0 + t * (vertex1 - vertex0));

    if (gvd_voxels[edge0] && std::abs(t) <= 0.5) {
      edge_status[i] |= 0x01;
    }
    if (gvd_voxels[edge1] && std::abs(t) >= 0.5) {
      edge_status[i] |= 0x02;
    }
  }
}

VoxelAwareMarchingCubes::VoxelAwareMarchingCubes() : voxblox::MarchingCubes() {}

inline int calculateVertexConfig(const SdfMatrix& vertex_sdf) {
  // voxblox / open-chisel version doesn't handle zeroed SDF values correctly
  // on mesh boundaries
  int to_return = 0;
  to_return |= (vertex_sdf(0) <= 0.0) ? 0x01 : 0;
  to_return |= (vertex_sdf(1) <= 0.0) ? 0x02 : 0;
  to_return |= (vertex_sdf(2) <= 0.0) ? 0x04 : 0;
  to_return |= (vertex_sdf(3) <= 0.0) ? 0x08 : 0;
  to_return |= (vertex_sdf(4) <= 0.0) ? 0x10 : 0;
  to_return |= (vertex_sdf(5) <= 0.0) ? 0x20 : 0;
  to_return |= (vertex_sdf(6) <= 0.0) ? 0x40 : 0;
  to_return |= (vertex_sdf(7) <= 0.0) ? 0x80 : 0;
  return to_return;
}

inline void updateVoxels(const BlockIndex& block,
                         int edge_coord,
                         VertexIndex new_vertex_index,
                         const std::vector<uint8_t>& status,
                         const std::vector<GvdVoxel*>& gvd_voxels,
                         const std::vector<bool>& voxels_in_block) {
  const int* pairs = voxblox::MarchingCubes::kEdgeIndexPairs[edge_coord];
  const uint8_t curr_status = status[edge_coord];

  GvdVoxel* first_voxel = gvd_voxels[pairs[0]];
  if (first_voxel && voxels_in_block[pairs[0]] && (curr_status & 0x01)) {
    setGvdSurfaceVoxel(*first_voxel);
    first_voxel->block_vertex_index = new_vertex_index;
    std::memcpy(first_voxel->mesh_block, block.data(), sizeof(first_voxel->mesh_block));
  }

  GvdVoxel* second_voxel = gvd_voxels[pairs[1]];
  if (second_voxel && voxels_in_block[pairs[1]] && (curr_status & 0x02)) {
    setGvdSurfaceVoxel(*second_voxel);
    second_voxel->block_vertex_index = new_vertex_index;
    std::memcpy(
        second_voxel->mesh_block, block.data(), sizeof(second_voxel->mesh_block));
  }
}

void VoxelAwareMarchingCubes::meshCube(const BlockIndex& block,
                                       const PointMatrix& vertex_coords,
                                       const SdfMatrix& vertex_sdf,
                                       VertexIndex* next_index,
                                       Mesh* mesh,
                                       const std::vector<GvdVoxel*>& gvd_voxels,
                                       const std::vector<bool>& voxels_in_block) {
  // TODO(nathan) references
  DCHECK(next_index != nullptr);
  DCHECK(mesh != nullptr);

  const int index = calculateVertexConfig(vertex_sdf);
  if (index == 0) {
    return;  // no surface crossing in sdf cube
  }

  EdgeIndexMatrix edge_vertex_coordinates;
  std::vector<uint8_t> edge_status;
  interpolateEdges(
      vertex_coords, vertex_sdf, edge_vertex_coordinates, edge_status, gvd_voxels);

  const int* table_row = kTriangleTable[index];

  int table_col = 0;
  while (table_row[table_col] != -1) {
    mesh->vertices.emplace_back(edge_vertex_coordinates.col(table_row[table_col + 2]));
    mesh->vertices.emplace_back(edge_vertex_coordinates.col(table_row[table_col + 1]));
    mesh->vertices.emplace_back(edge_vertex_coordinates.col(table_row[table_col]));
    mesh->indices.push_back(*next_index);
    mesh->indices.push_back((*next_index) + 1);
    mesh->indices.push_back((*next_index) + 2);
    const Point& p0 = mesh->vertices[*next_index];
    const Point& p1 = mesh->vertices[*next_index + 1];
    const Point& p2 = mesh->vertices[*next_index + 2];
    Point px = (p1 - p0);
    Point py = (p2 - p0);
    Point n = px.cross(py).normalized();
    mesh->normals.push_back(n);
    mesh->normals.push_back(n);
    mesh->normals.push_back(n);

    // mark voxels with a nearest vertex. overwriting is okay (as remapping downstream
    // tracks which vertices are the same)
    updateVoxels(block,
                 table_row[table_col + 2],
                 *next_index,
                 edge_status,
                 gvd_voxels,
                 voxels_in_block);
    updateVoxels(block,
                 table_row[table_col + 1],
                 *next_index + 1,
                 edge_status,
                 gvd_voxels,
                 voxels_in_block);
    updateVoxels(block,
                 table_row[table_col],
                 *next_index + 2,
                 edge_status,
                 gvd_voxels,
                 voxels_in_block);

    *next_index += 3;
    table_col += 3;
  }
}

}  // namespace topology
}  // namespace hydra
