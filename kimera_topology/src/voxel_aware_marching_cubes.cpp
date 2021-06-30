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
// all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// TODO(nathan) figure out ETHZ license

#include "kimera_topology/voxel_aware_marching_cubes.h"
#include "kimera_topology/gvd_utilities.h"

namespace kimera {
namespace topology {

using voxblox::Point;

static constexpr FloatingPoint kMinSdfDifference = 1e-6;

void interpolateEdges(const PointMatrix& vertex_coords,
                      const SdfMatrix& vertex_sdf,
                      EdgeIndexMatrix& edge_coords,
                      const std::vector<GvdVoxel*>& gvd_voxels) {
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
        setGvdSurfaceVoxel(*gvd_voxels[edge0]);
      }

      if (gvd_voxels[edge1]) {
        setGvdSurfaceVoxel(*gvd_voxels[edge1]);
      }

      continue;
    }

    // t \in [-1, 1] (as 0 \in [sdf0, sdf1])
    const float t = sdf0 / sdf_diff;
    edge_coords.col(i) = Point(vertex0 + t * (vertex1 - vertex0));

    if (gvd_voxels[edge0] && std::abs(t) <= 0.5) {
      setGvdSurfaceVoxel(*gvd_voxels[edge0]);
    }
    if (gvd_voxels[edge1] && std::abs(t) >= 0.5) {
      setGvdSurfaceVoxel(*gvd_voxels[edge1]);
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

void VoxelAwareMarchingCubes::meshCube(const PointMatrix& vertex_coords,
                                       const SdfMatrix& vertex_sdf,
                                       VertexIndex* next_index,
                                       Mesh* mesh,
                                       const std::vector<GvdVoxel*>& gvd_voxels) {
  // TODO(nathan) references
  DCHECK(next_index != nullptr);
  DCHECK(mesh != nullptr);

  const int index = calculateVertexConfig(vertex_sdf);
  if (index == 0) {
    return;  // no surface crossing in sdf cube
  }

  EdgeIndexMatrix edge_vertex_coordinates;
  interpolateEdges(vertex_coords, vertex_sdf, edge_vertex_coordinates, gvd_voxels);

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
    *next_index += 3;
    table_col += 3;
  }
}

}  // namespace topology
}  // namespace kimera
