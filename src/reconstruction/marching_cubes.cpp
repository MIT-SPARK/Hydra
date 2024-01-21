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
#include "hydra/reconstruction/marching_cubes.h"

#include <voxblox/mesh/marching_cubes.h>

#include "hydra/common/common.h"
#include "hydra/reconstruction/voxblox_utilities.h"

namespace hydra {

using voxblox::BlockIndex;
using voxblox::Mesh;

std::ostream& operator<<(std::ostream& out, const SdfPoint& point) {
  out << "<";
  out << "d=" << point.distance;
  out << ", weight=" << point.weight;
  out << ", pos=[" << point.pos.x() << ", " << point.pos.y() << ", " << point.pos.z()
      << "]";
  out << ", color=[" << static_cast<int>(point.color.r) << ", "
      << static_cast<int>(point.color.g) << ", " << static_cast<int>(point.color.b)
      << "]";
  out << ", label=" << (point.label ? std::to_string(point.label.value()) : "n/a");
  out << ", vertex=" << (point.vertex_voxel ? "y" : "n");
  out << ">";
  return out;
}

std::optional<uint32_t> interpLabel(const SdfPoint& v0, const SdfPoint& v1, float t) {
  const auto val = std::abs(t);
  if (std::abs(t) < 0.5f) {
    return v0.label;
  } else if (val == 0.5f) {
    return (v0.weight > v1.weight) ? v0.label : v1.label;
  } else {
    return v1.label;
  }
}

voxblox::Color interpColor(const SdfPoint& v0, const SdfPoint& v1, float t) {
  const auto r = static_cast<uint8_t>(v0.color.r + t * (v1.color.r - v0.color.r));
  const auto g = static_cast<uint8_t>(v0.color.g + t * (v1.color.g - v0.color.g));
  const auto b = static_cast<uint8_t>(v0.color.b + t * (v1.color.b - v0.color.b));
  const auto a = static_cast<uint8_t>(v0.color.a + t * (v1.color.a - v0.color.a));
  return {r, g, b, a};
}

void MarchingCubes::interpolateEdges(const SdfPoints& points,
                                     EdgePoints& edge_points,
                                     EdgeStatus& edge_status,
                                     float min_sdf_difference) {
  for (size_t i = 0; i < 12; ++i) {
    // clear edge status flags
    edge_status[i] = 0;

    const auto* pairs = voxblox::MarchingCubes::kEdgeIndexPairs[i];
    const auto edge0 = pairs[0];
    const auto edge1 = pairs[1];
    const auto sdf0 = points[edge0].distance;
    const auto sdf1 = points[edge1].distance;
    auto& edge_point = edge_points[i];

    const bool has_crossing =
        (sdf0 < 0.0f && sdf1 >= 0.0f) || (sdf1 < 0.0f && sdf0 >= 0.0f);
    if (!has_crossing) {
      continue;  // zero-crossing must be present
    }

    const auto& point0 = points[edge0];
    const auto& point1 = points[edge1];

    // TODO(nathan): this rarely triggers / should never tigger
    // this case corresponds to a plane nearly parallel to the face containing the two
    // corners intersecting at some point through the edge between the two corners.
    const float sdf_diff = sdf0 - sdf1;
    if (std::abs(sdf_diff) <= min_sdf_difference) {
      edge_point.pos = 0.5f * (point0.pos + point1.pos);
      // force interpolation to occur exactly in the middle
      edge_point.color = interpColor(point0, point1, 0.5);
      edge_point.label = interpLabel(point0, point1, 0.5);

      VLOG(VLEVEL_EXTRA) << "- t=n/a"
                         << ", v0=" << point0.pos.transpose()
                         << ", v1=" << point1.pos.transpose()
                         << ", coord: " << edge_point.pos.transpose();

      edge_status[i] |= 0x01;
      edge_status[i] |= 0x02;
      continue;
    }

    // t \in [-1, 1] (as 0 \in [sdf0, sdf1])
    const float t = sdf0 / sdf_diff;
    edge_point.pos = point0.pos + t * (point1.pos - point0.pos);
    edge_point.color = interpColor(point0, point1, t);
    edge_point.label = interpLabel(point0, point1, t);

    VLOG(VLEVEL_EXTRA) << "- t=" << t << ", v0=" << point0.pos.transpose()
                       << ", v1=" << point1.pos.transpose()
                       << ", coord: " << edge_point.pos.transpose();

    if (std::abs(t) <= 0.5) {
      edge_status[i] |= 0x01;
    }

    if (std::abs(t) >= 0.5) {
      edge_status[i] |= 0x02;
    }
  }
}

inline int calculateVertexConfig(const MarchingCubes::SdfPoints& points) {
  // voxblox / open-chisel version doesn't handle zeroed SDF values correctly
  // on mesh boundaries
  int to_return = 0;
  to_return |= (points[0].distance < 0.0) ? 0x01 : 0;
  to_return |= (points[1].distance < 0.0) ? 0x02 : 0;
  to_return |= (points[2].distance < 0.0) ? 0x04 : 0;
  to_return |= (points[3].distance < 0.0) ? 0x08 : 0;
  to_return |= (points[4].distance < 0.0) ? 0x10 : 0;
  to_return |= (points[5].distance < 0.0) ? 0x20 : 0;
  to_return |= (points[6].distance < 0.0) ? 0x40 : 0;
  to_return |= (points[7].distance < 0.0) ? 0x80 : 0;
  return to_return;
}

inline void updateVoxels(const BlockIndex& block,
                         int edge_coord,
                         size_t new_vertex_index,
                         const MarchingCubes::EdgeStatus& status,
                         const MarchingCubes::SdfPoints& points) {
  // note that because of the pointer indirection, const points work fine for this
  // this is somewhat intentional; nothing specific to struct should ever change (just
  // the underlying vertex voxel info)
  const int* pairs = ::voxblox::MarchingCubes::kEdgeIndexPairs[edge_coord];
  const uint8_t curr_status = status[edge_coord];

  if (VLOG_IS_ON(VLEVEL_EXTRA) && (curr_status & 0x01 || curr_status & 0x02)) {
    VLOG(VLEVEL_EXTRA) << "vertex added: " << new_vertex_index << " @ "
                       << showIndex(block);
  }

  auto* first_voxel = points[pairs[0]].vertex_voxel;
  if (first_voxel && (curr_status & 0x01)) {
    first_voxel->on_surface = true;
    first_voxel->block_vertex_index = new_vertex_index;
    std::memcpy(first_voxel->mesh_block, block.data(), sizeof(first_voxel->mesh_block));
  }

  auto* second_voxel = points[pairs[1]].vertex_voxel;
  if (second_voxel && (curr_status & 0x02)) {
    second_voxel->on_surface = true;
    second_voxel->block_vertex_index = new_vertex_index;
    std::memcpy(
        second_voxel->mesh_block, block.data(), sizeof(second_voxel->mesh_block));
  }
}

void MarchingCubes::meshCube(const BlockIndex& block,
                             const SdfPoints& points,
                             Mesh& mesh,
                             std::vector<uint32_t>* mesh_labels,
                             bool compute_normals) {
  if (VLOG_IS_ON(VLEVEL_EXTRA)) {
    VLOG(VLEVEL_EXTRA) << "[mesh] points: ";
    for (size_t i = 0; i < 8; ++i) {
      VLOG(VLEVEL_EXTRA) << " - " << i << ": " << points[i];
    }
  }

  const int index = calculateVertexConfig(points);
  VLOG(VLEVEL_EXTRA) << "[mesh] vertex sdf index: " << index;

  if (index == 0) {
    return;  // no surface crossing in sdf cube
  }

  // TODO(nathan) augment edge points
  EdgePoints edge_points;
  EdgeStatus status;
  interpolateEdges(points, edge_points, status);

  const int* table_row = ::voxblox::MarchingCubes::kTriangleTable[index];

  int table_col = 0;
  size_t next_index = mesh.size();
  while (table_row[table_col] != -1) {
    const auto& v1 = edge_points[table_row[table_col + 2]];
    const auto& v2 = edge_points[table_row[table_col + 1]];
    const auto& v3 = edge_points[table_row[table_col]];
    mesh.vertices.emplace_back(v1.pos);
    mesh.vertices.emplace_back(v2.pos);
    mesh.vertices.emplace_back(v3.pos);
    mesh.colors.emplace_back(v1.color);
    mesh.colors.emplace_back(v2.color);
    mesh.colors.emplace_back(v3.color);
    mesh.indices.push_back(next_index);
    mesh.indices.push_back(next_index + 1);
    mesh.indices.push_back(next_index + 2);
    if (mesh_labels) {
      mesh_labels->push_back(v1.label.value_or(std::numeric_limits<uint32_t>::max()));
      mesh_labels->push_back(v2.label.value_or(std::numeric_limits<uint32_t>::max()));
      mesh_labels->push_back(v3.label.value_or(std::numeric_limits<uint32_t>::max()));
    }

    if (compute_normals) {
      const auto& p0 = mesh.vertices[next_index];
      const auto& p1 = mesh.vertices[next_index + 1];
      const auto& p2 = mesh.vertices[next_index + 2];
      const auto n = (p1 - p0).cross((p2 - p0)).normalized();
      mesh.normals.push_back(n);
      mesh.normals.push_back(n);
      mesh.normals.push_back(n);
    }

    // mark voxels with a nearest vertex. overwriting is okay (as remapping downstream
    // tracks which vertices are the same)
    updateVoxels(block, table_row[table_col + 2], next_index, status, points);
    updateVoxels(block, table_row[table_col + 1], next_index + 1, status, points);
    updateVoxels(block, table_row[table_col], next_index + 2, status, points);

    next_index += 3;
    table_col += 3;
  }
}

}  // namespace hydra
