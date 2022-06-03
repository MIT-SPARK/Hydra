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
#include "hydra_topology/graph_extraction_utilities.h"

namespace hydra {
namespace topology {

using voxblox::Connectivity;
using Neighborhood26Connected = Neighborhood<Connectivity::kTwentySix>;
using IndexOffsets26Connected = Neighborhood<Connectivity::kTwentySix>::IndexOffsets;

std::bitset<27> convertRowMajorFlags(std::bitset<27> flags_row_major) {
  std::bitset<27> flags_neighborhood{0};

  const IndexOffsets26Connected& offsets = Neighborhood26Connected::kOffsets;
  for (int i = 0; i < offsets.cols(); ++i) {
    // x -> y -> z ordering (plus shift from [-1, 1] to [0, 2] to reflect lower corner
    // origin)
    size_t row_major_index = 1 * static_cast<size_t>(offsets(0, i) + 1) +
                             3 * static_cast<size_t>(offsets(1, i) + 1) +
                             9 * static_cast<size_t>(offsets(2, i) + 1);

    flags_neighborhood.set(i, flags_row_major[26 - row_major_index]);
  }

  flags_neighborhood.set(26, flags_row_major[13]);
  return flags_neighborhood;
}

namespace {

inline bool isValidPoint(const GvdVoxel* voxel, uint8_t min_extra_basis) {
  if (!voxel) {
    return false;
  }
  return voxel->num_extra_basis >= min_extra_basis;
}

}  // namespace

std::bitset<27> extractNeighborhoodFlags(const Layer<GvdVoxel>& layer,
                                         const GlobalIndex& index,
                                         uint8_t min_extra_basis) {
  // TODO(nathan) this is a lot of memory to keep pushing onto the stack
  Neighborhood<>::IndexMatrix neighbor_indices;
  Neighborhood<>::getFromGlobalIndex(index, &neighbor_indices);

  std::bitset<27> neighbor_values;
  for (unsigned int n = 0u; n < neighbor_indices.cols(); ++n) {
    const GvdVoxel* voxel = layer.getVoxelPtrByGlobalIndex(neighbor_indices.col(n));
    neighbor_values.set(n, isValidPoint(voxel, min_extra_basis));
  }

  const GvdVoxel* voxel = layer.getVoxelPtrByGlobalIndex(index);
  neighbor_values.set(26, isValidPoint(voxel, min_extra_basis));

  return neighbor_values;
}

// by default, a template only passes if all points in the 3x3 cube are voronoi
GvdCornerTemplate::GvdCornerTemplate() : fg_mask(0x7FFF'FFFF) {
  unused_mask_array = MaskArray{0, 0, 0, 0};
}

GvdCornerTemplate::GvdCornerTemplate(std::bitset<27> fg_mask_rm,
                                     GvdCornerTemplate::MaskArray unused_mask_rm)
    : fg_mask(convertRowMajorFlags(fg_mask_rm)) {
  for (size_t i = 0; i < 4; ++i) {
    unused_mask_array[i] = convertRowMajorFlags(unused_mask_rm[i]);
  }
}

std::ostream& operator<<(std::ostream& out, const GvdCornerTemplate& corner_template) {
  out << "Foreground / Background mask: " << std::endl;
  out << "  - " << corner_template.fg_mask << std::endl;
  out << "Unused mask: " << std::endl;
  out << "  -   0 degrees: " << corner_template.unused_mask_array[0] << std::endl;
  out << "  -  90 degrees: " << corner_template.unused_mask_array[1] << std::endl;
  out << "  - 180 degrees: " << corner_template.unused_mask_array[2] << std::endl;
  out << "  - 270 degrees: " << corner_template.unused_mask_array[3] << std::endl;
  return out;
}

CornerFinder::CornerFinder() {
  // format for each template is {fg_mask, {0, 90, 180, 270}} where 0, 90, 180, and 270
  // represent rotations of the unused_mask around the current axis of the corresponding
  // amount of degrees (following the right-hand rule around the axis described by the
  // fg_mask template).
  // for the actual bitset values: we follow a row-major layout (matrix "shape" z, y, x)
  // where 9 * z + 3 * y + x gives the index of the bit that describes that voxel. This
  // means that you can read each mask as a group of 9 "vectors" in three groups: the
  // slices z=0, z=1, and z=2 where each vector points along the x-axis. The first
  // "vector" starts at the lower left corner of the 3x3x3 grid and ends at the lower
  // right corner.
  negative_x_template = {0b000'000'000'000'110'000'000'000'000,
                         ///////////  unused masks ////////////
                         {0b000'110'110'000'000'110'000'000'000,
                          0b110'110'000'110'000'000'000'000'000,
                          0b000'000'000'110'000'000'110'110'000,
                          0b000'000'000'000'000'110'000'110'110}};

  positive_x_template = {0b000'000'000'000'011'000'000'000'000,
                         ///////////  unused masks ////////////
                         {0b000'011'011'000'000'011'000'000'000,
                          0b000'000'000'000'000'011'000'011'011,
                          0b000'000'000'011'000'000'011'011'000,
                          0b011'011'000'011'000'000'000'000'000}};

  negative_y_template = {0b000'000'000'010'010'000'000'000'000,
                         ///////////  unused masks ////////////
                         {0b110'110'000'100'100'000'000'000'000,
                          0b011'011'000'001'001'000'000'000'000,
                          0b000'000'000'001'001'000'011'011'000,
                          0b000'000'000'100'100'000'110'110'000}};

  positive_y_template = {0b000'000'000'000'010'010'000'000'000,
                         ///////////  unused masks ////////////
                         {0b000'011'011'000'001'001'000'000'000,
                          0b000'110'110'000'100'100'000'000'000,
                          0b000'000'000'000'100'100'000'110'110,
                          0b000'000'000'000'001'001'000'011'011}};

  negative_z_template = {0b000'010'000'000'010'000'000'000'000,
                         ///////////  unused masks ////////////
                         {0b000'001'011'000'001'011'000'000'000,
                          0b011'001'000'011'001'000'000'000'000,
                          0b110'100'000'110'100'000'000'000'000,
                          0b000'100'110'000'100'110'000'000'000}};

  positive_z_template = {0b000'000'000'000'010'000'000'010'000,
                         ///////////  unused masks ////////////
                         {0b000'000'000'000'100'110'000'100'110,
                          0b000'000'000'110'100'000'110'100'000,
                          0b000'000'000'011'001'000'011'001'000,
                          0b000'000'000'000'001'011'000'001'011}};
}

// implementation loosely based on: https://gist.github.com/yamamushi/5823518
voxblox::AlignedVector<GlobalIndex> makeBresenhamLine(const GlobalIndex& start,
                                                      const GlobalIndex& end) {
  GlobalIndex diff = end - start;
  GlobalIndex inc(diff(0) < 0 ? -1 : 1, diff(1) < 0 ? -1 : 1, diff(2) < 0 ? -1 : 1);

  diff = diff.array().abs();
  const GlobalIndex diff_twice = 2 * diff;

  int max_idx;
  int min_idx_1;
  int min_idx_2;
  if (diff(0) >= diff(1) && diff(0) >= diff(2)) {
    max_idx = 0;
    min_idx_1 = 1;
    min_idx_2 = 2;
  } else if (diff(1) >= diff(0) && diff(1) >= diff(2)) {
    max_idx = 1;
    min_idx_1 = 0;
    min_idx_2 = 2;
  } else {
    max_idx = 2;
    min_idx_1 = 0;
    min_idx_2 = 1;
  }

  if (diff(max_idx) <= 1) {
    return voxblox::AlignedVector<GlobalIndex>();
  }

  voxblox::AlignedVector<GlobalIndex> line_points(diff(max_idx) - 1);
  GlobalIndex point = start;

  int64_t err_1 = diff_twice(min_idx_1) - diff(max_idx);
  int64_t err_2 = diff_twice(min_idx_2) - diff(max_idx);
  for (int64_t i = 0; i < diff(max_idx); ++i) {
    if (i > 0) {
      line_points[i - 1] = point;
    }

    if (err_1 > 0) {
      point(min_idx_1) += inc(min_idx_1);
      err_1 -= diff_twice(max_idx);
    }
    if (err_2 > 0) {
      point(min_idx_2) += inc(min_idx_2);
      err_2 -= diff_twice(max_idx);
    }
    err_1 += diff_twice(min_idx_1);
    err_2 += diff_twice(min_idx_2);
    point[max_idx] += inc(max_idx);
  }

  return line_points;
}

// TODO(nathan) consider removing
double getNeighborhoodOverlap(const SceneGraphLayer& graph,
                              std::unordered_set<NodeId> neighborhood,
                              NodeId other_node,
                              size_t num_hops) {
  std::unordered_set<NodeId> other_neighborhood =
      graph.getNeighborhood(other_node, num_hops);

  size_t num_same = 0;
  for (const auto neighbor : neighborhood) {
    if (other_neighborhood.count(neighbor)) {
      num_same++;
    }
  }

  size_t union_cardinality = neighborhood.size() + other_neighborhood.size() - num_same;

  return static_cast<double>(num_same) / static_cast<double>(union_cardinality);
}

void addFreespaceEdge(SceneGraphLayer& graph,
                      NodeId node,
                      NodeId neighbor,
                      double min_clearance) {
  if (node == neighbor) {
    return;
  }

  if (graph.hasEdge(node, neighbor)) {
    return;
  }

  const double r1 = getNodeGvdDistance(graph, node);
  const double r2 = getNodeGvdDistance(graph, neighbor);
  const double d = (graph.getPosition(node) - graph.getPosition(neighbor)).norm();

  if (d >= r1 + r2) {
    return;
  }

  if (d <= r1 || d <= r2) {
    // intersection is inside one node's sphere
    graph.insertEdge(
        node, neighbor, std::make_unique<EdgeAttributes>(std::min(r1, r2)));
    return;
  }

  // see https://mathworld.wolfram.com/Sphere-SphereIntersection.html
  const double clearance =
      std::sqrt(4 * std::pow(d, 2) * std::pow(r1, 2) -
                std::pow(std::pow(d, 2) - std::pow(r2, 2) + std::pow(r1, 2), 2)) /
      (2 * d);
  if (clearance >= min_clearance) {
    return;
  }

  graph.insertEdge(node, neighbor, std::make_unique<EdgeAttributes>(clearance));
}

}  // namespace topology
}  // namespace hydra
