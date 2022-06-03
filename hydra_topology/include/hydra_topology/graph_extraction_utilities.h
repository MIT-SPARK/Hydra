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
#include "hydra_topology/gvd_voxel.h"
#include "hydra_topology/voxblox_types.h"

#include <hydra_utils/dsg_types.h>

#include <bitset>
#include <iostream>

namespace hydra {
namespace topology {

std::bitset<27> convertRowMajorFlags(std::bitset<27> flags_row_major);

std::bitset<27> extractNeighborhoodFlags(const Layer<GvdVoxel>& layer,
                                         const GlobalIndex& index,
                                         uint8_t min_extra_basis = 1);

struct GvdCornerTemplate {
  using MaskArray = std::array<std::bitset<27>, 4>;

  GvdCornerTemplate();

  GvdCornerTemplate(std::bitset<27> fg_mask_row_major,
                    MaskArray unused_mask_array_row_major);

  std::bitset<27> fg_mask;
  MaskArray unused_mask_array;
};

std::ostream& operator<<(std::ostream& out, const GvdCornerTemplate& corner_template);

inline bool matchesCorner(const GvdCornerTemplate& corner_template,
                          std::bitset<27> state) {
  // this computes a bitset where every voxel that matches the status flag is 1
  // this is equivalent to the bitwise and (to get gvd voxels) and the bitwise not-or
  // (to get non-gvd voxels)
  std::bitset<27> intermediate =
      (state & corner_template.fg_mask) | ~(state | corner_template.fg_mask);
  // flags we don't care about get bitwise-or'd and then each result is checked
  return (intermediate | corner_template.unused_mask_array[0]).all() ||
         (intermediate | corner_template.unused_mask_array[1]).all() ||
         (intermediate | corner_template.unused_mask_array[2]).all() ||
         (intermediate | corner_template.unused_mask_array[3]).all();
}

struct CornerFinder {
  CornerFinder();

  ~CornerFinder() = default;

  inline bool match(std::bitset<27> values) const {
    return matchesCorner(negative_x_template, values) ||
           matchesCorner(positive_x_template, values) ||
           matchesCorner(negative_y_template, values) ||
           matchesCorner(positive_y_template, values) ||
           matchesCorner(negative_z_template, values) ||
           matchesCorner(positive_z_template, values);
  }

  GvdCornerTemplate negative_x_template;
  GvdCornerTemplate positive_x_template;
  GvdCornerTemplate negative_y_template;
  GvdCornerTemplate positive_y_template;
  GvdCornerTemplate negative_z_template;
  GvdCornerTemplate positive_z_template;
};

voxblox::AlignedVector<GlobalIndex> makeBresenhamLine(const GlobalIndex& start,
                                                      const GlobalIndex& end);

double getNeighborhoodOverlap(const SceneGraphLayer& graph,
                              std::unordered_set<NodeId> neighborhood,
                              NodeId other_node,
                              size_t num_hops);

inline double getNodeGvdDistance(const SceneGraphLayer& graph, NodeId node) {
  return graph.getNode(node).value().get().attributes<PlaceNodeAttributes>().distance;
}

void addFreespaceEdge(SceneGraphLayer& graph,
                      NodeId node,
                      NodeId neighbor,
                      double min_clearance);

}  // namespace topology
}  // namespace hydra
