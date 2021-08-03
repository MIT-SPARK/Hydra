#pragma once
#include "kimera_topology/gvd_voxel.h"
#include "kimera_topology/voxblox_types.h"

#include <kimera_dsg/scene_graph_layer.h>
#include <kimera_dsg/node_attributes.h>

#include <bitset>
#include <iostream>

namespace kimera {
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

void addFreespaceEdge(SceneGraphLayer& graph, NodeId node, NodeId neighbor, double min_clearance);

}  // namespace topology
}  // namespace kimera
