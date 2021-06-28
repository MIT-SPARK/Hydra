#pragma once
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <voxblox/utils/bucket_queue.h>
#include <voxblox/utils/neighbor_tools.h>
#include <iostream>

namespace kimera {
namespace topology {

using voxblox::AlignedQueue;
using voxblox::Block;
using voxblox::BlockIndex;
using voxblox::BlockIndexList;
using voxblox::BucketQueue;
using voxblox::FloatingPoint;
using voxblox::GlobalIndex;
using voxblox::IndexSet;
using voxblox::Layer;
using voxblox::Neighborhood;
using voxblox::NeighborhoodLookupTables;
using voxblox::SignedIndex;
using voxblox::TsdfVoxel;

// TODO(nathan) packed?
// TODO(nathan) consider parent pointers
struct GvdVoxel {
  float distance;
  bool observed = false;
  bool in_queue = false;
  bool fixed = false;

  bool has_parent = false;
  GlobalIndex::Scalar parent[3];

  bool is_voronoi = false;
  bool is_voronoi_parent = false;
  uint8_t num_basis = 0;
  GlobalIndex::Scalar nearest_voronoi[3];
  float nearest_voronoi_distance;
};

std::ostream& operator<<(std::ostream& out, const GvdVoxel& voxel);

}  // namespace topology
}  // namespace kimera
