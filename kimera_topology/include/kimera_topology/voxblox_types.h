#pragma once
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <voxblox/mesh/mesh.h>
#include <voxblox/mesh/mesh_layer.h>
#include <voxblox/utils/bucket_queue.h>
#include <voxblox/utils/neighbor_tools.h>
#include <voxblox/integrator/integrator_utils.h>

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
using voxblox::Mesh;
using voxblox::MeshLayer;
using voxblox::Neighborhood;
using voxblox::NeighborhoodLookupTables;
using voxblox::SignedIndex;
using voxblox::TsdfVoxel;
using voxblox::VertexIndex;
using voxblox::VoxelIndex;
using voxblox::ThreadSafeIndex;

}  // namespace topology
}  // namespace kimera
