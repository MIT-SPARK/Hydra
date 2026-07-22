#include <gtest/gtest.h>

#include <array>
#include <vector>

#include "hydra/places/region_growing_traversability_clustering.h"

namespace hydra::places {

// Test shim: expose the protected static helpers.
struct RegionGrowingTest : public RegionGrowingTraversabilityClustering {
  using RegionGrowingTraversabilityClustering::RegionGrowingTraversabilityClustering;
  using RegionGrowingTraversabilityClustering::growRegion;
};

using VoxelSet = RegionGrowingTraversabilityClustering::VoxelSet;

namespace {
VoxelSet makeSet(const std::vector<std::array<int, 2>>& pts) {
  VoxelSet s;
  for (const auto& p : pts) {
    s.insert(VoxelIndex(p[0], p[1], 0));
  }
  return s;
}
}  // namespace

TEST(RegionGrowingConnectivity, DiagonalGapBlockedBy4Connectivity) {
  // Two 1-voxel rooms touching only diagonally: (0,0) and (1,1).
  const VoxelSet candidates = makeSet({{0, 0}, {1, 1}});

  // 8-connected: diagonal counts -> both reachable.
  const auto c8 = RegionGrowingTest::growRegion(candidates, VoxelIndex(0, 0, 0), 8u);
  EXPECT_EQ(c8.size(), 2u);

  // 4-connected: diagonal does NOT count -> only the seed room.
  const auto c4 = RegionGrowingTest::growRegion(candidates, VoxelIndex(0, 0, 0), 4u);
  EXPECT_EQ(c4.size(), 1u);
  EXPECT_TRUE(c4.count(VoxelIndex(0, 0, 0)));
  EXPECT_FALSE(c4.count(VoxelIndex(1, 1, 0)));
}

}  // namespace hydra::places
