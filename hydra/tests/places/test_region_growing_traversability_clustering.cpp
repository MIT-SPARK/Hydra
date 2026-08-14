#include <gtest/gtest.h>

#include <array>
#include <vector>

#include "hydra/places/region_growing_traversability_clustering.h"

namespace hydra::places {

using VoxelSet = RegionGrowingTraversabilityClustering::VoxelSet;
using Clustering = RegionGrowingTraversabilityClustering;

namespace {

VoxelSet makeSet(const std::vector<std::array<int, 2>>& pts) {
  VoxelSet s;
  for (const auto& p : pts) {
    s.insert(VoxelIndex(p[0], p[1], 0));
  }

  return s;
}

}  // namespace

TEST(RegionGrowingTraversabilityClustering, DiagonalGapBlockedBy4Connectivity) {
  const auto candidates = makeSet({{0, 0}, {1, 1}});

  // 8-connected means single cluster
  const auto c8 = Clustering::growRegion(candidates, VoxelIndex(0, 0, 0), 8u);
  EXPECT_EQ(c8.size(), 2u);

  // 4-connected means two cluster
  const auto c4 = Clustering::growRegion(candidates, VoxelIndex(0, 0, 0), 4u);
  EXPECT_EQ(c4.size(), 1u);
  EXPECT_TRUE(c4.count(VoxelIndex(0, 0, 0)));
  EXPECT_FALSE(c4.count(VoxelIndex(1, 1, 0)));
}

}  // namespace hydra::places
