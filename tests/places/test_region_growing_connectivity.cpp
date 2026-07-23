#include <gtest/gtest.h>

#include <array>
#include <vector>

#include "hydra/places/region_growing_traversability_clustering.h"

namespace hydra::places {

// Test shim: expose the protected static helpers.
struct RegionGrowingTest : public RegionGrowingTraversabilityClustering {
  using RegionGrowingTraversabilityClustering::erodeCandidates;
  using RegionGrowingTraversabilityClustering::growConnectedWithMinWidth;
  using RegionGrowingTraversabilityClustering::growRegion;
  using RegionGrowingTraversabilityClustering::RegionGrowingTraversabilityClustering;
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

namespace {
VoxelSet makeRoom(int x0, int x1, int y0, int y1) {
  VoxelSet s;
  for (int x = x0; x <= x1; ++x) {
    for (int y = y0; y <= y1; ++y) {
      s.insert(VoxelIndex(x, y, 0));
    }
  }
  return s;
}
}  // namespace

TEST(RegionGrowingConnectivity, MinWidthSeversThinBridgeKeepsRoom) {
  // Room A [0..2]x[0..2], Room B [4..6]x[0..2], joined by a 1-voxel bridge (3,1).
  VoxelSet candidates = makeRoom(0, 2, 0, 2);
  for (const auto& v : makeRoom(4, 6, 0, 2)) candidates.insert(v);
  candidates.insert(VoxelIndex(3, 1, 0));

  const int radius = 1;  // min_connection_width_voxels = 2 -> radius = 1
  const VoxelSet core =
      RegionGrowingTest::erodeCandidates(candidates, radius, /*use_diagonal=*/false);
  EXPECT_FALSE(core.count(VoxelIndex(3, 1, 0)));  // 1-wide bridge eroded away
  EXPECT_TRUE(core.count(VoxelIndex(1, 1, 0)));   // room-A interior survives

  const auto result = RegionGrowingTest::growConnectedWithMinWidth(
      candidates, core, VoxelIndex(1, 1, 0), 4u);
  // Room B is unreachable across the thin bridge.
  EXPECT_FALSE(result.count(VoxelIndex(4, 1, 0)));
  EXPECT_FALSE(result.count(VoxelIndex(5, 1, 0)));
  // Room A preserved (center + edges reachable through core).
  EXPECT_TRUE(result.count(VoxelIndex(1, 1, 0)));
  EXPECT_TRUE(result.count(VoxelIndex(2, 1, 0)));
  EXPECT_TRUE(result.count(VoxelIndex(0, 1, 0)));
}

TEST(RegionGrowingConnectivity, ErosionStructuringElementFollowsConnectivity) {
  // Full 3x3 block minus one diagonal corner (1,1). The center (0,0) has all 4
  // orthogonal neighbors present but is missing a diagonal neighbor.
  VoxelSet candidates = makeRoom(-1, 1, -1, 1);
  candidates.erase(VoxelIndex(1, 1, 0));

  // 4-connected (plus) erosion: only orthogonal neighbors checked -> center survives.
  const auto plus =
      RegionGrowingTest::erodeCandidates(candidates, 1, /*use_diagonal=*/false);
  EXPECT_TRUE(plus.count(VoxelIndex(0, 0, 0)));

  // 8-connected (square) erosion: the missing diagonal (1,1) disqualifies the center.
  const auto square =
      RegionGrowingTest::erodeCandidates(candidates, 1, /*use_diagonal=*/true);
  EXPECT_FALSE(square.count(VoxelIndex(0, 0, 0)));
}

TEST(RegionGrowingConnectivity, WideDoorwayConnects) {
  // Same rooms but a full 3-wide doorway at x=3 (y=0,1,2).
  VoxelSet candidates = makeRoom(0, 2, 0, 2);
  for (const auto& v : makeRoom(4, 6, 0, 2)) candidates.insert(v);
  for (int y = 0; y <= 2; ++y) candidates.insert(VoxelIndex(3, y, 0));

  const int radius = 1;
  const VoxelSet core =
      RegionGrowingTest::erodeCandidates(candidates, radius, /*use_diagonal=*/false);
  EXPECT_TRUE(core.count(VoxelIndex(3, 1, 0)));  // wide doorway survives erosion

  const auto result = RegionGrowingTest::growConnectedWithMinWidth(
      candidates, core, VoxelIndex(1, 1, 0), 4u);
  EXPECT_TRUE(result.count(VoxelIndex(5, 1, 0)));  // room B reached via doorway
}

}  // namespace hydra::places
