#include <gtest/gtest.h>

#include <kimera_topology/gvd_utilities.h>

namespace kimera {
namespace topology {

using ParentMap = Eigen::Map<const GlobalIndex>;

struct GvdVoxelWithIndex {
  GvdVoxel voxel;
  GlobalIndex index;
};

GvdVoxelWithIndex makeGvdVoxel(uint64_t x, uint64_t y, uint64_t z) {
  GvdVoxelWithIndex to_return;
  to_return.index << x, y, z;
  return to_return;
}

GvdVoxelWithIndex makeGvdVoxel(uint64_t x, uint64_t y, uint64_t z, float distance) {
  GvdVoxelWithIndex to_return = makeGvdVoxel(x, y, z);
  to_return.voxel.distance = distance;
  to_return.voxel.observed = true;
  return to_return;
}

GvdVoxelWithIndex makeGvdVoxel(uint64_t x,
                               uint64_t y,
                               uint64_t z,
                               float distance,
                               uint64_t px,
                               uint64_t py,
                               uint64_t pz) {
  GvdVoxelWithIndex to_return = makeGvdVoxel(x, y, z, distance);
  to_return.voxel.has_parent = true;
  to_return.voxel.parent[0] = px;
  to_return.voxel.parent[1] = py;
  to_return.voxel.parent[2] = pz;
  return to_return;
}

TEST(GvdUtilities, setGvdParent) {
  {  // assign parent from neighbor parent
    GvdVoxelWithIndex neighbor = makeGvdVoxel(1, 2, 3, 4.0, 5, 6, 7);
    GvdVoxel current;
    EXPECT_FALSE(current.has_parent);

    setGvdParent(current, neighbor.voxel, neighbor.index);

    GlobalIndex expected;
    expected << 5, 6, 7;
    EXPECT_TRUE(current.has_parent);
    EXPECT_EQ(expected, ParentMap(current.parent));
  }

  {  // assign parent from neighbor
    GvdVoxelWithIndex neighbor = makeGvdVoxel(1, 2, 3, 4.0);
    GvdVoxel current;
    EXPECT_FALSE(current.has_parent);

    setGvdParent(current, neighbor.voxel, neighbor.index);

    GlobalIndex expected;
    expected << 1, 2, 3;
    EXPECT_TRUE(current.has_parent);
    EXPECT_EQ(expected, ParentMap(current.parent));
  }
}

TEST(GvdUtilities, ressetGvdParent) {
  GvdVoxelWithIndex current;
  // invariant of new voxels
  EXPECT_FALSE(current.voxel.has_parent);
  // resetting can't assign a new parent
  resetGvdParent(current.voxel);
  EXPECT_FALSE(current.voxel.has_parent);

  current = makeGvdVoxel(1, 2, 3, 4.0, 5, 6, 7);
  EXPECT_TRUE(current.voxel.has_parent);
  // resetting should clear a previous parent
  resetGvdParent(current.voxel);
  EXPECT_FALSE(current.voxel.has_parent);
}

TEST(GvdUtilities, setGvdSurfaceVoxel) {
  {  // assign parent from neighbor parent
    GvdVoxel current;
    EXPECT_FALSE(current.has_parent);
    EXPECT_FALSE(current.on_surface);

    setGvdSurfaceVoxel(current);

    EXPECT_FALSE(current.has_parent);
    EXPECT_TRUE(current.on_surface);
  }

  {  // assign parent from neighbor parent
    GvdVoxel current = makeGvdVoxel(1, 2, 3, 4.0, 5, 6, 7).voxel;
    EXPECT_TRUE(current.has_parent);
    EXPECT_FALSE(current.on_surface);

    setGvdSurfaceVoxel(current);

    EXPECT_FALSE(current.has_parent);
    EXPECT_TRUE(current.on_surface);
  }
}

// test that estimate distance handles all the different
// sign cases correctly
TEST(GvdUtilities, estimateDistanceCorrect) {
  FloatingPoint d = 0.25;  // distance to use

  {  // test case 1: both curr and neighbor are positive
    FloatingPoint v_d = 1.0;
    FloatingPoint v_n = 2.0;
    FloatingPoint v_n_est = getLowerDistance(v_d, v_n, d).distance;
    EXPECT_GT(std::abs(v_n - v_d),
              std::abs(v_n - v_n_est));  // we got closer to the true distance
  }

  {  // test case 2: both curr and neighbor are negative
    FloatingPoint v_d = -1.0;
    FloatingPoint v_n = -2.0;
    FloatingPoint v_n_est = getLowerDistance(v_d, v_n, d).distance;
    EXPECT_GT(std::abs(v_n - v_d),
              std::abs(v_n - v_n_est));  // we got closer to the true distance
  }

  {  // test case 3: curr positive, neighbor negative
    FloatingPoint v_d = 1.0;
    FloatingPoint v_n = -2.0;
    FloatingPoint v_n_est = getLowerDistance(v_d, v_n, d).distance;
    EXPECT_GT(std::abs(v_n - v_d),
              std::abs(v_n - v_n_est));  // we got closer to the true distance
  }

  {  // test case 4: curr negative, neighbor positive
    FloatingPoint v_d = -1.0;
    FloatingPoint v_n = 2.0;
    FloatingPoint v_n_est = getLowerDistance(v_d, v_n, d).distance;
    EXPECT_GT(std::abs(v_n - v_d),
              std::abs(v_n - v_n_est));  // we got closer to the true distance
  }

  {  // test case 5: curr zero, neighbor positive
    FloatingPoint v_d = 0.0;
    FloatingPoint v_n = 2.0;
    FloatingPoint v_n_est = getLowerDistance(v_d, v_n, d).distance;
    EXPECT_GT(std::abs(v_n - v_d),
              std::abs(v_n - v_n_est));  // we got closer to the true distance
  }

  {  // test case 6: curr zero, neighbor positive
    FloatingPoint v_d = 0.0;
    FloatingPoint v_n = -2.0;
    FloatingPoint v_n_est = getLowerDistance(v_d, v_n, d).distance;
    EXPECT_GT(std::abs(v_n - v_d),
              std::abs(v_n - v_n_est));  // we got closer to the true distance
  }

  {  // test case 7: curr negative, neighbor zero
    FloatingPoint v_d = -1.0;
    FloatingPoint v_n = 0.0;
    FloatingPoint v_n_est = getLowerDistance(v_d, v_n, d).distance;
    EXPECT_GT(std::abs(v_n - v_d),
              std::abs(v_n - v_n_est));  // we got closer to the true distance
  }

  {  // test case 8: curr positive, neighbor zero
    FloatingPoint v_d = 1.0;
    FloatingPoint v_n = 0.0;
    FloatingPoint v_n_est = getLowerDistance(v_d, v_n, d).distance;
    EXPECT_GT(std::abs(v_n - v_d),
              std::abs(v_n - v_n_est));  // we got closer to the true distance
  }

  // the last case (that both are zero) is intentionally not handled
}

TEST(GvdUtilities, checkVoronoiMinDistance) {
  {  // current is invalid
    GvdVoxelWithIndex current = makeGvdVoxel(1, 2, 3, 0.05, 0, 0, 0);
    GvdVoxelWithIndex neighbor = makeGvdVoxel(1, 2, 4, 1.0, 5, 5, 5);
    VoronoiCondition result =
        checkVoronoi(current.voxel, current.index, neighbor.voxel, neighbor.index, 0.1);
    EXPECT_FALSE(result.neighbor_is_voronoi);
    EXPECT_FALSE(result.current_is_voronoi);
  }

  {  // neighbor is invalid
    GvdVoxelWithIndex current = makeGvdVoxel(1, 2, 3, 1.0, 0, 0, 0);
    GvdVoxelWithIndex neighbor = makeGvdVoxel(1, 2, 4, 0.05, 5, 5, 5);
    VoronoiCondition result =
        checkVoronoi(current.voxel, current.index, neighbor.voxel, neighbor.index, 0.1);
    EXPECT_FALSE(result.neighbor_is_voronoi);
    EXPECT_FALSE(result.current_is_voronoi);
  }
}

TEST(GvdUtilities, checkVoronoiParentsSame) {
  GvdVoxelWithIndex current = makeGvdVoxel(1, 2, 3, 1.0, 0, 0, 0);
  GvdVoxelWithIndex neighbor = makeGvdVoxel(1, 2, 4, 1.0, 0, 0, 0);
  VoronoiCondition result =
      checkVoronoi(current.voxel, current.index, neighbor.voxel, neighbor.index, 0.1);
  EXPECT_FALSE(result.neighbor_is_voronoi);
  EXPECT_FALSE(result.current_is_voronoi);
}

TEST(GvdUtilities, checkVoronoiParentNeighbors) {
  {  // neighbor are 1 step away
    GvdVoxelWithIndex current = makeGvdVoxel(1, 2, 3, 1.0, 0, 0, 0);
    GvdVoxelWithIndex neighbor = makeGvdVoxel(1, 2, 4, 1.0, 0, 0, 1);
    VoronoiCondition result =
        checkVoronoi(current.voxel, current.index, neighbor.voxel, neighbor.index, 0.1);
    EXPECT_FALSE(result.neighbor_is_voronoi);
    EXPECT_FALSE(result.current_is_voronoi);
  }

  {  // neighbor are 8-connected
    GvdVoxelWithIndex current = makeGvdVoxel(1, 2, 3, 1.0, 0, 0, 0);
    GvdVoxelWithIndex neighbor = makeGvdVoxel(1, 2, 4, 1.0, 0, 1, 1);
    VoronoiCondition result =
        checkVoronoi(current.voxel, current.index, neighbor.voxel, neighbor.index, 0.1);
    EXPECT_FALSE(result.neighbor_is_voronoi);
    EXPECT_FALSE(result.current_is_voronoi);
  }

  {  // neighbor are 28-connected
    GvdVoxelWithIndex current = makeGvdVoxel(1, 2, 3, 1.0, 0, 0, 0);
    GvdVoxelWithIndex neighbor = makeGvdVoxel(1, 2, 4, 1.0, 1, 1, 1);
    VoronoiCondition result =
        checkVoronoi(current.voxel, current.index, neighbor.voxel, neighbor.index, 0.1);
    EXPECT_FALSE(result.neighbor_is_voronoi);
    EXPECT_FALSE(result.current_is_voronoi);
  }

  {  // neighbor are not connected
    GvdVoxelWithIndex current = makeGvdVoxel(1, 2, 3, 1.0, 1, 0, 3);
    GvdVoxelWithIndex neighbor = makeGvdVoxel(1, 2, 4, 1.0, 3, 2, 4);
    VoronoiCondition result =
        checkVoronoi(current.voxel, current.index, neighbor.voxel, neighbor.index, 0.1);
    EXPECT_TRUE(result.neighbor_is_voronoi);
    EXPECT_TRUE(result.current_is_voronoi);
  }
}

}  // namespace topology
}  // namespace kimera
