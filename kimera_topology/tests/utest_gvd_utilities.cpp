#include <gtest/gtest.h>

#include <kimera_topology/gvd_utilities.h>

namespace kimera {
namespace topology {

struct GvdVoxelIndex {
  GvdVoxel voxel;
  GlobalIndex index;
};

GvdVoxelIndex makeGvdVoxel(uint64_t x, uint64_t y, uint64_t z) {
  GvdVoxelIndex to_return;
  to_return.index << x, y, z;
  return to_return;
}

GvdVoxelIndex makeGvdVoxel(uint64_t x, uint64_t y, uint64_t z, double distance) {
  GvdVoxelIndex to_return = makeGvdVoxel(x, y, z);
  to_return.index << x, y, z;
  to_return.voxel.distance = distance;
  to_return.voxel.observed = true;
  return to_return;
}

GvdVoxelIndex makeGvdVoxel(uint64_t x,
                           uint64_t y,
                           uint64_t z,
                           double distance,
                           uint64_t px,
                           uint64_t py,
                           uint64_t pz) {
  GvdVoxelIndex to_return = makeGvdVoxel(x, y, z, distance);
  to_return.voxel.has_parent = true;
  to_return.voxel.parent[0] = px;
  to_return.voxel.parent[1] = py;
  to_return.voxel.parent[2] = pz;
  return to_return;
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

TEST(GvdUtilities, DISABLED_checkVoronoiNoParent) {
  {  // current is invalid
    GvdVoxelIndex current = makeGvdVoxel(1, 2, 3, 0.05, 0, 0, 0);
    GvdVoxelIndex neighbor = makeGvdVoxel(1, 2, 4, 1.0, 5, 5, 5);
    VoronoiCondition result =
        checkVoronoi(current.voxel, current.index, neighbor.voxel, neighbor.index, 0.1);
    EXPECT_FALSE(result.neighbor_is_voronoi);
    EXPECT_FALSE(result.current_is_voronoi);
  }

  {  // neighbor is invalid
    GvdVoxelIndex current = makeGvdVoxel(1, 2, 3, 1.0, 0, 0, 0);
    GvdVoxelIndex neighbor = makeGvdVoxel(1, 2, 4, 0.05, 5, 5, 5);
    VoronoiCondition result =
        checkVoronoi(current.voxel, current.index, neighbor.voxel, neighbor.index, 0.1);
    EXPECT_FALSE(result.neighbor_is_voronoi);
    EXPECT_FALSE(result.current_is_voronoi);
  }
}

TEST(GvdUtilities, DISABLED_checkVoronoiMinDistance) {
  {  // current is invalid
    GvdVoxelIndex current = makeGvdVoxel(1, 2, 3, 0.05, 0, 0, 0);
    GvdVoxelIndex neighbor = makeGvdVoxel(1, 2, 4, 1.0, 5, 5, 5);
    VoronoiCondition result =
        checkVoronoi(current.voxel, current.index, neighbor.voxel, neighbor.index, 0.1);
    EXPECT_FALSE(result.neighbor_is_voronoi);
    EXPECT_FALSE(result.current_is_voronoi);
  }

  {  // neighbor is invalid
    GvdVoxelIndex current = makeGvdVoxel(1, 2, 3, 1.0, 0, 0, 0);
    GvdVoxelIndex neighbor = makeGvdVoxel(1, 2, 4, 0.05, 5, 5, 5);
    VoronoiCondition result =
        checkVoronoi(current.voxel, current.index, neighbor.voxel, neighbor.index, 0.1);
    EXPECT_FALSE(result.neighbor_is_voronoi);
    EXPECT_FALSE(result.current_is_voronoi);
  }
}

}  // namespace topology
}  // namespace kimera
