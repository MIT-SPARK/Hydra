#include <gtest/gtest.h>

#include <kimera_topology/gvd_utilities.h>

namespace kimera {
namespace topology {

using ParentMap = Eigen::Map<const GlobalIndex>;
using ParentPosMap = Eigen::Map<const voxblox::Point>;

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
  to_return.voxel.parent_pos[0] = px;
  to_return.voxel.parent_pos[1] = py;
  to_return.voxel.parent_pos[2] = pz;
  return to_return;
}

VoronoiCheckConfig makeL1Config(double min_distance_m = 0.1,
                                double min_l1_separation = 3.0) {
  VoronoiCheckConfig config;
  config.mode = ParentUniquenessMode::L1_DISTANCE;
  config.min_distance_m = min_distance_m;
  config.parent_l1_separation = min_l1_separation;
  return config;
}

VoronoiCheckConfig makeAngleConfig(double min_distance_m = 0.1,
                                   double max_cos_separation = 0.5) {
  VoronoiCheckConfig config;
  config.mode = ParentUniquenessMode::ANGLE;
  config.min_distance_m = min_distance_m;
  config.parent_cos_angle_separation = max_cos_separation;
  return config;
}

VoronoiCheckConfig makeL1AndAngleConfig(double min_distance_m = 0.1,
                                        double min_l1_separation = 3.0,
                                        double max_cos_separation = 0.5) {
  VoronoiCheckConfig config;
  config.mode = ParentUniquenessMode::L1_THEN_ANGLE;
  config.min_distance_m = min_distance_m;
  config.parent_l1_separation = min_l1_separation;
  config.parent_cos_angle_separation = max_cos_separation;
  return config;
}

TEST(GvdUtilities, setSdfParent) {
  {  // assign parent from neighbor parent
    GvdVoxelWithIndex neighbor = makeGvdVoxel(1, 2, 3, 4.0, 5, 6, 7);
    voxblox::Point neighbor_pos(1.0, 2.0, 3.0);
    GvdVoxel current;
    EXPECT_FALSE(current.has_parent);

    setSdfParent(current, neighbor.voxel, neighbor.index, neighbor_pos);

    GlobalIndex expected;
    expected << 5, 6, 7;
    voxblox::Point expected_pos;
    expected_pos << 5.0f, 6.0f, 7.0f;
    EXPECT_TRUE(current.has_parent);
    EXPECT_EQ(expected, ParentMap(current.parent));
    EXPECT_EQ(expected_pos, ParentPosMap(current.parent_pos));
  }

  {  // assign parent from neighbor
    GvdVoxelWithIndex neighbor = makeGvdVoxel(1, 2, 3, 4.0);
    voxblox::Point neighbor_pos(1.0f, 2.0f, 3.0f);
    GvdVoxel current;
    EXPECT_FALSE(current.has_parent);

    setSdfParent(current, neighbor.voxel, neighbor.index, neighbor_pos);

    GlobalIndex expected(1, 2, 3);
    EXPECT_TRUE(current.has_parent);
    EXPECT_EQ(expected, ParentMap(current.parent));
    EXPECT_EQ(neighbor_pos, ParentPosMap(current.parent_pos));
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
    VoronoiCondition result = checkVoronoi(
        makeL1Config(), current.voxel, current.index, neighbor.voxel, neighbor.index);
    EXPECT_FALSE(result.neighbor_is_voronoi);
    EXPECT_FALSE(result.current_is_voronoi);
  }

  {  // neighbor is invalid
    GvdVoxelWithIndex current = makeGvdVoxel(1, 2, 3, 1.0, 0, 0, 0);
    GvdVoxelWithIndex neighbor = makeGvdVoxel(1, 2, 4, 0.05, 5, 5, 5);
    VoronoiCondition result = checkVoronoi(
        makeL1Config(), current.voxel, current.index, neighbor.voxel, neighbor.index);
    EXPECT_FALSE(result.neighbor_is_voronoi);
    EXPECT_FALSE(result.current_is_voronoi);
  }
}

TEST(GvdUtilities, checkVoronoiParentsSame) {
  GvdVoxelWithIndex current = makeGvdVoxel(1, 2, 3, 1.0, 0, 0, 0);
  GvdVoxelWithIndex neighbor = makeGvdVoxel(1, 2, 4, 1.0, 0, 0, 0);
  VoronoiCondition result = checkVoronoi(
      makeL1Config(), current.voxel, current.index, neighbor.voxel, neighbor.index);
  EXPECT_FALSE(result.neighbor_is_voronoi);
  EXPECT_FALSE(result.current_is_voronoi);
}

TEST(GvdUtilities, checkVoronoiParentNeighbors) {
  {  // neighbor are 1 step away
    GvdVoxelWithIndex current = makeGvdVoxel(1, 2, 3, 1.0, 0, 0, 0);
    GvdVoxelWithIndex neighbor = makeGvdVoxel(1, 2, 4, 1.0, 0, 0, 1);
    VoronoiCondition result = checkVoronoi(
        makeL1Config(), current.voxel, current.index, neighbor.voxel, neighbor.index);
    EXPECT_FALSE(result.neighbor_is_voronoi);
    EXPECT_FALSE(result.current_is_voronoi);
  }

  {  // neighbor are 8-connected
    GvdVoxelWithIndex current = makeGvdVoxel(1, 2, 3, 1.0, 0, 0, 0);
    GvdVoxelWithIndex neighbor = makeGvdVoxel(1, 2, 4, 1.0, 0, 1, 1);
    VoronoiCondition result = checkVoronoi(
        makeL1Config(), current.voxel, current.index, neighbor.voxel, neighbor.index);
    EXPECT_FALSE(result.neighbor_is_voronoi);
    EXPECT_FALSE(result.current_is_voronoi);
  }

  {  // neighbor are 28-connected
    GvdVoxelWithIndex current = makeGvdVoxel(1, 2, 3, 1.0, 0, 0, 0);
    GvdVoxelWithIndex neighbor = makeGvdVoxel(1, 2, 4, 1.0, 1, 1, 1);
    VoronoiCondition result = checkVoronoi(
        makeL1Config(), current.voxel, current.index, neighbor.voxel, neighbor.index);
    EXPECT_FALSE(result.neighbor_is_voronoi);
    EXPECT_FALSE(result.current_is_voronoi);
  }

  {  // neighbor are not connected
    GvdVoxelWithIndex current = makeGvdVoxel(1, 2, 3, 1.0, 1, 0, 3);
    GvdVoxelWithIndex neighbor = makeGvdVoxel(1, 2, 4, 1.0, 3, 2, 4);
    VoronoiCondition result = checkVoronoi(
        makeL1Config(), current.voxel, current.index, neighbor.voxel, neighbor.index);
    EXPECT_TRUE(result.neighbor_is_voronoi);
    EXPECT_TRUE(result.current_is_voronoi);
  }
}

TEST(GvdUtilities, checkVoronoiParentSeparation) {
  {  // test that l1 separation rejects parents that are too close
    GlobalIndex voxel1(1, 2, 3);
    GlobalIndex parent1(5, 6, 7);
    GlobalIndex parent2(5, 7, 8);
    EXPECT_FALSE(isParentUnique(makeL1Config(), voxel1, parent1, parent2));

    // l1 separation is independent of current voxel
    GlobalIndex voxel2(9, 10, -1);
    EXPECT_FALSE(isParentUnique(makeL1Config(), voxel2, parent1, parent2));
  }

  {  // test that l1 separation accepts parents far enough away
    GlobalIndex voxel1(1, 2, 3);
    GlobalIndex parent1(0, -2, 7);
    GlobalIndex parent2(5, 7, 8);
    EXPECT_TRUE(isParentUnique(makeL1Config(), voxel1, parent1, parent2));

    // l1 separation is independent of current voxel
    GlobalIndex voxel2(9, 10, -1);
    EXPECT_TRUE(isParentUnique(makeL1Config(), voxel2, parent1, parent2));
  }

  {  // test that angle separation works as expected
    GlobalIndex voxel1(1, 2, 3);
    GlobalIndex parent1(5, 6, 7);
    GlobalIndex parent2(5, 7, 10);
    EXPECT_FALSE(isParentUnique(makeAngleConfig(), voxel1, parent1, parent2));

    // test another known bad case
    GlobalIndex voxel2(0, 6, 8);
    EXPECT_FALSE(isParentUnique(makeAngleConfig(), voxel2, parent1, parent2));

    // test a positive, but good case
    GlobalIndex voxel3(3, 6, 8);
    EXPECT_TRUE(isParentUnique(makeAngleConfig(), voxel3, parent1, parent2));

    // angle separation works for dot products less than 0
    GlobalIndex voxel4(5, 6, 8);
    EXPECT_TRUE(isParentUnique(makeAngleConfig(), voxel4, parent1, parent2));
  }

  {  // test that cascaded checks work as expected
    GlobalIndex voxel1(1, 2, 3);
    GlobalIndex parent1(5, 6, 6);
    GlobalIndex parent2(5, 7, 10);
    EXPECT_FALSE(
        isParentUnique(makeL1AndAngleConfig(0.1, 10.0), voxel1, parent1, parent2));

    // test a previously bad angle
    GlobalIndex voxel2(0, 6, 8);
    EXPECT_FALSE(isParentUnique(makeAngleConfig(), voxel2, parent1, parent2));
    EXPECT_TRUE(isParentUnique(makeL1Config(), voxel2, parent1, parent2));
    EXPECT_TRUE(isParentUnique(makeL1AndAngleConfig(), voxel2, parent1, parent2));

    // test a bad distance
    parent1 << 5, 7, 7;
    GlobalIndex voxel3(5, 6, 8);
    EXPECT_FALSE(isParentUnique(makeL1Config(), voxel3, parent1, parent2));
    EXPECT_TRUE(isParentUnique(makeAngleConfig(), voxel3, parent1, parent2));
    EXPECT_TRUE(isParentUnique(makeL1AndAngleConfig(), voxel3, parent1, parent2));
  }
}

}  // namespace topology
}  // namespace kimera
