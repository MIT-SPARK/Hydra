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
#include "hydra_topology_test/test_fixtures.h"

#include <gtest/gtest.h>

#include <hydra_topology/graph_extraction_utilities.h>
#include <hydra_topology/voxblox_types.h>

namespace hydra {
namespace topology {

using test_helpers::SingleBlockExtractionTestFixture;
using voxblox::Connectivity;
using voxblox::IndexElement;

using Neighborhood26Connected = Neighborhood<Connectivity::kTwentySix>;
using IndexOffsets26Connected = Neighborhood<Connectivity::kTwentySix>::IndexOffsets;
using IndexRotation = Eigen::Matrix<IndexElement, 3, 3>;

IndexRotation rotX() {
  IndexRotation to_return;
  to_return << 1, 0, 0, 0, 0, -1, 0, 1, 0;
  return to_return;
}

IndexRotation rotY() {
  IndexRotation to_return;
  to_return << 0, 0, 1, 0, 1, 0, -1, 0, 0;
  return to_return;
}

IndexRotation rotZ() {
  IndexRotation to_return;
  to_return << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  return to_return;
}

std::bitset<27> rotateCorner(const IndexRotation& rotation, std::bitset<27> values) {
  const IndexOffsets26Connected& offsets = Neighborhood26Connected::kOffsets;

  std::bitset<27> new_values(0);
  new_values.set(26, values[26]);
  for (int i = 0; i < offsets.cols(); ++i) {
    voxblox::AnyIndex rotated = rotation * offsets.block<3, 1>(0, i);

    for (int j = 0; j < offsets.cols(); ++j) {
      if (rotated == offsets.block<3, 1>(0, j)) {
        new_values[i] = values[j];
        break;
      }
    }
  }
  return new_values;
}

std::bitset<27> getCorner(uint8_t unused_combination_index,
                          bool perturb_corner = false,
                          uint8_t pertubation_index = 0) {
  // this lets us iterate through all combinations of the unused voxels in a linear /
  // flattened fasion
  std::bitset<6> unused_values{unused_combination_index};
  std::bitset<27> corner_values;
  corner_values.set(13, true);  // center is always a voronoi cell
  corner_values.set(12, true);  // negative-x
  corner_values.set(3, unused_values[0]);
  corner_values.set(4, unused_values[1]);
  corner_values.set(6, unused_values[2]);
  corner_values.set(7, unused_values[3]);
  corner_values.set(15, unused_values[4]);
  corner_values.set(16, unused_values[5]);
  if (perturb_corner) {
    // getting the template to fail is trickier. setting the "don't care" points to be
    // members of the GVD ensures that we fail when toggling the exterior non-GVD
    // points, and toggling the corner center always causes the corner template to fail.
    std::array<size_t, 20> non_ignored{0,  1,  2,  5,  8,  9,  10, 11, 13, 14,
                                       17, 18, 19, 20, 21, 22, 23, 24, 25, 26};

    size_t to_perturb = non_ignored[pertubation_index];
    corner_values.set(to_perturb, ~corner_values[to_perturb]);
  }
  return convertRowMajorFlags(corner_values);
}

TEST(GraphExtractionUtilities, RowMajorConversion) {
  {  // 0 -> 0
    std::bitset<27> row_major{0x0000'0000};
    std::bitset<27> expected{0x0000'0000};
    std::bitset<27> result = convertRowMajorFlags(row_major);
    EXPECT_EQ(expected, result);
  }

  {  // 1 -> 1
    std::bitset<27> row_major{0x7FFF'FFFF};
    std::bitset<27> expected{0x7FFF'FFFF};
    std::bitset<27> result = convertRowMajorFlags(row_major);
    EXPECT_EQ(expected, result);
  }

  {  // single-bit correctness
    // derived by drawing 3x3 grid and examining the voxblox offset table
    std::map<size_t, size_t> voxblox_to_row_major{
        {0, 14},  {1, 12},  {2, 16}, {3, 10},  {4, 22},  {5, 4},   {6, 17},
        {7, 11},  {8, 15},  {9, 9},  {10, 25}, {11, 7},  {12, 19}, {13, 1},
        {14, 23}, {15, 21}, {16, 5}, {17, 3},  {18, 26}, {19, 8},  {20, 20},
        {21, 2},  {22, 24}, {23, 6}, {24, 18}, {25, 0}};

    std::set<size_t> voxblox_indices;
    std::set<size_t> row_major_indices;
    for (const auto& index_pair : voxblox_to_row_major) {
      voxblox_indices.insert(index_pair.first);
      row_major_indices.insert(index_pair.second);
    }

    // we should have a bijective index mapping {0, 26} -> {0, 27} \ {13}
    ASSERT_EQ(26u, voxblox_indices.size());
    ASSERT_EQ(0u, *std::min_element(voxblox_indices.begin(), voxblox_indices.end()));
    ASSERT_EQ(25u, *std::max_element(voxblox_indices.begin(), voxblox_indices.end()));
    ASSERT_EQ(26u, row_major_indices.size());
    ASSERT_EQ(0u,
              *std::min_element(row_major_indices.begin(), row_major_indices.end()));
    ASSERT_EQ(26u,
              *std::max_element(row_major_indices.begin(), row_major_indices.end()));
    ASSERT_EQ(0u, row_major_indices.count(13));

    for (const auto& index_pair : voxblox_to_row_major) {
      std::bitset<27> row_major{0};
      row_major.set(index_pair.second, true);
      row_major.set(13, true);

      std::bitset<27> expected{0};
      expected.set(index_pair.first, true);
      expected.set(26, true);

      std::bitset<27> result = convertRowMajorFlags(row_major);
      EXPECT_EQ(expected, result);
    }
  }
}

TEST_F(SingleBlockExtractionTestFixture, NeighborhoodExtraction) {
  {  // outside any allocated voxels -> nothing should be in the GVD
    GlobalIndex index;
    index << 10, 12, 15;
    std::bitset<27> result = extractNeighborhoodFlags(*gvd_layer, index);
    EXPECT_TRUE(result.none());
  }

  {  // origin of block -> still no gvd voxels touched
    GlobalIndex index;
    index << 0, 0, 0;
    std::bitset<27> expected_row_major{0};
    std::bitset<27> expected = convertRowMajorFlags(expected_row_major);
    std::bitset<27> result = extractNeighborhoodFlags(*gvd_layer, index);
    EXPECT_EQ(expected, result);
  }

  {  // upper right corner is gvd
    GlobalIndex index;
    index << 1, 1, 1;
    std::bitset<27> expected_row_major = 0b00'000'000'000'000'000'000'000'001;
    std::bitset<27> expected = convertRowMajorFlags(expected_row_major);
    std::bitset<27> result = extractNeighborhoodFlags(*gvd_layer, index);
    EXPECT_EQ(expected, result);
  }

  {  // lowest gvd corner is center
    GlobalIndex index;
    index << 2, 2, 2;
    std::bitset<27> expected_row_major = 0b000'000'000'000'011'011'000'011'011;
    std::bitset<27> expected = convertRowMajorFlags(expected_row_major);
    std::bitset<27> result = extractNeighborhoodFlags(*gvd_layer, index);
    EXPECT_EQ(expected, result);
  }
}

#define CHECK_TEMPLATE_SOUNDNESS(finder, template_name)                    \
  EXPECT_EQ(2u, finder.template_name.fg_mask.count()) << #template_name;   \
  for (const auto& unused_mask : finder.template_name.unused_mask_array) { \
    EXPECT_EQ(6u, unused_mask.count()) << #template_name;                  \
    EXPECT_TRUE((finder.template_name.fg_mask & unused_mask).none())       \
        << #template_name;                                                 \
  }                                                                        \
  static_assert(true, "")

TEST(GraphExtractionUtilities, CornerDetectionTemplatesSound) {
  CornerFinder finder;
  CHECK_TEMPLATE_SOUNDNESS(finder, negative_x_template);
  CHECK_TEMPLATE_SOUNDNESS(finder, positive_x_template);
  CHECK_TEMPLATE_SOUNDNESS(finder, negative_y_template);
  CHECK_TEMPLATE_SOUNDNESS(finder, positive_y_template);
  CHECK_TEMPLATE_SOUNDNESS(finder, negative_z_template);
  CHECK_TEMPLATE_SOUNDNESS(finder, positive_z_template);
}

#undef CHECK_TEMPLATE_SOUNDNESS

#define TEST_CORNER_ROTATION(finder, rotation)       \
  for (uint8_t i = 0; i < 64; ++i) {                 \
    std::bitset<27> corner = getCorner(i);           \
    corner = rotateCorner(rotation, corner);         \
    EXPECT_TRUE(finder.match(corner));               \
  }                                                  \
  for (uint8_t i = 0; i < 20; ++i) {                 \
    std::bitset<27> corner = getCorner(63, true, i); \
    corner = rotateCorner(rotation, corner);         \
    EXPECT_FALSE(finder.match(corner));              \
  }                                                  \
  static_assert(true, "")

// TODO(nathan) there's probably a more exhaustive way to test this
TEST(GraphExtractionUtilities, CornerDetectionTemplateCorrect) {
  CornerFinder finder;

  for (uint8_t i = 0; i < 64; ++i) {
    EXPECT_TRUE(finder.match(getCorner(i)));
  }

  for (size_t i = 0; i < 20; ++i) {
    EXPECT_FALSE(finder.match(getCorner(63, true, i)));
  }

  {  // 90 degree x
    IndexRotation rotation = rotX();
    TEST_CORNER_ROTATION(finder, rotation);
  }

  {  // 90 degree y
    IndexRotation rotation = rotY();
    TEST_CORNER_ROTATION(finder, rotation);
  }

  {  // 90 degree z
    IndexRotation rotation = rotZ();
    TEST_CORNER_ROTATION(finder, rotation);
  }

  {  // x composed with z
    IndexRotation rotation = rotX() * rotZ();
    TEST_CORNER_ROTATION(finder, rotation);
  }

  {  // 180 z
    IndexRotation rotation = rotZ() * rotZ();
    TEST_CORNER_ROTATION(finder, rotation);
  }

  {  // 270 z
    IndexRotation rotation = rotZ() * rotZ() * rotZ();
    TEST_CORNER_ROTATION(finder, rotation);
  }

  {  // 180 y
    IndexRotation rotation = rotY() * rotY();
    TEST_CORNER_ROTATION(finder, rotation);
  }

  {  // 270 y
    IndexRotation rotation = rotY() * rotY() * rotY();
    TEST_CORNER_ROTATION(finder, rotation);
  }
}

#undef TEST_CORNER_ROTATION

TEST(GraphExtractionUtilities, TestBresenhamLine) {
  {  // x primary axis
    GlobalIndex start(1, 2, 3);
    GlobalIndex end(6, 4, 5);
    voxblox::AlignedVector<GlobalIndex> expected(4);
    expected[0] << 2, 2, 3;
    expected[1] << 3, 3, 4;
    expected[2] << 4, 3, 4;
    expected[3] << 5, 4, 5;

    voxblox::AlignedVector<GlobalIndex> result = makeBresenhamLine(start, end);
    ASSERT_EQ(4u, result.size());
    for (size_t i = 0; i < 4u; i++) {
      EXPECT_EQ(expected[i], result[i]);
    }
  }

  {  // y primary axis
    GlobalIndex start(2, 1, 3);
    GlobalIndex end(4, 6, 5);
    voxblox::AlignedVector<GlobalIndex> expected(4);
    expected[0] << 2, 2, 3;
    expected[1] << 3, 3, 4;
    expected[2] << 3, 4, 4;
    expected[3] << 4, 5, 5;

    voxblox::AlignedVector<GlobalIndex> result = makeBresenhamLine(start, end);
    ASSERT_EQ(4u, result.size());
    for (size_t i = 0; i < 4u; i++) {
      EXPECT_EQ(expected[i], result[i]);
    }
  }

  {  // z primary axis
    GlobalIndex start(2, 3, 1);
    GlobalIndex end(4, 5, 6);
    voxblox::AlignedVector<GlobalIndex> expected(4);
    expected[0] << 2, 3, 2;
    expected[1] << 3, 4, 3;
    expected[2] << 3, 4, 4;
    expected[3] << 4, 5, 5;

    voxblox::AlignedVector<GlobalIndex> result = makeBresenhamLine(start, end);
    ASSERT_EQ(4u, result.size());
    for (size_t i = 0; i < 4u; i++) {
      EXPECT_EQ(expected[i], result[i]);
    }
  }

  {  // equal slope
    GlobalIndex start(1, 1, 1);
    GlobalIndex end(6, 6, 6);
    voxblox::AlignedVector<GlobalIndex> expected(4);
    expected[0] << 2, 2, 2;
    expected[1] << 3, 3, 3;
    expected[2] << 4, 4, 4;
    expected[3] << 5, 5, 5;

    voxblox::AlignedVector<GlobalIndex> result = makeBresenhamLine(start, end);
    ASSERT_EQ(4u, result.size());
    for (size_t i = 0; i < 4u; i++) {
      EXPECT_EQ(expected[i], result[i]);
    }
  }
}

TEST(GraphExtractionUtilities, TestBresenhamLineEmpty) {
  {  // same start and end
    GlobalIndex start(1, 1, 1);
    GlobalIndex end(1, 1, 1);
    EXPECT_TRUE(makeBresenhamLine(start, end).empty());
  }

  {  // one step away x
    GlobalIndex start(1, 1, 1);
    GlobalIndex end(2, 1, 1);
    EXPECT_TRUE(makeBresenhamLine(start, end).empty());
  }

  {  // one step away y
    GlobalIndex start(1, 1, 1);
    GlobalIndex end(1, 2, 1);
    EXPECT_TRUE(makeBresenhamLine(start, end).empty());
  }

  {  // one step away z
    GlobalIndex start(1, 1, 1);
    GlobalIndex end(1, 1, 2);
    EXPECT_TRUE(makeBresenhamLine(start, end).empty());
  }
}

}  // namespace topology
}  // namespace hydra
