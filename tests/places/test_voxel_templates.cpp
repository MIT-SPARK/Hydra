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
#include <gtest/gtest.h>
#include <hydra/places/voxel_templates.h>
#include <spatial_hash/neighbor_utils.h>

#include "hydra_test/place_fixtures.h"

namespace hydra::places {

using test::SingleBlockExtractionTestFixture;

CubeFlagExtractor::IndexRotation rotX() {
  CubeFlagExtractor::IndexRotation to_return;
  to_return << 1, 0, 0, 0, 0, -1, 0, 1, 0;
  return to_return;
}

CubeFlagExtractor::IndexRotation rotY() {
  CubeFlagExtractor::IndexRotation to_return;
  to_return << 0, 0, 1, 0, 1, 0, -1, 0, 0;
  return to_return;
}

CubeFlagExtractor::IndexRotation rotZ() {
  CubeFlagExtractor::IndexRotation to_return;
  to_return << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  return to_return;
}

/**
 * @brief Get a 3x3 cube permutation that matches (or doesn't match) an unrotated corner
 * template
 */
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

  return CubeFlagExtractor::fromRowMajor(corner_values);
}

TEST(VoxelTemplates, RowMajorConversion) {
  {  // 0 -> 0
    std::bitset<27> expected_rm{0x0000'0000};
    std::bitset<27> expected_sh{0x0000'0000};
    const auto result_sh = CubeFlagExtractor::fromRowMajor(expected_rm);
    const auto result_rm = CubeFlagExtractor::toRowMajor(expected_sh);
    EXPECT_EQ(expected_rm, result_rm);
    EXPECT_EQ(expected_sh, result_sh);
  }

  {  // 1 -> 1
    std::bitset<27> expected_rm{0x7FFF'FFFF};
    std::bitset<27> expected_sh{0x7FFF'FFFF};
    const auto result_sh = CubeFlagExtractor::fromRowMajor(expected_rm);
    const auto result_rm = CubeFlagExtractor::toRowMajor(expected_sh);
    EXPECT_EQ(expected_rm, result_rm);
    EXPECT_EQ(expected_sh, result_sh);
  }

  {  // single-bit correctness
    // derived by drawing 3x3 grid and examining the voxblox offset table
    std::map<size_t, size_t> neighbor_to_row_major{
        {1, 14},  {2, 12},  {3, 16}, {4, 10},  {5, 22},  {6, 4},   {7, 17},
        {8, 11},  {9, 15},  {10, 9}, {11, 25}, {12, 7},  {13, 19}, {14, 1},
        {15, 23}, {16, 21}, {17, 5}, {18, 3},  {19, 26}, {20, 8},  {21, 20},
        {22, 2},  {23, 24}, {24, 6}, {25, 18}, {26, 0}};

    std::set<size_t> neighbor_indices;
    std::set<size_t> row_major_indices;
    for (const auto& index_pair : neighbor_to_row_major) {
      neighbor_indices.insert(index_pair.first);
      row_major_indices.insert(index_pair.second);
    }

    // we should have a bijective index mapping {0, 26} -> {0, 27} \ {13}
    ASSERT_EQ(26u, neighbor_indices.size());
    ASSERT_EQ(1u, *std::min_element(neighbor_indices.begin(), neighbor_indices.end()));
    ASSERT_EQ(26u, *std::max_element(neighbor_indices.begin(), neighbor_indices.end()));
    ASSERT_EQ(26u, row_major_indices.size());
    ASSERT_EQ(0u,
              *std::min_element(row_major_indices.begin(), row_major_indices.end()));
    ASSERT_EQ(26u,
              *std::max_element(row_major_indices.begin(), row_major_indices.end()));
    ASSERT_EQ(0u, row_major_indices.count(13));

    for (const auto& index_pair : neighbor_to_row_major) {
      std::bitset<27> expected_rm{0};
      expected_rm.set(index_pair.second, true);
      expected_rm.set(13, true);

      std::bitset<27> expected_sh{0};
      expected_sh.set(index_pair.first, true);
      expected_sh.set(0, true);

      const auto result_rm = CubeFlagExtractor::toRowMajor(expected_sh);
      const auto result_sh = CubeFlagExtractor::fromRowMajor(expected_rm);
      EXPECT_EQ(expected_rm, result_rm);
      EXPECT_EQ(expected_sh, result_sh);
    }
  }
}

TEST_F(SingleBlockExtractionTestFixture, NeighborhoodExtraction) {
  {  // outside any allocated voxels -> nothing should be in the GVD
    GlobalIndex index;
    index << 10, 12, 15;
    const auto result = CubeFlagExtractor::extract(*gvd_layer, index);
    EXPECT_TRUE(result.none());
  }

  {  // origin of block -> still no gvd voxels touched
    GlobalIndex index;
    index << 0, 0, 0;
    std::bitset<27> expected_row_major{0};
    const auto expected = CubeFlagExtractor::fromRowMajor(expected_row_major);
    const auto result = CubeFlagExtractor::extract(*gvd_layer, index);
    EXPECT_EQ(expected, result);
  }

  {  // upper right corner is gvd
    GlobalIndex index;
    index << 1, 1, 1;
    std::bitset<27> expected_row_major = 0b00'000'000'000'000'000'000'000'001;
    const auto expected = CubeFlagExtractor::fromRowMajor(expected_row_major);
    const auto result = CubeFlagExtractor::extract(*gvd_layer, index);
    EXPECT_EQ(expected, result);
  }

  {  // lowest gvd corner is center
    GlobalIndex index;
    index << 2, 2, 2;
    std::bitset<27> expected_row_major = 0b000'000'000'000'011'011'000'011'011;
    const auto expected = CubeFlagExtractor::fromRowMajor(expected_row_major);
    const auto result = CubeFlagExtractor::extract(*gvd_layer, index);
    EXPECT_EQ(expected, result);
  }
}

TEST(VoxelTemplates, RotationCorrect) {
  {  // check that rotation makes sense
    const auto rot = rotZ();
    std::bitset<27> init_rm = 0b000'000'000'000'000'000'100'010'001;
    std::bitset<27> expected_rm = 0b000'000'000'000'000'000'001'010'100;
    const auto result =
        CubeFlagExtractor::rotate(rot, CubeFlagExtractor::fromRowMajor(init_rm));
    EXPECT_EQ(result, CubeFlagExtractor::fromRowMajor(expected_rm));
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

TEST(VoxelTemplates, CornerDetectionTemplatesSound) {
  CornerFinder finder;
  CHECK_TEMPLATE_SOUNDNESS(finder, negative_x_template);
  CHECK_TEMPLATE_SOUNDNESS(finder, positive_x_template);
  CHECK_TEMPLATE_SOUNDNESS(finder, negative_y_template);
  CHECK_TEMPLATE_SOUNDNESS(finder, positive_y_template);
  CHECK_TEMPLATE_SOUNDNESS(finder, negative_z_template);
  CHECK_TEMPLATE_SOUNDNESS(finder, positive_z_template);
}

#undef CHECK_TEMPLATE_SOUNDNESS

#define TEST_CORNER_ROTATION(finder, rotation)            \
  for (uint8_t i = 0; i < 64; ++i) {                      \
    std::bitset<27> corner = getCorner(i);                \
    corner = CubeFlagExtractor::rotate(rotation, corner); \
    EXPECT_TRUE(finder.match(corner));                    \
  }                                                       \
  for (uint8_t i = 0; i < 20; ++i) {                      \
    std::bitset<27> corner = getCorner(63, true, i);      \
    corner = CubeFlagExtractor::rotate(rotation, corner); \
    EXPECT_FALSE(finder.match(corner));                   \
  }                                                       \
  static_assert(true, "")

TEST(VoxelTemplates, CornerDetectionTemplateCorrect) {
  CornerFinder finder;

  for (uint8_t i = 0; i < 64; ++i) {
    EXPECT_TRUE(finder.match(getCorner(i)));
  }

  for (size_t i = 0; i < 20; ++i) {
    EXPECT_FALSE(finder.match(getCorner(63, true, i)));
  }

  {  // 90 degree x
    const auto rotation = rotX();
    TEST_CORNER_ROTATION(finder, rotation);
  }

  {  // 90 degree y
    const auto rotation = rotY();
    TEST_CORNER_ROTATION(finder, rotation);
  }

  {  // 90 degree z
    const auto rotation = rotZ();
    TEST_CORNER_ROTATION(finder, rotation);
  }

  {  // x composed with z
    const CubeFlagExtractor::IndexRotation rotation = rotX() * rotZ();
    TEST_CORNER_ROTATION(finder, rotation);
  }

  {  // 180 z
    const CubeFlagExtractor::IndexRotation rotation = rotZ() * rotZ();
    TEST_CORNER_ROTATION(finder, rotation);
  }

  {  // 270 z
    const CubeFlagExtractor::IndexRotation rotation = rotZ() * rotZ() * rotZ();
    TEST_CORNER_ROTATION(finder, rotation);
  }

  {  // 180 y
    const CubeFlagExtractor::IndexRotation rotation = rotY() * rotY();
    TEST_CORNER_ROTATION(finder, rotation);
  }

  {  // 270 y
    const CubeFlagExtractor::IndexRotation rotation = rotY() * rotY() * rotY();
    TEST_CORNER_ROTATION(finder, rotation);
  }
}

#undef TEST_CORNER_ROTATION

}  // namespace hydra::places
