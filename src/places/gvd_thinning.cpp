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
#include "hydra/places/gvd_thinning.h"

#include <array>
#include <iostream>
#include <list>
#include <unordered_map>

namespace hydra {
namespace places {

// based on:
// https://github.com/scikit-image/scikit-image/blob/v0.19.2/skimage/morphology/_skeletonize_3d_cy.pyx.in
// and:
// Lee et al. "Building skeleton models via 3-D medial surface/axis thinning
// algorithms."

namespace {

struct OctantNeighborInfo {
  size_t index;
  std::list<uint8_t> octants;
};

static const std::array<int8_t, 128> EULER_DELTA = {
    1,  -1, -1, 1,  -3, -1, -1, 1,  -1, 1,  1, -1, 3,  1,  1, -1, -3, -1, 3,
    1,  1,  -1, 3,  1,  -1, 1,  1,  -1, 3,  1, 1,  -1, -3, 3, -1, 1,  1,  3,
    -1, 1,  -1, 1,  1,  -1, 3,  1,  1,  -1, 1, 3,  3,  1,  5, 3,  3,  1,  -1,
    1,  1,  -1, 3,  1,  1,  -1, -7, -1, -1, 1, -3, -1, -1, 1, -1, 1,  1,  -1,
    3,  1,  1,  -1, -3, -1, 3,  1,  1,  -1, 3, 1,  -1, 1,  1, -1, 3,  1,  1,
    -1, -3, 3,  -1, 1,  1,  3,  -1, 1,  -1, 1, 1,  -1, 3,  1, 1,  -1, 1,  3,
    3,  1,  5,  3,  3,  1,  -1, 1,  1,  -1, 3, 1,  1,  -1};

static const std::unordered_map<size_t, uint8_t> INDEX_TO_OCTANT{
    {0, 1},  {1, 1},  {2, 2},  {3, 1},  {4, 1},  {5, 2},  {6, 3},  {7, 3},  {8, 4},
    {9, 1},  {10, 1}, {11, 2}, {12, 1}, {13, 2}, {14, 3}, {15, 3}, {16, 4}, {17, 5},
    {18, 5}, {19, 6}, {20, 5}, {21, 5}, {22, 6}, {23, 7}, {24, 7}, {25, 8},
};

static const std::array<std::array<uint8_t, 7>, 8> OCTANTS{{
    {{2, 1, 11, 10, 5, 4, 13}},
    {{0, 9, 3, 12, 1, 10, 4}},
    {{8, 7, 16, 15, 5, 4, 13}},
    {{6, 14, 7, 15, 3, 12, 4}},
    {{19, 22, 18, 21, 11, 13, 10}},
    {{17, 20, 9, 12, 18, 21, 10}},
    {{25, 22, 16, 13, 24, 21, 15}},
    {{23, 24, 14, 15, 20, 21, 12}},
}};

static const std::unordered_map<uint8_t, std::list<OctantNeighborInfo>> OCTANT_CHILDREN{
    {1,
     {{0, {}},
      {1, {2}},
      {3, {3}},
      {4, {2, 3, 4}},
      {9, {5}},
      {10, {2, 5, 6}},
      {12, {3, 5, 7}}}},
    {2,
     {{1, {1}},
      {4, {1, 3, 4}},
      {10, {1, 5, 6}},
      {2, {}},
      {5, {4}},
      {11, {6}},
      {13, {4, 6, 8}}}},
    {3,
     {{3, {1}},
      {4, {1, 2, 4}},
      {12, {1, 5, 7}},
      {6, {}},
      {7, {4}},
      {14, {7}},
      {15, {4, 7, 8}}}},
    {4,
     {{4, {1, 2, 3}},
      {5, {2}},
      {13, {2, 6, 8}},
      {7, {3}},
      {15, {3, 7, 8}},
      {8, {}},
      {16, {8}}}},
    {5,
     {{9, {1}},
      {10, {1, 2, 6}},
      {12, {1, 3, 7}},
      {17, {}},
      {18, {6}},
      {20, {7}},
      {21, {6, 7, 8}}}},
    {6,
     {{10, {1, 2, 5}},
      {11, {2}},
      {13, {2, 4, 8}},
      {18, {5}},
      {21, {5, 7, 8}},
      {19, {}},
      {22, {8}}}},
    {7,
     {{12, {1, 3, 5}},
      {14, {3}},
      {15, {3, 4, 8}},
      {20, {5}},
      {21, {5, 6, 8}},
      {23, {}},
      {24, {8}}}},
    {8,
     {{13, {2, 4, 6}},
      {15, {3, 4, 7}},
      {16, {4}},
      {21, {5, 6, 7}},
      {22, {6}},
      {24, {7}},
      {25, {}}}},
};

void searchOctTree(uint8_t octant, std::bitset<26>& labels) {
  for (const auto& child : OCTANT_CHILDREN.at(octant)) {
    if (labels[child.index]) {
      labels[child.index] = false;
      for (const auto new_octant : child.octants) {
        searchOctTree(new_octant, labels);
      }
    }
  }
}

}  // namespace

bool noEulerChange(const std::bitset<26>& values) {
  int32_t delta_value = 0;
  for (size_t i = 0; i < 8; ++i) {
    // center vertex is always lsb (and has to be GVD member)
    std::bitset<8> curr_octant(1);
    for (size_t idx = 1; idx < 8; ++idx) {
      curr_octant[i] = values[OCTANTS[i][idx]];
    }

    delta_value += EULER_DELTA[static_cast<uint8_t>(curr_octant.to_ulong())];
  }

  std::cout << "values: " << values << ", change: " << delta_value << std::endl;

  return delta_value == 0;
}

bool isSimplePoint(const std::bitset<26>& neighborhood) {
  std::bitset<26> labels = neighborhood;

  size_t num_components = 0;
  for (size_t i = 0; i < 26; ++i) {
    if (!labels[i]) {
      continue;
    }

    num_components++;
    if (num_components > 1) {
      return false;
    }

    searchOctTree(INDEX_TO_OCTANT.at(i), labels);
  }

  return true;
}

}  // namespace places
}  // namespace hydra
