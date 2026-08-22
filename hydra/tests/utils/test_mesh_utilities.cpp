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
#include <hydra/utils/mesh_utilities.h>

namespace hydra {

using ClusterResult = std::vector<std::vector<size_t>>;
using spark_dsg::Mesh;

namespace {

void compareClusters(const ClusterResult& expected, const ClusterResult& result) {
  EXPECT_EQ(expected.size(), result.size());
  if (expected.size() != result.size()) {
    return;
  }

  std::set<size_t> seen_expected;
  for (const auto& cluster : result) {
    std::optional<size_t> match_idx;
    std::vector<size_t> result_indices = cluster;
    std::sort(result_indices.begin(), result_indices.end());
    for (size_t i = 0; i < expected.size(); ++i) {
      std::vector<size_t> expected_indices = expected[i];
      std::sort(expected_indices.begin(), expected_indices.end());
      if (result_indices == expected_indices) {
        match_idx = i;
        seen_expected.insert(i);
        break;
      }
    }

    if (!match_idx) {
      FAIL() << "Could not find match for " << ::testing::PrintToString(cluster)
             << " vs. expected " << ::testing::PrintToString(expected);
    }
  }

  EXPECT_EQ(seen_expected.size(), expected.size())
      << "Could not find all expected " << ::testing::PrintToString(expected) << " vs. "
      << ::testing::PrintToString(result);
}

}  // namespace

TEST(MeshUtilities, SegmentSplit) {
  Mesh::Positions points;

  Mesh::Pos seed0(1.0, 2.0, 3.0);
  Mesh::Pos seed1(2.0, 3.0, 4.0);
  points.push_back(seed0);
  points.push_back(seed1);
  points.push_back(seed0 + Eigen::Vector3f(0.0, 0.25, 0.0));
  points.push_back(seed1 + Eigen::Vector3f(0.0, 0.25, 0.0));
  points.push_back(seed0 - Eigen::Vector3f(0.0, 0.25, 0.0));
  points.push_back(seed1 - Eigen::Vector3f(0.0, 0.25, 0.0));
  points.push_back(seed0 + Eigen::Vector3f(0.25, 0.0, 0.0));
  points.push_back(seed1 + Eigen::Vector3f(0.25, 0.0, 0.0));
  points.push_back(seed0 - Eigen::Vector3f(0.25, 0.0, 0.0));
  points.push_back(seed1 - Eigen::Vector3f(0.25, 0.0, 0.0));
  points.push_back(seed0 + Eigen::Vector3f(0.0, 0.0, 0.25));
  points.push_back(seed1 + Eigen::Vector3f(0.0, 0.0, 0.25));
  points.push_back(seed0 - Eigen::Vector3f(0.0, 0.0, 0.25));
  points.push_back(seed1 - Eigen::Vector3f(0.0, 0.0, 0.25));

  {  // separate points
    ClusterResult expected{{0, 2, 4, 6, 8, 10, 12}, {1, 3, 5, 7, 9, 11, 13}};
    const auto results = getConnectedComponents(points, 0.3);
    compareClusters(expected, results);
  }

  {  // all points
    ClusterResult expected{{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13}};
    const auto results = getConnectedComponents(points, 3.0);
    compareClusters(expected, results);
  }
}

}  // namespace hydra
