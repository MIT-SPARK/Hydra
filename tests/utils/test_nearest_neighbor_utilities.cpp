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
#include <hydra/utils/nearest_neighbor_utilities.h>

#include <random>

namespace hydra {
namespace {

std::set<size_t> getIndexResults(const PointNeighborSearch& finder,
                                 const Eigen::Vector3f& pos,
                                 float radius) {
  const auto vec = finder.pointsInRadius(pos, radius);
  std::set result(vec.begin(), vec.end());
  return result;
}

}  // namespace

TEST(NearestNeighborUtilities, SkipFirst) {
  SceneGraphLayer layer(1);
  layer.emplaceNode(0, std::make_unique<NodeAttributes>(Eigen::Vector3d(0, 0, 3)));
  layer.emplaceNode(1, std::make_unique<NodeAttributes>(Eigen::Vector3d(0, 0, 0)));
  layer.emplaceNode(2, std::make_unique<NodeAttributes>(Eigen::Vector3d(3, 0, 0)));
  layer.emplaceNode(3, std::make_unique<NodeAttributes>(Eigen::Vector3d(0, 3, 0)));

  std::vector<NodeId> nodes{0, 1, 2, 3};
  NearestNodeFinder finder(layer, nodes);

  {  // test 1:
    NodeId result;
    double distance;
    finder.find(
        Eigen::Vector3d(0, 0, 2), 1, false, [&](NodeId node, size_t, double dist) {
          result = node;
          distance = std::sqrt(dist);
        });
    EXPECT_EQ(result, 0u);
    EXPECT_NEAR(distance, 1.0, 1.0e-9);
  }

  {  // test 1:
    NodeId result;
    double distance;
    finder.find(
        Eigen::Vector3d(0, 0, 2), 1, true, [&](NodeId node, size_t, double dist) {
          result = node;
          distance = std::sqrt(dist);
        });
    EXPECT_EQ(result, 1u);
    EXPECT_NEAR(distance, 2.0, 1.0e-9);
  }
}

TEST(NearestNeighborUtilities, RadiusSearch) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dist(-0.25, 0.25);
  std::vector<Eigen::Vector3f> points;

  Eigen::Vector3f seed0(1.0, 2.0, 3.0);
  Eigen::Vector3f seed1(2.0, 3.0, 4.0);
  for (size_t i = 0; i < 10; ++i) {
    points.push_back(seed0 + Eigen::Vector3f(dist(gen), dist(gen), dist(gen)));
    points.push_back(seed1 + Eigen::Vector3f(dist(gen), dist(gen), dist(gen)));
  }

  PointNeighborSearch finder(points);
  {  // separate points
    const std::set<size_t> expected0{0, 2, 4, 6, 8, 10, 12, 14, 16, 18};
    EXPECT_EQ(getIndexResults(finder, seed0, 0.5), expected0);
    const std::set<size_t> expected1{1, 3, 5, 7, 9, 11, 13, 15, 17, 19};
    EXPECT_EQ(getIndexResults(finder, seed1, 0.5), expected1);
  }

  {  // all points
    std::set<size_t> expected{0,  1,  2,  3,  4,  5,  6,  7,  8,  9,
                              10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
    EXPECT_EQ(getIndexResults(finder, seed0, 5.0), expected);
    EXPECT_EQ(getIndexResults(finder, seed1, 5.0), expected);
  }

  {  // no points
    const std::set<size_t> expected{};
    Eigen::Vector3f query(100.0, 200.0, 300.0);
    EXPECT_EQ(getIndexResults(finder, query, 5.0), expected);
  }
}

}  // namespace hydra
