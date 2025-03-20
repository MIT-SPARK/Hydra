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
#include <hydra/utils/disjoint_set.h>
#include <hydra/utils/minimum_spanning_tree.h>

namespace hydra {

// most for coverage
TEST(DisjointSet, TestSameCluster) {
  DisjointSet set;
  set.addSet(0);
  set.addSet(1);
  auto result = set.doUnion(0, 1);
  ASSERT_TRUE(result);
  EXPECT_EQ(result.value(), 1u);
  auto retry_result = set.doUnion(0, 1);
  EXPECT_FALSE(retry_result);
}

TEST(MinimumSpanningTreeTests, TestSingleChain) {
  SceneGraphLayer layer(1);
  layer.emplaceNode(0,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(0.0, 0.0, 0.0)));
  layer.emplaceNode(1,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(1.0, 0.0, 0.0)));
  layer.emplaceNode(2,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(3.0, 0.0, 0.0)));
  layer.emplaceNode(3,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(6.0, 0.0, 0.0)));
  layer.insertEdge(0, 1);
  layer.insertEdge(1, 2);
  layer.insertEdge(2, 3);

  MinimumSpanningTreeInfo info = getMinimumSpanningEdges(layer);
  EXPECT_TRUE(info.leaves.count(0));
  EXPECT_TRUE(info.leaves.count(3));
  EXPECT_EQ(2u, info.leaves.size());
  ASSERT_EQ(3u, info.edges.size());
  EXPECT_EQ(0u, info.edges[0].source);
  EXPECT_EQ(1u, info.edges[0].target);
  EXPECT_EQ(1u, info.edges[1].source);
  EXPECT_EQ(2u, info.edges[1].target);
  EXPECT_EQ(2u, info.edges[2].source);
  EXPECT_EQ(3u, info.edges[2].target);
}

TEST(MinimumSpanningTreeTests, TestCompleteGraph) {
  SceneGraphLayer layer(1);
  layer.emplaceNode(0,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(0.0, 0.0, 0.0)));
  layer.emplaceNode(1,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(1.0, 0.0, 0.0)));
  layer.emplaceNode(2,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(3.0, 0.0, 0.0)));
  layer.emplaceNode(3,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(6.0, 0.0, 0.0)));
  layer.insertEdge(0, 1);
  layer.insertEdge(0, 2);
  layer.insertEdge(0, 3);
  layer.insertEdge(1, 2);
  layer.insertEdge(1, 3);
  layer.insertEdge(2, 3);

  MinimumSpanningTreeInfo info = getMinimumSpanningEdges(layer);
  EXPECT_TRUE(info.leaves.count(0));
  EXPECT_TRUE(info.leaves.count(3));
  EXPECT_EQ(2u, info.leaves.size());
  ASSERT_EQ(3u, info.edges.size());
  EXPECT_EQ(0u, info.edges[0].source);
  EXPECT_EQ(1u, info.edges[0].target);
  EXPECT_EQ(1u, info.edges[1].source);
  EXPECT_EQ(2u, info.edges[1].target);
  EXPECT_EQ(2u, info.edges[2].source);
  EXPECT_EQ(3u, info.edges[2].target);
}

}  // namespace hydra
