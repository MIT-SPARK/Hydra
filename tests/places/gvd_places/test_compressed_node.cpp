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
#include <hydra/places/gvd_places/compressed_node.h>

namespace hydra::places {

using NodeMap = std::unordered_map<uint64_t, CompressedNode>;
using NeighborMap = std::unordered_map<uint64_t, std::set<uint64_t>>;

TEST(CompressedNode, AddEdgesCorrect) {
  NodeMap nodes{{0, CompressedNode(0)}, {1, CompressedNode(1)}, {2, CompressedNode(2)}};
  nodes.at(0).addEdgeObservation(0, 1, 1);
  nodes.at(1).addEdgeObservation(1, 0, 0);
  nodes.at(1).addEdgeObservation(1, 2, 2);
  nodes.at(2).addEdgeObservation(2, 1, 1);

  NeighborMap expected_siblings{
      {0, {1}},
      {1, {0, 2}},
      {2, {1}},
  };
  EXPECT_EQ(nodes.at(0).siblings, expected_siblings.at(0));
  EXPECT_EQ(nodes.at(1).siblings, expected_siblings.at(1));
  EXPECT_EQ(nodes.at(2).siblings, expected_siblings.at(2));
}

TEST(CompressedNode, RemoveEdgesCorrect) {
  NodeMap nodes{{0, CompressedNode(0)}, {1, CompressedNode(1)}, {2, CompressedNode(2)}};
  nodes.at(0).addEdgeObservation(0, 1, 1);
  nodes.at(1).addEdgeObservation(1, 0, 0);
  nodes.at(1).addEdgeObservation(1, 2, 2);
  nodes.at(2).addEdgeObservation(2, 1, 1);

  const auto removed_siblings = nodes.at(1).removeEdgeObservations(1, nodes);
  EXPECT_TRUE(nodes.at(0).siblings.empty());
  EXPECT_TRUE(nodes.at(1).siblings.empty());
  EXPECT_TRUE(nodes.at(2).siblings.empty());

  std::list<uint64_t> expected_removed{0, 2};
  EXPECT_EQ(removed_siblings, expected_removed);
}

}  // namespace hydra::places
