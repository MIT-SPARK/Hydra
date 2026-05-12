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
#include <hydra/common/partial_graph.h>
#include <spark_dsg/printing.h>

namespace hydra {

using spark_dsg::NodeAttributes;
using spark_dsg::NodeId;

using TestGraph = PartialGraph<NodeAttributes>;

TEST(PartialGraph, AddUpdateCorrect) {
  TestGraph graph;
  graph.add(5);
  graph.add(5, 6);
  graph.add(6, 7);

  EXPECT_TRUE(graph.has(5));
  EXPECT_TRUE(graph.has(6));
  EXPECT_TRUE(graph.has(7));
  EXPECT_TRUE(graph.find(7));  // 7 added by edge
  EXPECT_FALSE(graph.has(8));

  EXPECT_TRUE(graph.has(5, 6));
  EXPECT_TRUE(graph.has(6, 7));
  EXPECT_FALSE(graph.has(7, 8));

  EXPECT_TRUE(graph.find(5, 6));
  EXPECT_TRUE(graph.find(6, 7));

  std::set<NodeId> expected;
  expected = {6};
  EXPECT_EQ(graph.neighbors(5), expected);
  expected = {5, 7};
  EXPECT_EQ(graph.neighbors(6), expected);
  expected = {6};
  EXPECT_EQ(graph.neighbors(7), expected);
}

TEST(PartialGraph, RemoveNodeCorrect) {
  TestGraph graph;
  graph.add(5, 6);
  graph.add(6, 7);
  EXPECT_TRUE(graph.deleted_nodes().empty());
  EXPECT_TRUE(graph.deleted_edges().empty());

  graph.remove(6);
  std::set<spark_dsg::NodeId> deleted_nodes{6};
  std::set<spark_dsg::EdgeKey> deleted_edges{{5, 6}, {6, 7}};
  EXPECT_EQ(graph.deleted_nodes(), deleted_nodes);
  EXPECT_EQ(graph.deleted_edges(), deleted_edges);

  EXPECT_EQ(graph.num_edges(), 0u);
  EXPECT_TRUE(graph.neighbors(5).empty());
  EXPECT_TRUE(graph.neighbors(6).empty());

  graph.add(6);
  EXPECT_EQ(graph.num_edges(), 0u);
  EXPECT_EQ(graph.deleted_edges(), deleted_edges);
  EXPECT_TRUE(graph.deleted_nodes().empty());

  graph.add(5, 6);
  deleted_edges = {{6, 7}};
  EXPECT_EQ(graph.num_edges(), 1u);
  EXPECT_EQ(graph.deleted_edges(), deleted_edges);
  EXPECT_TRUE(graph.deleted_nodes().empty());
}

TEST(PartialGraph, RemoveEdgeCorrect) {
  TestGraph graph;
  graph.add(5, 6);
  graph.add(6, 7);
  EXPECT_TRUE(graph.deleted_nodes().empty());
  EXPECT_TRUE(graph.deleted_edges().empty());

  graph.remove(6, 7);
  std::set<spark_dsg::EdgeKey> deleted_edges{{6, 7}};
  EXPECT_TRUE(graph.deleted_nodes().empty());
  EXPECT_EQ(graph.deleted_edges(), deleted_edges);

  EXPECT_EQ(graph.num_edges(), 1u);
  std::set<NodeId> expected;
  expected = {6};
  EXPECT_EQ(graph.neighbors(5), expected);
  expected = {5};
  EXPECT_EQ(graph.neighbors(6), expected);
  EXPECT_TRUE(graph.neighbors(7).empty());

  graph.add(6, 7);
  EXPECT_EQ(graph.num_edges(), 2u);
  EXPECT_TRUE(graph.deleted_nodes().empty());
  EXPECT_TRUE(graph.deleted_edges().empty());
}

TEST(PartialGraph, PruneGraphTrivial) {
  TestGraph graph;
  graph.add(5, 6);
  graph.add(6, 7);
  graph.add(7, 8);
  graph.remove(7, 8);

  std::set<spark_dsg::EdgeKey> deleted_edges{{7, 8}};
  EXPECT_EQ(graph.deleted_edges(), deleted_edges);

  graph.prune();
  EXPECT_EQ(graph.num_nodes(), 0u);
  EXPECT_EQ(graph.num_edges(), 0u);
  EXPECT_TRUE(graph.deleted_nodes().empty());
  EXPECT_TRUE(graph.deleted_edges().empty());
}

TEST(PartialGraph, PruneGraph) {
  TestGraph graph;
  graph.add(5).is_active = true;
  graph.add(5, 6);
  graph.add(6, 7);
  graph.archive(6);
  graph.archive(7);

  const auto pruned = graph.prune();
  std::vector<uint64_t> expected{7};
  EXPECT_EQ(pruned, expected);
  EXPECT_EQ(graph.num_nodes(), 2u);
  EXPECT_EQ(graph.num_edges(), 1u);
  EXPECT_TRUE(graph.deleted_nodes().empty());
  EXPECT_TRUE(graph.deleted_edges().empty());
}

}  // namespace hydra
