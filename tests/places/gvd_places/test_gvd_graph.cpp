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
#include <hydra/places/gvd_places/gvd_graph.h>

namespace hydra::places {
namespace {

void callArchive(GvdGraph& graph, const std::vector<GlobalIndex>& indices) {
  std::queue<GlobalIndex> temp_queue;
  for (const auto& index : indices) {
    temp_queue.push(index);
  }

  graph.archive(temp_queue);
}

void callRemove(GvdGraph& graph, const std::vector<GlobalIndex>& indices) {
  std::queue<GlobalIndex> temp_queue;
  for (const auto& index : indices) {
    temp_queue.push(index);
  }

  graph.remove(temp_queue);
}

void fillGraph(GvdGraph& graph, const std::vector<GlobalIndex>& indices) {
  for (const auto& idx : indices) {
    graph.add(idx, 0.1, 2);
  }
}

bool hasEdge(const GvdGraph& graph, uint64_t source_id, uint64_t target_id) {
  auto source = graph.uncompressed().find(source_id);
  if (source == graph.uncompressed().end()) {
    return false;
  }

  auto target = graph.uncompressed().find(target_id);
  if (target == graph.uncompressed().end()) {
    return false;
  }

  return source->second.siblings.count(target_id) &&
         target->second.siblings.count(source_id);
}

bool hasCompressedEdge(const GvdGraph& graph, uint64_t source_id, uint64_t target_id) {
  auto source = graph.compressed().find(source_id);
  if (source == graph.compressed().end()) {
    return false;
  }

  auto target = graph.compressed().find(target_id);
  if (target == graph.compressed().end()) {
    return false;
  }

  return source->second.siblings.count(target_id) &&
         target->second.siblings.count(source_id);
}

}  // namespace

TEST(GvdGraph, ArchivedCorrect) {
  GvdGraph::CompressedNode node(0);
  node.active_refs.insert(1);
  node.archived_refs.insert(2);
  EXPECT_FALSE(node.archived());

  node.active_refs.erase(1);
  EXPECT_TRUE(node.archived());

  node.archived_refs.erase(2);
  EXPECT_FALSE(node.archived());
}

TEST(GvdGraph, AddCorrect) {
  GvdGraph graph(0.1, 1.0);
  graph.add(GlobalIndex(1, 2, 3), 0.1, 2);
  EXPECT_EQ(graph.uncompressed().size(), 1u);
  EXPECT_EQ(graph.compressed().size(), 1u);
  ASSERT_TRUE(graph.get(0));

  // re-adding node doesn't change counts
  graph.add(GlobalIndex(1, 2, 3), 0.1, 2);
  EXPECT_EQ(graph.uncompressed().size(), 1u);
  EXPECT_EQ(graph.compressed().size(), 1u);
  ASSERT_TRUE(graph.get(0));

  // add connected voxel to first one
  graph.add(GlobalIndex(1, 2, 4), 0.1, 2);
  EXPECT_EQ(graph.uncompressed().size(), 2u);
  EXPECT_EQ(graph.compressed().size(), 1u);
  EXPECT_TRUE(hasEdge(graph, 0, 1));
  ASSERT_TRUE(graph.get(1));

  // add new connected voxel connected
  graph.add(GlobalIndex(4, 4, 4), 0.1, 2);
  EXPECT_EQ(graph.uncompressed().size(), 3u);
  EXPECT_EQ(graph.compressed().size(), 2u);
  EXPECT_FALSE(hasEdge(graph, 1, 2));
  EXPECT_FALSE(hasCompressedEdge(graph, 0, 1));
  ASSERT_TRUE(graph.get(2));
}

TEST(GvdGraph, RemoveCorrect) {
  GvdGraph graph(0.1, 1.0);
  fillGraph(graph, {{1, 2, 3}, {1, 2, 4}, {4, 4, 4}});

  // non-existent index does not change counts
  callRemove(graph, {GlobalIndex(10, 10, 5)});
  EXPECT_EQ(graph.uncompressed().size(), 3u);
  EXPECT_EQ(graph.compressed().size(), 2u);

  // support doesn't change for compressed nodes
  callRemove(graph, {GlobalIndex(1, 2, 4)});
  EXPECT_EQ(graph.uncompressed().size(), 2u);
  EXPECT_EQ(graph.compressed().size(), 2u);

  callRemove(graph, {GlobalIndex(1, 2, 3)});
  EXPECT_EQ(graph.uncompressed().size(), 1u);
  EXPECT_EQ(graph.compressed().size(), 1u);

  callRemove(graph, {GlobalIndex(4, 4, 4)});
  EXPECT_EQ(graph.uncompressed().size(), 0u);
  EXPECT_EQ(graph.compressed().size(), 0u);
}

TEST(GvdGraph, ArchiveCorrect) {
  GvdGraph graph(0.1, 1.0);
  fillGraph(graph, {{1, 2, 3}, {1, 2, 4}, {4, 4, 4}});

  // non-existent index does not change counts
  callArchive(graph, {GlobalIndex(10, 10, 5)});
  EXPECT_EQ(graph.uncompressed().size(), 3u);
  EXPECT_EQ(graph.compressed().size(), 2u);

  // archive doesn't change edges
  callArchive(graph, {GlobalIndex(1, 2, 4)});
  EXPECT_EQ(graph.uncompressed().size(), 3u);
  EXPECT_EQ(graph.compressed().size(), 2u);
  EXPECT_TRUE(hasEdge(graph, 0, 1));

  // support doesn't change for compressed nodes
  callArchive(graph, {GlobalIndex(1, 2, 3)});
  EXPECT_EQ(graph.uncompressed().size(), 3u);
  EXPECT_EQ(graph.compressed().size(), 2u);
  EXPECT_TRUE(hasEdge(graph, 0, 1));

  callArchive(graph, {GlobalIndex(4, 4, 4)});
  EXPECT_EQ(graph.uncompressed().size(), 3u);
  EXPECT_EQ(graph.compressed().size(), 2u);
  EXPECT_TRUE(hasEdge(graph, 0, 1));
}

TEST(GvdGraph, DropCorrect) {
  GvdGraph graph(0.1, 1.0);
  fillGraph(graph, {{1, 2, 3}, {1, 2, 4}, {4, 4, 4}});

  graph.dropCompressed(3);
  EXPECT_EQ(graph.uncompressed().size(), 3u);
  EXPECT_EQ(graph.compressed().size(), 2u);

  graph.dropCompressed(1);
  EXPECT_EQ(graph.uncompressed().size(), 2u);
  EXPECT_EQ(graph.compressed().size(), 1u);
  EXPECT_TRUE(hasEdge(graph, 0, 1));

  graph.dropCompressed(0);
  EXPECT_EQ(graph.uncompressed().size(), 0u);
  EXPECT_EQ(graph.compressed().size(), 0u);
}

TEST(GvdGraph, AddWithEdgeAcrossBlocks) {
  GvdGraph graph(0.1, 1.0);
  fillGraph(graph,
            {
                {0, 0, 0},
                {0, 0, 1},
                {0, 1, 0},
                {1, 0, 0},
                {0, 0, -1},
                {0, 1, -1},
                {1, 0, -1},
            });

  EXPECT_EQ(graph.uncompressed().size(), 7u);
  EXPECT_EQ(graph.compressed().size(), 2u);
  EXPECT_TRUE(hasCompressedEdge(graph, 0, 1));
}

TEST(GvdGraph, AddWithClusteringCorrect) {
  GvdGraph graph(0.1, 1.0);
  graph.add(GlobalIndex(0, 0, 0), 0.1, 2);
  graph.add(GlobalIndex(2, 0, 0), 0.1, 2);
  EXPECT_EQ(graph.uncompressed().size(), 2u);
  EXPECT_EQ(graph.compressed().size(), 2u);
  EXPECT_FALSE(hasCompressedEdge(graph, 0, 1));
  graph.add(GlobalIndex(1, 0, 0), 0.1, 2);
  EXPECT_EQ(graph.uncompressed().size(), 3u);
  EXPECT_EQ(graph.compressed().size(), 1u);
}

TEST(GvdGraph, DropWithClusteringCorrect) {
  GvdGraph graph(0.1, 1.0);
  graph.add(GlobalIndex(0, 0, 0), 0.1, 2);
  graph.add(GlobalIndex(1, 0, 0), 0.1, 2);
  graph.add(GlobalIndex(2, 0, 0), 0.1, 2);
  EXPECT_EQ(graph.uncompressed().size(), 3u);
  EXPECT_EQ(graph.compressed().size(), 1u);

  callRemove(graph, {{1, 0, 0}});
  EXPECT_EQ(graph.uncompressed().size(), 2u);
  EXPECT_EQ(graph.compressed().size(), 2u);
}

}  // namespace hydra::places
