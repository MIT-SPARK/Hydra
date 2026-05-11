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

void callRemove(GvdGraph& graph, const GlobalIndexSet& indices) {
  graph.remove(indices);
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

// test that compressed nodes have correct archival flag logic
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

// test that add handles compression and edges correctly
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

// test that removal drops uncompressed and compressed nodes correctly
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

// test that archival doesn't change graph
TEST(GvdGraph, ArchiveCorrect) {
  GvdGraph graph(0.1, 1.0);
  fillGraph(graph, {{1, 2, 3}, {1, 2, 4}, {4, 4, 4}});

  // non-existent index does not change counts
  graph.archive(GlobalIndex(10, 10, 5));
  EXPECT_EQ(graph.uncompressed().size(), 3u);
  EXPECT_EQ(graph.compressed().size(), 2u);

  // archive doesn't change edges
  graph.archive(GlobalIndex(1, 2, 4));
  EXPECT_EQ(graph.uncompressed().size(), 3u);
  EXPECT_EQ(graph.compressed().size(), 2u);
  EXPECT_TRUE(hasEdge(graph, 0, 1));

  // support doesn't change for compressed nodes
  graph.archive(GlobalIndex(1, 2, 3));
  EXPECT_EQ(graph.uncompressed().size(), 3u);
  EXPECT_EQ(graph.compressed().size(), 2u);
  EXPECT_TRUE(hasEdge(graph, 0, 1));

  graph.archive(GlobalIndex(4, 4, 4));
  EXPECT_EQ(graph.uncompressed().size(), 3u);
  EXPECT_EQ(graph.compressed().size(), 2u);
  EXPECT_TRUE(hasEdge(graph, 0, 1));
}

// check that deleting compressed nodes deleted relevant uncompressed nodes
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

// test that voxels across blocks result in connections in the compressed graph
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

// test that adding a connecting voxel merges clusters
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

// test that siblings and compression remapping is correct when adding nodes between
// blocks (this is somewhat redundant with other tests as a result of being ported from
// the graph extractor test suite)
TEST(GvdGraph, CompressionInformationCorrect) {
  GvdGraph gvd(0.1, 1.0);
  EXPECT_TRUE(gvd.uncompressed().empty());

  gvd.add(GlobalIndex(0, 0, 1), 0.1, 1);
  gvd.add(GlobalIndex(0, 0, 2), 0.1, 1);
  gvd.add(GlobalIndex(0, 0, 4), 0.1, 1);
  gvd.add(GlobalIndex(0, 1, 0), 0.1, 1);
  EXPECT_EQ(gvd.uncompressed().size(), 4u);
  EXPECT_TRUE(gvd.get(0));
  EXPECT_TRUE(gvd.get(1));
  EXPECT_TRUE(gvd.get(2));
  EXPECT_TRUE(gvd.get(3));

  {
    const auto& node = gvd.uncompressed().at(0);
    std::set<uint64_t> expected_siblings{1, 3};
    EXPECT_EQ(node.siblings, expected_siblings);
  }

  {
    const auto& node = gvd.uncompressed().at(1);
    std::set<uint64_t> expected_siblings{0};
    EXPECT_EQ(node.siblings, expected_siblings);
  }

  {
    const auto& node = gvd.uncompressed().at(3);
    std::set<uint64_t> expected_siblings{0};
    EXPECT_EQ(node.siblings, expected_siblings);
  }

  GvdGraph::NodeRemapping expected_remapping{{0, 0}, {1, 0}, {2, 1}, {3, 0}};
  EXPECT_EQ(gvd.remapping(), expected_remapping);
  ASSERT_TRUE(gvd.compressed().count(0));
  ASSERT_TRUE(gvd.compressed().count(1));

  {
    const std::set<uint64_t> expected_active_refs{0, 1, 3};
    const auto& info = gvd.compressed().at(0);
    EXPECT_TRUE(info.siblings.empty());
    EXPECT_EQ(info.active_refs, expected_active_refs);
    EXPECT_TRUE(info.archived_refs.empty());
  }

  {
    const std::set<uint64_t> expected_active_refs{2};
    const auto& info = gvd.compressed().at(1);
    EXPECT_TRUE(info.siblings.empty());
    EXPECT_EQ(info.active_refs, expected_active_refs);
    EXPECT_TRUE(info.archived_refs.empty());
  }
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

// test that readding a node updates the underlying gvd information
// somewhat redundant with add due to port from graph extractor
TEST(GvdGraph, NodeUpdatesCorrect) {
  GvdGraph gvd(0.1, 1.0);
  EXPECT_TRUE(gvd.uncompressed().empty());

  gvd.add(GlobalIndex(0, 0, 1), 0.1, 1);
  gvd.add(GlobalIndex(0, 0, 2), 0.2, 2);
  EXPECT_EQ(gvd.uncompressed().size(), 2u);
  EXPECT_TRUE(gvd.get(0));
  EXPECT_TRUE(gvd.get(1));
  EXPECT_FALSE(gvd.get(2));

  {
    const auto info = gvd.get(0);
    ASSERT_TRUE(info != nullptr);
    EXPECT_LT((info->position - Eigen::Vector3f(0.05, 0.05, 0.15)).norm(), 1.0e-9);
    EXPECT_EQ(info->distance, 0.1);
    EXPECT_EQ(info->num_basis_points, 1u);
  }

  {
    const auto info = gvd.get(1);
    ASSERT_TRUE(info != nullptr);
    EXPECT_LT((info->position - Eigen::Vector3f(0.05, 0.05, 0.25)).norm(), 1.0e-9);
    EXPECT_EQ(info->distance, 0.2);
    EXPECT_EQ(info->num_basis_points, 2u);
  }

  gvd.add(GlobalIndex(0, 0, 2), 0.3, 2);
  EXPECT_EQ(gvd.uncompressed().size(), 2u);
  {
    const auto info = gvd.get(1);
    ASSERT_TRUE(info != nullptr);
    EXPECT_LT((info->position - Eigen::Vector3f(0.05, 0.05, 0.25)).norm(), 1.0e-9);
    EXPECT_EQ(info->distance, 0.3);
    EXPECT_EQ(info->num_basis_points, 2u);
  }
}

// not sure what this was doing in the graph extractor test suite,
// should consider dropping at some point
TEST(GvdGraph, UniformGvd) {
  GvdGraph gvd(0.1, 0.5);
  EXPECT_TRUE(gvd.uncompressed().empty());

  gvd.add(GlobalIndex(0, 0, 0), 0.1, 1);
  gvd.add(GlobalIndex(1, 0, 0), 0.1, 1);
  gvd.add(GlobalIndex(0, 1, 0), 0.1, 1);
  gvd.add(GlobalIndex(0, 0, 1), 0.1, 1);
  gvd.add(GlobalIndex(2, 0, 0), 0.1, 1);
  gvd.add(GlobalIndex(0, 2, 0), 0.1, 1);
  gvd.add(GlobalIndex(0, 0, 2), 0.1, 1);
  gvd.add(GlobalIndex(3, 0, 0), 0.1, 1);
  gvd.add(GlobalIndex(0, 3, 0), 0.1, 1);
  gvd.add(GlobalIndex(0, 0, 3), 0.1, 1);
  EXPECT_EQ(gvd.uncompressed().size(), 10u);
  EXPECT_EQ(gvd.compressed().size(), 1u);
}

// adapted from graph extractor tests, checks if archived flag updates after voxel
// deletion
TEST(GvdGraph, ArchiveAndDeleteCorrect) {
  GvdGraph gvd(0.1, 3.0);
  EXPECT_TRUE(gvd.uncompressed().empty());

  gvd.add(GlobalIndex(0, 0, 1), 0.1, 1);
  gvd.add(GlobalIndex(0, 0, 3), 0.2, 4);
  gvd.add(GlobalIndex(0, 0, 4), 0.3, 3);
  gvd.add(GlobalIndex(0, 1, 0), 0.4, 5);
  gvd.add(GlobalIndex(0, 0, 5), 0.5, 5);
  EXPECT_EQ(gvd.uncompressed().size(), 5u);

  {
    const GvdGraph::NodeRemapping expected{{0, 0}, {1, 1}, {2, 1}, {3, 0}, {4, 1}};
    EXPECT_EQ(gvd.remapping(), expected);
  }

  gvd.archive(GlobalIndex(0, 0, 1));
  callRemove(gvd, {{0, 1, 0}});

  EXPECT_EQ(gvd.uncompressed().size(), 4u);
  ASSERT_TRUE(gvd.compressed().count(0));
  EXPECT_TRUE(gvd.compressed().at(0).archived());

  {
    const GvdGraph::NodeRemapping expected{{0, 0}, {1, 1}, {2, 1}, {4, 1}};
    EXPECT_EQ(gvd.remapping(), expected);
  }
}

// mostly duplicate with previous test due to port from graph extractor,
// but does check that sorting order of components is correct
TEST(GvdGraph, SingleVoxelDeletion) {
  GvdGraph gvd(0.1, 3.0);
  EXPECT_TRUE(gvd.uncompressed().empty());

  gvd.add(GlobalIndex(0, 0, 2), 0.2, 2);
  gvd.add(GlobalIndex(0, 0, 3), 0.3, 3);
  gvd.add(GlobalIndex(0, 0, 4), 0.4, 4);
  gvd.add(GlobalIndex(1, 0, 2), 0.5, 3);
  EXPECT_EQ(gvd.uncompressed().size(), 4u);

  {
    const GvdGraph::NodeRemapping expected_remapping{{0, 0}, {1, 0}, {2, 0}, {3, 0}};
    EXPECT_EQ(gvd.remapping(), expected_remapping);
  }

  callRemove(gvd, {{0, 0, 3}});
  EXPECT_EQ(gvd.uncompressed().size(), 3u);

  {
    const GvdGraph::NodeRemapping expected_remapping{{0, 0}, {2, 1}, {3, 0}};
    EXPECT_EQ(gvd.remapping(), expected_remapping);
  }
}

// check that merge policy and compressed node works together
TEST(GvdGraph, AttributeAssignmentOneToOne) {
  GvdGraph gvd(0.1, 1.0);
  EXPECT_TRUE(gvd.uncompressed().empty());

  gvd.add(GlobalIndex(0, 0, 1), 0.1, 1);
  gvd.add(GlobalIndex(0, 0, 2), 0.2, 5);
  gvd.add(GlobalIndex(0, 0, 4), 0.3, 3);
  gvd.add(GlobalIndex(0, 1, 0), 0.4, 4);
  EXPECT_EQ(gvd.uncompressed().size(), 4u);
  EXPECT_EQ(gvd.compressed().size(), 2u);

  const BasisPointMergePolicy policy;
  const auto info_0 = gvd.getCompressed(0, policy);
  ASSERT_TRUE(info_0.info);
  const auto info_1 = gvd.getCompressed(1, policy);
  ASSERT_TRUE(info_1.info);

  EXPECT_EQ(info_0.info->distance, 0.2);
  EXPECT_EQ(info_0.info->num_basis_points, 5u);
  EXPECT_EQ(info_1.info->distance, 0.3);
  EXPECT_EQ(info_1.info->num_basis_points, 3u);
}

// check that merge policy and compressed node works together
TEST(GvdGraph, AttributeAssignmentManyToOne) {
  GvdGraph gvd(0.1, 3.0);
  gvd.add(GlobalIndex(0, 0, 1), 0.1, 1);
  gvd.add(GlobalIndex(0, 0, 2), 0.2, 4);
  gvd.add(GlobalIndex(0, 0, 3), 0.3, 3);
  gvd.add(GlobalIndex(0, 1, 0), 0.4, 5);
  gvd.add(GlobalIndex(0, 0, 4), 0.5, 5);
  gvd.archive(GlobalIndex(0, 0, 4));
  gvd.archive(GlobalIndex(0, 0, 3));
  EXPECT_EQ(gvd.uncompressed().size(), 5u);
  EXPECT_EQ(gvd.compressed().size(), 1u);

  GvdGraph::NodeRemapping expected_remapping{{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}};
  EXPECT_EQ(gvd.remapping(), expected_remapping);

  const BasisPointMergePolicy policy;
  const auto info_0 = gvd.getCompressed(0, policy);

  // note that this relies on active being checked before archived
  ASSERT_TRUE(info_0.info);
  EXPECT_EQ(info_0.info->distance, 0.4);
  EXPECT_EQ(info_0.info->num_basis_points, 5u);
}

// test that merge clusters doesn't throw an error
TEST(GvdGraph, MergeCleanupCorrect) {
  GvdGraph graph(0.1, 1.0);
  graph.add(GlobalIndex(0, 0, 0), 0.1, 2);
  graph.add(GlobalIndex(2, 0, 0), 0.1, 2);
  graph.add(GlobalIndex(0, -1, 0), 0.1, 2);
  graph.add(GlobalIndex(2, -1, 0), 0.1, 2);
  EXPECT_EQ(graph.compressed().size(), 4u);

  graph.add(GlobalIndex(1, 0, 0), 0.1, 2);
  EXPECT_EQ(graph.compressed().size(), 3u);

  graph.add(GlobalIndex(1, -1, 0), 0.1, 2);
  EXPECT_EQ(graph.compressed().size(), 2u);

  ASSERT_TRUE(graph.compressed().count(0));
  ASSERT_TRUE(graph.compressed().count(2));
  std::set<uint64_t> siblings{2};
  EXPECT_EQ(graph.compressed().at(0).siblings, siblings);
  siblings = {0};
  EXPECT_EQ(graph.compressed().at(2).siblings, siblings);
}

// test that splitting stays contained to a single block
TEST(GvdGraph, SplittingCorrect) {
  GvdGraph graph(0.1, 1.0);
  graph.add(GlobalIndex(0, 0, 0), 0.1, 2);
  graph.add(GlobalIndex(0, 0, 1), 0.1, 2);
  graph.add(GlobalIndex(0, 0, 2), 0.1, 2);
  graph.add(GlobalIndex(2, 0, 0), 0.1, 2);
  graph.add(GlobalIndex(2, 0, 1), 0.1, 2);
  graph.add(GlobalIndex(2, 0, 2), 0.1, 2);
  graph.add(GlobalIndex(0, -1, 0), 0.1, 2);
  graph.add(GlobalIndex(1, -1, 0), 0.1, 2);
  graph.add(GlobalIndex(2, -1, 0), 0.1, 2);
  EXPECT_EQ(graph.compressed().size(), 3u);

  callRemove(graph, {{1, -1, 0}});  // trigger splitting
  EXPECT_EQ(graph.compressed().size(), 4u);
  ASSERT_TRUE(graph.compressed().count(0));
  ASSERT_TRUE(graph.compressed().count(1));
  ASSERT_TRUE(graph.compressed().count(2));
  ASSERT_TRUE(graph.compressed().count(3));

  std::set<uint64_t> refs{0, 1, 2};
  EXPECT_EQ(graph.compressed().at(0).active_refs, refs);
  EXPECT_TRUE(graph.compressed().at(0).archived_refs.empty());
  refs = {3, 4, 5};
  EXPECT_EQ(graph.compressed().at(1).active_refs, refs);
  EXPECT_TRUE(graph.compressed().at(1).archived_refs.empty());
  refs = {6};
  EXPECT_EQ(graph.compressed().at(2).active_refs, refs);
  EXPECT_TRUE(graph.compressed().at(2).archived_refs.empty());
  refs = {8};
  EXPECT_EQ(graph.compressed().at(3).active_refs, refs);
  EXPECT_TRUE(graph.compressed().at(3).archived_refs.empty());
}

// try to replicate weird archival behavior
TEST(GvdGraph, ArchiveBug) {
  GvdGraph graph(0.1, 1.0);
  graph.add(GlobalIndex(0, -1, 0), 0.1, 2);
  EXPECT_EQ(graph.uncompressed().size(), 1u);

  graph.archive(GlobalIndex(0, -1, 0));
  graph.add(GlobalIndex(0, -1, 0), 0.3, 3);
  EXPECT_EQ(graph.uncompressed().size(), 2u);
  {
    const auto n0 = graph.get(0);
    const auto n1 = graph.get(1);
    ASSERT_TRUE(n0);
    ASSERT_TRUE(n1);
    EXPECT_EQ(n0->index, n1->index);
    EXPECT_NE(n0->num_basis_points, n1->num_basis_points);
    EXPECT_TRUE(graph.uncompressed().at(0).archived);
    EXPECT_FALSE(graph.uncompressed().at(1).archived);
  }

  graph.remove({{0, -1, 0}});
  graph.add(GlobalIndex(0, -1, 0), 0.3, 3);
  EXPECT_EQ(graph.uncompressed().size(), 2u);
  {
    const auto n0 = graph.get(0);
    const auto n1 = graph.get(1);
    ASSERT_TRUE(n0);
    ASSERT_TRUE(n1);
    EXPECT_EQ(n0->index, n1->index);
    EXPECT_NE(n0->num_basis_points, n1->num_basis_points);
    EXPECT_TRUE(graph.uncompressed().at(0).archived);
    EXPECT_FALSE(graph.uncompressed().at(1).archived);
  }

  graph.dropCompressed(2);
  EXPECT_EQ(graph.uncompressed().size(), 1u);
  EXPECT_TRUE(graph.uncompressed().count(0));

  graph.add(GlobalIndex(0, -1, 0), 0.3, 3);
  EXPECT_EQ(graph.uncompressed().size(), 2u);
  {
    const auto n0 = graph.get(0);
    const auto n1 = graph.get(1);
    ASSERT_TRUE(n0);
    ASSERT_TRUE(n1);
    EXPECT_EQ(n0->index, n1->index);
    EXPECT_NE(n0->num_basis_points, n1->num_basis_points);
    EXPECT_TRUE(graph.uncompressed().at(0).archived);
    EXPECT_FALSE(graph.uncompressed().at(1).archived);
  }
}

}  // namespace hydra::places
