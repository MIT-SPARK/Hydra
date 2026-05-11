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
#include <hydra/places/gvd_places/graph_extractor.h>
#include <hydra/places/gvd_places/gvd_parent_tracker.h>
#include <spark_dsg/node_symbol.h>

namespace hydra::places {

using spark_dsg::NodeId;
using spark_dsg::operator""_id;

namespace {

class TestGraphExtractor : public GraphExtractor {
 public:
  explicit TestGraphExtractor(const Config& config, float voxel_size)
      : GraphExtractor(config, voxel_size), gvd_layer(voxel_size, 16) {
    const auto block_index = BlockIndex::Zero();
    auto gvd_block = gvd_layer.allocateBlockPtr(block_index);
    for (size_t i = 0; i < gvd_block->numVoxels(); ++i) {
      auto& voxel = gvd_block->getVoxel(i);
      voxel.distance = 0.3;
      voxel.num_extra_basis = 4;
    }

    tracker.markNewGvdParent(gvd_layer, GlobalIndex(0, 0, 0));
  }

  ~TestGraphExtractor() = default;

  using GraphExtractor::updatePartialGraph;

  void update(const GlobalIndexSet& removed = {}) {
    VoxelIndexChanges changes{{}, removed};
    updateGvdGraph(0, gvd_layer, changes);   // this propagates archives and deletions
    updatePartialGraph(gvd_layer, tracker);  // this builds the graph
  }

  void addNode(uint64_t x, uint64_t y, uint64_t z, double distance, uint8_t basis) {
    const GlobalIndex index(x, y, z);
    gvd_.add(index, distance, basis);
    tracker.parents[index] = {GlobalIndex(0, 0, 0)};
  }

  GvdLayer gvd_layer;
  GvdParentTracker tracker;
};

void checkNode(const GraphExtractor::LocalGraph& graph,
               NodeId node_id,
               const Eigen::Vector3d& p_expected,
               double d_expected,
               uint8_t expected_basis) {
  Eigen::IOFormat fmt(
      Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "; ", "", "", "[", "]");
  const auto attrs = graph.at(node_id);
  ASSERT_TRUE(attrs);
  EXPECT_NEAR((attrs->position - p_expected).norm(), 0.0, 1.0e-9)
      << attrs->position.format(fmt) << " vs. expected " << p_expected.format(fmt);
  EXPECT_EQ(attrs->distance, d_expected);
  EXPECT_EQ(attrs->num_basis_points, expected_basis);
}

size_t numArchived(const GraphExtractor::LocalGraph& graph) {
  size_t num_archived = 0;
  for (const auto& [node_id, node] : graph) {
    if (!node.attrs) {
      ++num_archived;
    }
  }

  return num_archived;
}

}  // namespace

TEST(GraphExtractor, VoxelDeletion) {
  GraphExtractor::Config config;
  config.compression_distance_m = 2.4;
  config.min_node_distance_m = 0.0;
  TestGraphExtractor extractor(config, 1.0);
  const auto& gvd = extractor.gvd();
  const auto& places = extractor.graph();

  extractor.addNode(0, 0, 1, 0.1, 1);
  extractor.addNode(0, 0, 2, 0.2, 4);
  extractor.addNode(0, 0, 3, 0.3, 3);
  extractor.addNode(0, 1, 0, 0.4, 5);
  extractor.addNode(0, 1, 2, 0.5, 5);
  EXPECT_EQ(gvd.uncompressed().size(), 5u);

  extractor.update();

  {  // scope after a normal update: remapping should exist as expected
    const GvdGraph::NodeRemapping expected{{0, 0}, {1, 1}, {2, 1}, {3, 0}, {4, 1}};
    EXPECT_EQ(gvd.remapping(), expected);
    checkNode(places, "p0"_id, Eigen::Vector3d(0.5, 1.5, 0.5), 0.4, 5u);
  }

  extractor.update({{0, 1, 0}});

  EXPECT_EQ(gvd.uncompressed().size(), 4u);
  EXPECT_EQ(numArchived(places), 0u);

  {  // scope after deleting one gvd member of p0: p0 attributes should update
    const GvdGraph::NodeRemapping expected{{0, 0}, {1, 1}, {2, 1}, {4, 1}};
    EXPECT_EQ(gvd.remapping(), expected);
    checkNode(places, "p0"_id, Eigen::Vector3d(0.5, 0.5, 1.5), 0.1, 1u);
  }

  extractor.update({{0, 0, 1}});

  EXPECT_EQ(gvd.uncompressed().size(), 3u);
  EXPECT_EQ(numArchived(places), 0u);
  EXPECT_EQ(places.num_nodes(), 1u);
  EXPECT_FALSE(places.has("p0"_id));

  ASSERT_TRUE(gvd.compressed().count(1));
  EXPECT_TRUE(gvd.compressed().at(1).siblings.empty());
  {
    const GvdGraph::NodeRemapping expected{{1, 1}, {2, 1}, {4, 1}};
    EXPECT_EQ(gvd.remapping(), expected);
  }

  extractor.addNode(0, 0, 1, 0.1, 1);
  extractor.update();
  {  // scope after re-adding one gvd member of p0
    const GvdGraph::NodeRemapping expected{{3, 2}, {1, 1}, {2, 1}, {4, 1}};
    EXPECT_EQ(gvd.remapping(), expected);
    checkNode(places, "p2"_id, Eigen::Vector3d(0.5, 0.5, 1.5), 0.1, 1u);
  }
}

TEST(GraphExtractor, VoxelArchival) {
  GraphExtractor::Config config;
  config.compression_distance_m = 2.4;
  config.min_node_distance_m = 0.0;

  TestGraphExtractor extractor(config, 1.0);
  const auto& gvd = extractor.gvd();
  const auto& places = extractor.graph();

  extractor.addNode(0, 0, 1, 0.1, 1);
  extractor.addNode(0, 0, 2, 0.2, 4);
  extractor.addNode(0, 0, 3, 0.3, 3);
  extractor.addNode(0, 1, 0, 0.4, 5);
  extractor.addNode(0, 1, 2, 0.5, 5);
  EXPECT_EQ(gvd.uncompressed().size(), 5u);

  extractor.update();

  {  // scope after a normal update: remapping should exist as expected
    const GvdGraph::NodeRemapping expected{{0, 0}, {1, 1}, {2, 1}, {3, 0}, {4, 1}};
    EXPECT_EQ(gvd.remapping(), expected);
    checkNode(places, "p0"_id, Eigen::Vector3d(0.5, 1.5, 0.5), 0.4, 5u);
  }

  // this should force the voxel attributes to flip to 0, 0, 1 for p0
  extractor.addNode(0, 0, 1, 0.1, 6);
  extractor.archiveIndex(GlobalIndex(0, 0, 1));
  extractor.update();

  EXPECT_EQ(gvd.uncompressed().size(), 5u);
  EXPECT_EQ(numArchived(places), 0u);

  {  // scope after deleting one gvd member of p0: p0 attributes should update
    const GvdGraph::NodeRemapping expected{{0, 0}, {1, 1}, {2, 1}, {3, 0}, {4, 1}};
    EXPECT_EQ(gvd.remapping(), expected);
    checkNode(places, "p0"_id, Eigen::Vector3d(0.5, 0.5, 1.5), 0.1, 6u);
  }

  extractor.archiveIndex(GlobalIndex(0, 1, 0));
  extractor.update();
  EXPECT_EQ(numArchived(places), 1u);

  extractor.update({{0, 0, 2}});
  EXPECT_EQ(numArchived(places), 1u);
  EXPECT_EQ(gvd.compressed().size(), 2u);
  EXPECT_EQ(gvd.uncompressed().size(), 4u);
}

}  // namespace hydra::places
