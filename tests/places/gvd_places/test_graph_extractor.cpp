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

namespace hydra::places {

using spark_dsg::operator"" _id;
using spark_dsg::NodeId;
using spark_dsg::PlaceNodeAttributes;

namespace {

class TestGraphExtractor : public GraphExtractor {
 public:
  explicit TestGraphExtractor(const Config& config) : GraphExtractor(config) {}
  ~TestGraphExtractor() = default;

  void assignCompressedNodeAttributes(bool clear_active = true) {
    GraphExtractor::assignCompressedNodeAttributes();
    if (clear_active) {
      compressed_.updated.clear();
    }
  }

  const CompressedGraph& compressed() const { return compressed_; }

  using GraphExtractor::clearArchived;
  using GraphExtractor::to_archive_;
  using GraphExtractor::updateGvdGraph;

  void setGvdNode(uint64_t x, uint64_t y, uint64_t z, double distance, uint8_t basis) {
    const GlobalIndex index(x, y, z);
    const Eigen::Vector3f position(x, y, z);

    auto iter = index_id_map_.find(index);
    if (iter == index_id_map_.end()) {
      const auto next_id = gvd_.addNode(position, index);
      iter = index_id_map_.emplace(index, next_id).first;
    }

    auto& info = *gvd_.getNode(iter->second);
    info.distance = distance;
    info.num_basis_points = basis;
  }
};

using Remapping = std::unordered_map<uint64_t, uint64_t>;
using PlacesGraph = PartialGraph<PlaceNodeAttributes>;

void checkNode(const PlacesGraph& graph,
               NodeId node_id,
               const Eigen::Vector3d& p_expected,
               double d_expected,
               uint8_t expected_basis) {
  const auto attrs = graph.at(node_id);
  ASSERT_TRUE(attrs);
  EXPECT_NEAR((attrs->position - p_expected).norm(), 0.0, 1.0e-9);
  EXPECT_EQ(attrs->distance, d_expected);
  EXPECT_EQ(attrs->num_basis_points, expected_basis);
}

}  // namespace

class GraphExtractorFixture : public ::testing::Test {
 public:
  GraphExtractorFixture() : ::testing::Test(), gvd_layer(1.0, 16) {}
  virtual ~GraphExtractorFixture() = default;

  virtual void SetUp() override {
    const auto block_index = BlockIndex::Zero();
    auto gvd_block = gvd_layer.allocateBlockPtr(block_index);
    for (size_t i = 0; i < gvd_block->numVoxels(); ++i) {
      auto& voxel = gvd_block->getVoxel(i);
      voxel.distance = 0.3;
      voxel.num_extra_basis = 4;
    }
  }

  GvdLayer gvd_layer;
};

TEST_F(GraphExtractorFixture, TestUpdateNode) {
  GraphExtractor::Config config;
  TestGraphExtractor extractor(config);
  const auto& gvd = extractor.gvd_graph();

  EXPECT_TRUE(gvd.empty());

  // this is just a convenience wrapper around updateNode
  extractor.setGvdNode(0, 0, 1, 0.1, 1);
  extractor.setGvdNode(0, 0, 2, 0.2, 2);

  EXPECT_EQ(gvd.nodes().size(), 2u);
  EXPECT_TRUE(gvd.hasNode(0));
  EXPECT_TRUE(gvd.hasNode(1));
  EXPECT_FALSE(gvd.hasNode(2));

  {
    const auto info = gvd.getNode(0);
    ASSERT_TRUE(info != nullptr);
    EXPECT_NEAR((info->position - Eigen::Vector3f(0, 0, 1)).norm(), 0.0, 1.0e-9);
    EXPECT_EQ(info->distance, 0.1);
    EXPECT_EQ(info->num_basis_points, 1u);
  }

  {
    const auto info = gvd.getNode(1);
    ASSERT_TRUE(info != nullptr);
    EXPECT_NEAR((info->position - Eigen::Vector3f(0, 0, 2)).norm(), 0.0, 1.0e-9);
    EXPECT_EQ(info->distance, 0.2);
    EXPECT_EQ(info->num_basis_points, 2u);
  }

  extractor.setGvdNode(0, 0, 2, 0.3, 2);
  EXPECT_EQ(gvd.nodes().size(), 2u);
  {
    const auto info = gvd.getNode(1);
    ASSERT_TRUE(info != nullptr);
    EXPECT_NEAR((info->position - Eigen::Vector3f(0, 0, 2)).norm(), 0.0, 1.0e-9);
    EXPECT_EQ(info->distance, 0.3);
    EXPECT_EQ(info->num_basis_points, 2u);
  }
}

TEST_F(GraphExtractorFixture, testUpdateGraph) {
  GraphExtractor::Config config;
  config.min_node_distance_m = 0.0;
  TestGraphExtractor extractor(config);
  const auto& gvd = extractor.gvd_graph();

  EXPECT_TRUE(gvd.empty());

  extractor.setGvdNode(0, 0, 1, 0.1, 1);
  extractor.setGvdNode(0, 0, 2, 0.1, 1);
  extractor.setGvdNode(0, 0, 4, 0.1, 1);
  extractor.setGvdNode(0, 1, 0, 0.1, 1);
  EXPECT_EQ(gvd.nodes().size(), 4u);

  // we also add 0, 0, 5 for test coverage (to simulate pruning)
  IndexVoxelQueue updated{{GlobalIndex(0, 0, 1), nullptr},
                          {GlobalIndex(0, 0, 2), nullptr},
                          {GlobalIndex(0, 1, 0), nullptr},
                          {GlobalIndex(0, 0, 4), nullptr},
                          {GlobalIndex(0, 0, 5), nullptr}};

  extractor.updateGvdGraph(gvd_layer, updated, 0);

  EXPECT_EQ(gvd.nodes().size(), 3u);
  EXPECT_TRUE(gvd.hasNode(0));
  EXPECT_TRUE(gvd.hasNode(1));
  EXPECT_FALSE(gvd.hasNode(2));
  EXPECT_TRUE(gvd.hasNode(3));

  {
    const auto info = gvd.getNode(0);
    std::set<uint64_t> expected_siblings{1, 3};
    EXPECT_EQ(info->siblings, expected_siblings);
  }

  {
    const auto info = gvd.getNode(1);
    std::set<uint64_t> expected_siblings{0};
    EXPECT_EQ(info->siblings, expected_siblings);
  }

  {
    const auto info = gvd.getNode(3);
    std::set<uint64_t> expected_siblings{0};
    EXPECT_EQ(info->siblings, expected_siblings);
  }

  Remapping expected_remapping{{0, 0}, {1, 1}, {3, 2}};
  EXPECT_EQ(extractor.compressed().remapping, expected_remapping);

  ASSERT_TRUE(extractor.compressed().nodes.count(0));
  ASSERT_TRUE(extractor.compressed().nodes.count(1));
  ASSERT_TRUE(extractor.compressed().nodes.count(2));

  {
    const std::set<uint64_t> expected_siblings{1, 2};
    const std::set<uint64_t> expected_active_refs{0};
    const auto& info = extractor.compressed().nodes.at(0);
    EXPECT_EQ(info.siblings, expected_siblings);
    EXPECT_EQ(info.active_refs, expected_active_refs);
    EXPECT_TRUE(info.archived_refs.empty());
  }

  {
    const std::set<uint64_t> expected_siblings{0};
    const std::set<uint64_t> expected_active_refs{1};
    const auto& info = extractor.compressed().nodes.at(1);
    EXPECT_EQ(info.siblings, expected_siblings);
    EXPECT_EQ(info.active_refs, expected_active_refs);
    EXPECT_TRUE(info.archived_refs.empty());
  }

  {
    const std::set<uint64_t> expected_siblings{0};
    const std::set<uint64_t> expected_active_refs{3};
    const auto& info = extractor.compressed().nodes.at(2);
    EXPECT_EQ(info.siblings, expected_siblings);
    EXPECT_EQ(info.active_refs, expected_active_refs);
    EXPECT_TRUE(info.archived_refs.empty());
  }
}

TEST_F(GraphExtractorFixture, testAttributeAssignmentOneToOne) {
  GraphExtractor::Config config;
  config.min_node_distance_m = 0.0;

  TestGraphExtractor extractor(config);
  const auto& gvd = extractor.gvd_graph();

  EXPECT_TRUE(gvd.empty());

  extractor.setGvdNode(0, 0, 1, 0.1, 1);
  extractor.setGvdNode(0, 0, 2, 0.2, 2);
  extractor.setGvdNode(0, 0, 4, 0.3, 3);
  extractor.setGvdNode(0, 1, 0, 0.4, 4);
  EXPECT_EQ(gvd.nodes().size(), 4u);

  IndexVoxelQueue updated{{GlobalIndex(0, 0, 1), nullptr},
                          {GlobalIndex(0, 0, 2), nullptr},
                          {GlobalIndex(0, 1, 0), nullptr},
                          {GlobalIndex(0, 0, 4), nullptr}};

  extractor.updateGvdGraph(gvd_layer, updated, 0);
  extractor.assignCompressedNodeAttributes();

  const auto& places = extractor.graph();
  EXPECT_EQ(places.num_nodes(), 3u);

  ASSERT_TRUE(places.has("p0"_id));
  ASSERT_TRUE(places.has("p1"_id));
  ASSERT_TRUE(places.has("p2"_id));
  checkNode(places, "p0"_id, Eigen::Vector3d(0, 0, 1), 0.1, 1u);
  checkNode(places, "p1"_id, Eigen::Vector3d(0, 0, 2), 0.2, 2u);
  checkNode(places, "p2"_id, Eigen::Vector3d(0, 1, 0), 0.4, 4u);
}

TEST_F(GraphExtractorFixture, testAttributeAssignmentManyToOne) {
  GraphExtractor::Config config;
  config.min_node_distance_m = 0.1;
  config.compression_distance_m = 3.0;
  TestGraphExtractor extractor(config);
  const auto& gvd = extractor.gvd_graph();

  EXPECT_TRUE(gvd.empty());

  extractor.setGvdNode(0, 0, 1, 0.1, 1);
  extractor.setGvdNode(0, 0, 2, 0.2, 4);
  extractor.setGvdNode(0, 0, 3, 0.3, 3);
  extractor.setGvdNode(0, 1, 0, 0.4, 5);
  extractor.setGvdNode(0, 0, 4, 0.5, 5);
  EXPECT_EQ(gvd.nodes().size(), 5u);

  IndexVoxelQueue updated{{GlobalIndex(0, 0, 1), nullptr},
                          {GlobalIndex(0, 0, 2), nullptr},
                          {GlobalIndex(0, 0, 3), nullptr},
                          {GlobalIndex(0, 1, 0), nullptr},
                          {GlobalIndex(0, 0, 4), nullptr}};

  extractor.updateGvdGraph(gvd_layer, updated, 0);
  extractor.assignCompressedNodeAttributes();

  const auto& places = extractor.graph();
  EXPECT_EQ(places.num_nodes(), 2u);
  ASSERT_TRUE(places.has("p0"_id));
  ASSERT_TRUE(places.has("p1"_id));
  checkNode(places, "p0"_id, Eigen::Vector3d(0, 1, 0), 0.4, 5u);
  checkNode(places, "p1"_id, Eigen::Vector3d(0, 0, 4), 0.5, 5u);

  Remapping expected_remapping{{0, 0}, {1, 1}, {2, 1}, {3, 0}, {4, 1}};
  EXPECT_EQ(extractor.compressed().remapping, expected_remapping);
}

// show that isolated voxels deletion gets propagated to compressed graph
TEST_F(GraphExtractorFixture, testSingleVoxelDeletion) {
  GraphExtractor::Config config;
  config.compression_distance_m = 3.0;
  config.min_node_distance_m = 0.1;

  TestGraphExtractor extractor(config);
  const auto& gvd = extractor.gvd_graph();
  const auto& places = extractor.graph();
  EXPECT_TRUE(gvd.empty());

  extractor.setGvdNode(0, 0, 2, 0.2, 2);
  extractor.setGvdNode(0, 0, 3, 0.3, 3);
  extractor.setGvdNode(0, 0, 4, 0.4, 4);
  extractor.setGvdNode(1, 0, 2, 0.5, 3);
  EXPECT_EQ(gvd.nodes().size(), 4u);

  IndexVoxelQueue updated{{GlobalIndex(0, 0, 2), nullptr},
                          {GlobalIndex(0, 0, 3), nullptr},
                          {GlobalIndex(0, 0, 4), nullptr},
                          {GlobalIndex(1, 0, 2), nullptr}};

  extractor.updateGvdGraph(gvd_layer, updated, 0);
  extractor.assignCompressedNodeAttributes();

  {  // scope after a normal update: remapping should exist as expected
    const Remapping expected_remapping{{0, 0}, {1, 0}, {2, 0}, {3, 0}};
    EXPECT_EQ(extractor.compressed().remapping, expected_remapping);
    checkNode(places, "p0"_id, Eigen::Vector3d(0, 0, 4), 0.4, 4u);
  }

  extractor.clearIndex(GlobalIndex(0, 0, 3));
  // technically we only need to update (0, 0, 4)
  extractor.updateGvdGraph(gvd_layer, updated, 0);
  extractor.assignCompressedNodeAttributes();

  EXPECT_EQ(gvd.nodes().size(), 2u);
  EXPECT_TRUE(extractor.to_archive_.empty());

  {  // scope after two gvd nodes are cleared: p0 attributes should update
    const Remapping expected_remapping{{0, 0}, {3, 0}};
    EXPECT_EQ(extractor.compressed().remapping, expected_remapping);
    checkNode(places, "p0"_id, Eigen::Vector3d(1, 0, 2), 0.5, 3u);
  }
}

TEST_F(GraphExtractorFixture, testVoxelDeletion) {
  GraphExtractor::Config config;
  config.compression_distance_m = 3.0;
  config.min_node_distance_m = 0.0;
  TestGraphExtractor extractor(config);
  const auto& gvd = extractor.gvd_graph();
  const auto& places = extractor.graph();

  EXPECT_TRUE(gvd.empty());

  extractor.setGvdNode(0, 0, 1, 0.1, 1);
  extractor.setGvdNode(0, 0, 2, 0.2, 4);
  extractor.setGvdNode(0, 0, 3, 0.3, 3);
  extractor.setGvdNode(0, 1, 0, 0.4, 5);
  extractor.setGvdNode(0, 0, 4, 0.5, 5);
  EXPECT_EQ(gvd.nodes().size(), 5u);

  IndexVoxelQueue updated{{GlobalIndex(0, 0, 1), nullptr},
                          {GlobalIndex(0, 0, 2), nullptr},
                          {GlobalIndex(0, 0, 3), nullptr},
                          {GlobalIndex(0, 1, 0), nullptr},
                          {GlobalIndex(0, 0, 4), nullptr}};

  extractor.updateGvdGraph(gvd_layer, updated, 0);
  extractor.assignCompressedNodeAttributes();

  {  // scope after a normal update: remapping should exist as expected
    const Remapping expected_remapping{{0, 0}, {1, 1}, {2, 1}, {3, 0}, {4, 1}};
    EXPECT_EQ(extractor.compressed().remapping, expected_remapping);
    checkNode(places, "p0"_id, Eigen::Vector3d(0, 1, 0), 0.4, 5u);
  }

  extractor.clearIndex(GlobalIndex(0, 1, 0));
  extractor.assignCompressedNodeAttributes();

  EXPECT_EQ(gvd.nodes().size(), 4u);
  EXPECT_TRUE(extractor.to_archive_.empty());

  {  // scope after deleting one gvd member of p0: p0 attributes should update
    const Remapping expected_remapping{{0, 0}, {1, 1}, {2, 1}, {4, 1}};
    EXPECT_EQ(extractor.compressed().remapping, expected_remapping);
    checkNode(places, "p0"_id, Eigen::Vector3d(0, 0, 1), 0.1, 1u);
  }

  extractor.clearIndex(GlobalIndex(0, 0, 1));
  EXPECT_EQ(gvd.nodes().size(), 3u);
  EXPECT_TRUE(extractor.to_archive_.empty());
  EXPECT_TRUE(extractor.compressed().updated.empty());
  EXPECT_EQ(places.num_nodes(), 1u);
  EXPECT_FALSE(places.has("p0"_id));

  EXPECT_TRUE(extractor.compressed().nodes.at(1).siblings.empty());
  EXPECT_EQ(extractor.compressed().nodes.size(), 1u);
  EXPECT_EQ(extractor.compressed().index_map.size(), 1u);
  EXPECT_EQ(extractor.compressed().id_map.size(), 1u);
  {
    const Remapping expected_remapping{{1, 1}, {2, 1}, {4, 1}};
    EXPECT_EQ(extractor.compressed().remapping, expected_remapping);
  }

  extractor.setGvdNode(0, 0, 1, 0.1, 1);
  updated = {{GlobalIndex(0, 0, 1), nullptr}};
  extractor.updateGvdGraph(gvd_layer, updated, 0);
  extractor.assignCompressedNodeAttributes();
  {  // scope after re-adding one gvd member of p0
    const Remapping expected_remapping{{3, 2}, {1, 1}, {2, 1}, {4, 1}};
    EXPECT_EQ(extractor.compressed().remapping, expected_remapping);
    checkNode(places, "p2"_id, Eigen::Vector3d(0, 0, 1), 0.1, 1u);
  }
}

TEST_F(GraphExtractorFixture, testVoxelArchival) {
  GraphExtractor::Config config;
  config.compression_distance_m = 3.0;
  config.min_node_distance_m = 0.0;
  TestGraphExtractor extractor(config);
  const auto& gvd = extractor.gvd_graph();
  const auto& places = extractor.graph();

  EXPECT_TRUE(gvd.empty());

  extractor.setGvdNode(0, 0, 1, 0.1, 1);
  extractor.setGvdNode(0, 0, 2, 0.2, 4);
  extractor.setGvdNode(0, 0, 3, 0.3, 3);
  extractor.setGvdNode(0, 1, 0, 0.4, 5);
  extractor.setGvdNode(0, 0, 4, 0.5, 5);
  EXPECT_EQ(gvd.nodes().size(), 5u);

  IndexVoxelQueue updated{{GlobalIndex(0, 0, 1), nullptr},
                          {GlobalIndex(0, 0, 2), nullptr},
                          {GlobalIndex(0, 0, 3), nullptr},
                          {GlobalIndex(0, 1, 0), nullptr},
                          {GlobalIndex(0, 0, 4), nullptr}};

  extractor.updateGvdGraph(gvd_layer, updated, 0);
  extractor.assignCompressedNodeAttributes();

  {  // scope after a normal update: remapping should exist as expected
    const Remapping expected_remapping{{0, 0}, {1, 1}, {2, 1}, {3, 0}, {4, 1}};
    EXPECT_EQ(extractor.compressed().remapping, expected_remapping);
    checkNode(places, "p0"_id, Eigen::Vector3d(0, 1, 0), 0.4, 5u);
  }

  // this should force the voxel attributes to flip to 0, 0, 1 for p0
  extractor.setGvdNode(0, 0, 1, 0.1, 6);
  extractor.archiveIndex(GlobalIndex(0, 0, 1));
  updated = {{GlobalIndex(0, 1, 0), nullptr}};
  extractor.updateGvdGraph(gvd_layer, updated, 0);
  extractor.assignCompressedNodeAttributes();

  EXPECT_EQ(gvd.nodes().size(), 5u);
  EXPECT_TRUE(extractor.to_archive_.empty());

  {  // scope after deleting one gvd member of p0: p0 attributes should update
    const Remapping expected_remapping{{0, 0}, {1, 1}, {2, 1}, {3, 0}, {4, 1}};
    EXPECT_EQ(extractor.compressed().remapping, expected_remapping);
    checkNode(places, "p0"_id, Eigen::Vector3d(0, 0, 1), 0.1, 6u);
  }

  extractor.archiveIndex(GlobalIndex(0, 1, 0));
  extractor.clearArchived();
  EXPECT_FALSE(extractor.to_archive_.empty());

  // this should disconnect the two sibilings
  extractor.clearIndex(GlobalIndex(0, 0, 2));
  extractor.clearArchived();
  EXPECT_TRUE(extractor.to_archive_.empty());
  EXPECT_EQ(extractor.compressed().nodes.size(), 1u);
  EXPECT_EQ(gvd.nodes().size(), 2u);
}

TEST_F(GraphExtractorFixture, testArchiveAndDelete) {
  GraphExtractor::Config config;
  config.compression_distance_m = 3.0;
  config.min_node_distance_m = 0.0;

  TestGraphExtractor extractor(config);
  const auto& gvd = extractor.gvd_graph();
  const auto& places = extractor.graph();

  EXPECT_TRUE(gvd.empty());

  extractor.setGvdNode(0, 0, 1, 0.1, 1);
  extractor.setGvdNode(0, 0, 2, 0.2, 4);
  extractor.setGvdNode(0, 0, 3, 0.3, 3);
  extractor.setGvdNode(0, 1, 0, 0.4, 5);
  extractor.setGvdNode(0, 0, 4, 0.5, 5);
  EXPECT_EQ(gvd.nodes().size(), 5u);

  IndexVoxelQueue updated{{GlobalIndex(0, 0, 1), nullptr},
                          {GlobalIndex(0, 0, 2), nullptr},
                          {GlobalIndex(0, 0, 3), nullptr},
                          {GlobalIndex(0, 1, 0), nullptr},
                          {GlobalIndex(0, 0, 4), nullptr}};

  extractor.updateGvdGraph(gvd_layer, updated, 0);
  extractor.assignCompressedNodeAttributes();

  {  // scope after a normal update: remapping should exist as expected
    const Remapping expected_remapping{{0, 0}, {1, 1}, {2, 1}, {3, 0}, {4, 1}};
    EXPECT_EQ(extractor.compressed().remapping, expected_remapping);
    checkNode(places, "p0"_id, Eigen::Vector3d(0, 1, 0), 0.4, 5u);
  }

  extractor.archiveIndex(GlobalIndex(0, 0, 1));
  extractor.clearIndex(GlobalIndex(0, 1, 0));
  extractor.clearArchived();
  extractor.assignCompressedNodeAttributes();

  EXPECT_EQ(gvd.nodes().size(), 4u);
  std::unordered_set<uint64_t> expected_archive{0};
  EXPECT_EQ(extractor.to_archive_, expected_archive);

  {  // scope after deleting one gvd member of p0: p0 attributes should update
    const Remapping expected_remapping{{0, 0}, {1, 1}, {2, 1}, {4, 1}};
    EXPECT_EQ(extractor.compressed().remapping, expected_remapping);
    checkNode(places, "p0"_id, Eigen::Vector3d(0, 0, 1), 0.1, 1u);
  }
}

TEST_F(GraphExtractorFixture, testUniformGvd) {
  GraphExtractor::Config config;
  config.compression_distance_m = 0.5;
  GraphExtractor extractor(config);

  const auto& gvd = extractor.gvd_graph();
  EXPECT_TRUE(gvd.empty());

  extractor.pushIndex(GlobalIndex(0, 0, 0));
  extractor.pushIndex(GlobalIndex(1, 0, 0));
  extractor.pushIndex(GlobalIndex(0, 1, 0));
  extractor.pushIndex(GlobalIndex(0, 0, 1));
  extractor.pushIndex(GlobalIndex(2, 0, 0));
  extractor.pushIndex(GlobalIndex(0, 2, 0));
  extractor.pushIndex(GlobalIndex(0, 0, 2));
  extractor.pushIndex(GlobalIndex(3, 0, 0));
  extractor.pushIndex(GlobalIndex(0, 3, 0));
  extractor.pushIndex(GlobalIndex(0, 0, 3));
  // make sure we get coverage for invalid voxels
  extractor.pushIndex(GlobalIndex(0, 0, 23));
  extractor.extract(gvd_layer, 0);

  EXPECT_EQ(gvd.nodes().size(), 10u);
}

}  // namespace hydra::places
