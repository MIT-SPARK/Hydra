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
#include <hydra/places/compression_graph_extractor.h>

namespace hydra::places {

namespace {

class TestGraphExtractor : public CompressionGraphExtractor {
 public:
  explicit TestGraphExtractor(const CompressionExtractorConfig& config)
      : CompressionGraphExtractor(config) {}

  ~TestGraphExtractor() = default;

  void assignCompressedNodeAttributes(bool clear_active = true) {
    CompressionGraphExtractor::assignCompressedNodeAttributes();
    if (clear_active) {
      updated_nodes_.clear();
    }
  }

  using CompressionGraphExtractor::clearArchived;
  using CompressionGraphExtractor::updateGvdGraph;
  using CompressionGraphExtractor::updateNode;

  using CompressionGraphExtractor::compressed_id_map_;
  using CompressionGraphExtractor::compressed_index_map_;
  using CompressionGraphExtractor::compressed_info_map_;
  using CompressionGraphExtractor::compressed_remapping_;
  using CompressionGraphExtractor::to_archive_;
  using CompressionGraphExtractor::updated_nodes_;

  void setGvdNode(
      uint64_t x, uint64_t y, uint64_t z, double distance, uint8_t num_basis) {
    const GlobalIndex index(x, y, z);
    const Eigen::Vector3d position(x, y, z);
    updateNode(index, position, distance, num_basis);
  }
};

using Remapping = std::unordered_map<uint64_t, uint64_t>;

}  // namespace

class CompressionGraphExtractorTestFixture : public ::testing::Test {
 public:
  CompressionGraphExtractorTestFixture() = default;
  virtual ~CompressionGraphExtractorTestFixture() = default;

  virtual void SetUp() override {
    gvd_layer.reset(new GvdLayer(1.0, 16));
    const auto block_index = BlockIndex::Zero();
    auto gvd_block = gvd_layer->allocateBlockPtr(block_index);
    for (size_t i = 0; i < gvd_block->numVoxels(); ++i) {
      auto& voxel = gvd_block->getVoxel(i);
      voxel.distance = 0.3;
      voxel.num_extra_basis = 4;
    }
  }

  GvdLayer::Ptr gvd_layer;
};

TEST_F(CompressionGraphExtractorTestFixture, TestUpdateNode) {
  CompressionExtractorConfig config;
  TestGraphExtractor extractor(config);
  const auto& gvd = extractor.getGvdGraph();

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
    EXPECT_NEAR((info->position - Eigen::Vector3d(0, 0, 1)).norm(), 0.0, 1.0e-9);
    EXPECT_EQ(info->distance, 0.1);
    EXPECT_EQ(info->num_basis_points, 1u);
  }

  {
    const auto info = gvd.getNode(1);
    ASSERT_TRUE(info != nullptr);
    EXPECT_NEAR((info->position - Eigen::Vector3d(0, 0, 2)).norm(), 0.0, 1.0e-9);
    EXPECT_EQ(info->distance, 0.2);
    EXPECT_EQ(info->num_basis_points, 2u);
  }

  extractor.setGvdNode(0, 0, 2, 0.3, 2);
  EXPECT_EQ(gvd.nodes().size(), 2u);
  {
    const auto info = gvd.getNode(1);
    ASSERT_TRUE(info != nullptr);
    EXPECT_NEAR((info->position - Eigen::Vector3d(0, 0, 2)).norm(), 0.0, 1.0e-9);
    EXPECT_EQ(info->distance, 0.3);
    EXPECT_EQ(info->num_basis_points, 2u);
  }
}

TEST_F(CompressionGraphExtractorTestFixture, testUpdateGraph) {
  CompressionExtractorConfig config;
  config.min_node_distance_m = 0.0;
  TestGraphExtractor extractor(config);
  const auto& gvd = extractor.getGvdGraph();

  EXPECT_TRUE(gvd.empty());

  extractor.setGvdNode(0, 0, 1, 0.1, 1);
  extractor.setGvdNode(0, 0, 2, 0.1, 1);
  extractor.setGvdNode(0, 0, 4, 0.1, 1);
  extractor.setGvdNode(0, 1, 0, 0.1, 1);
  EXPECT_EQ(gvd.nodes().size(), 4u);

  // updateGvdGraph doesn't use voxel pointers, so nullptr is safe here
  // we also add 0, 0, 5 for test coverage (to simulate pruning)
  IndexVoxelQueue updated{{GlobalIndex(0, 0, 1), nullptr},
                          {GlobalIndex(0, 0, 2), nullptr},
                          {GlobalIndex(0, 1, 0), nullptr},
                          {GlobalIndex(0, 0, 4), nullptr},
                          {GlobalIndex(0, 0, 5), nullptr}};

  extractor.updateGvdGraph(updated, 0);

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
  EXPECT_EQ(extractor.compressed_remapping_, expected_remapping);

  ASSERT_TRUE(extractor.compressed_info_map_.count(0));
  ASSERT_TRUE(extractor.compressed_info_map_.count(1));
  ASSERT_TRUE(extractor.compressed_info_map_.count(2));

  {
    const std::set<uint64_t> expected_siblings{1, 2};
    const std::set<uint64_t> expected_active_refs{0};
    const auto& info = extractor.compressed_info_map_.at(0);
    EXPECT_EQ(info.siblings, expected_siblings);
    EXPECT_EQ(info.active_refs, expected_active_refs);
    EXPECT_TRUE(info.archived_refs.empty());
  }

  {
    const std::set<uint64_t> expected_siblings{0};
    const std::set<uint64_t> expected_active_refs{1};
    const auto& info = extractor.compressed_info_map_.at(1);
    EXPECT_EQ(info.siblings, expected_siblings);
    EXPECT_EQ(info.active_refs, expected_active_refs);
    EXPECT_TRUE(info.archived_refs.empty());
  }

  {
    const std::set<uint64_t> expected_siblings{0};
    const std::set<uint64_t> expected_active_refs{3};
    const auto& info = extractor.compressed_info_map_.at(2);
    EXPECT_EQ(info.siblings, expected_siblings);
    EXPECT_EQ(info.active_refs, expected_active_refs);
    EXPECT_TRUE(info.archived_refs.empty());
  }
}

TEST_F(CompressionGraphExtractorTestFixture, testAttributeAssignmentOneToOne) {
  CompressionExtractorConfig config;
  config.min_node_distance_m = 0.0;

  TestGraphExtractor extractor(config);
  const auto& gvd = extractor.getGvdGraph();

  EXPECT_TRUE(gvd.empty());

  extractor.setGvdNode(0, 0, 1, 0.1, 1);
  extractor.setGvdNode(0, 0, 2, 0.2, 2);
  extractor.setGvdNode(0, 0, 4, 0.3, 3);
  extractor.setGvdNode(0, 1, 0, 0.4, 4);
  EXPECT_EQ(gvd.nodes().size(), 4u);

  // updateGvdGraph doesn't use voxel pointers, so nullptr is safe here
  IndexVoxelQueue updated{{GlobalIndex(0, 0, 1), nullptr},
                          {GlobalIndex(0, 0, 2), nullptr},
                          {GlobalIndex(0, 1, 0), nullptr},
                          {GlobalIndex(0, 0, 4), nullptr}};

  extractor.updateGvdGraph(updated, 0);
  extractor.assignCompressedNodeAttributes();

  const auto& places = extractor.getGraph();
  EXPECT_EQ(places.numNodes(), 3u);

  ASSERT_TRUE(places.hasNode("p0"_id));
  ASSERT_TRUE(places.hasNode("p1"_id));
  ASSERT_TRUE(places.hasNode("p2"_id));

  {
    const auto& attrs = places.getNode("p0"_id).attributes<PlaceNodeAttributes>();
    EXPECT_NEAR((attrs.position - Eigen::Vector3d(0, 0, 1)).norm(), 0.0, 1.0e-9);
    EXPECT_EQ(attrs.distance, 0.1);
    EXPECT_EQ(attrs.num_basis_points, 1u);
  }

  {
    const auto& attrs = places.getNode("p0"_id).attributes<PlaceNodeAttributes>();
    EXPECT_NEAR((attrs.position - Eigen::Vector3d(0, 0, 1)).norm(), 0.0, 1.0e-9);
    EXPECT_EQ(attrs.distance, 0.1);
    EXPECT_EQ(attrs.num_basis_points, 1u);
  }

  {
    const auto& attrs = places.getNode("p1"_id).attributes<PlaceNodeAttributes>();
    EXPECT_NEAR((attrs.position - Eigen::Vector3d(0, 0, 2)).norm(), 0.0, 1.0e-9);
    EXPECT_EQ(attrs.distance, 0.2);
    EXPECT_EQ(attrs.num_basis_points, 2u);
  }

  {
    const auto& attrs = places.getNode("p2"_id).attributes<PlaceNodeAttributes>();
    EXPECT_NEAR((attrs.position - Eigen::Vector3d(0, 1, 0)).norm(), 0.0, 1.0e-9);
    EXPECT_EQ(attrs.distance, 0.4);
    EXPECT_EQ(attrs.num_basis_points, 4u);
  }
}

TEST_F(CompressionGraphExtractorTestFixture, testAttributeAssignmentManyToOne) {
  CompressionExtractorConfig config;
  config.min_node_distance_m = 0.1;
  config.compression_distance_m = 3.0;
  TestGraphExtractor extractor(config);
  const auto& gvd = extractor.getGvdGraph();

  EXPECT_TRUE(gvd.empty());

  extractor.setGvdNode(0, 0, 1, 0.1, 1);
  extractor.setGvdNode(0, 0, 2, 0.2, 4);
  extractor.setGvdNode(0, 0, 3, 0.3, 3);
  extractor.setGvdNode(0, 1, 0, 0.4, 5);
  extractor.setGvdNode(0, 0, 4, 0.5, 5);
  EXPECT_EQ(gvd.nodes().size(), 5u);

  // updateGvdGraph doesn't use voxel pointers, so nullptr is safe here
  IndexVoxelQueue updated{{GlobalIndex(0, 0, 1), nullptr},
                          {GlobalIndex(0, 0, 2), nullptr},
                          {GlobalIndex(0, 0, 3), nullptr},
                          {GlobalIndex(0, 1, 0), nullptr},
                          {GlobalIndex(0, 0, 4), nullptr}};

  extractor.updateGvdGraph(updated, 0);
  extractor.assignCompressedNodeAttributes();

  const auto& places = extractor.getGraph();
  EXPECT_EQ(places.numNodes(), 2u);

  ASSERT_TRUE(places.hasNode("p0"_id));
  ASSERT_TRUE(places.hasNode("p1"_id));

  Remapping expected_remapping{{0, 0}, {1, 1}, {2, 1}, {3, 0}, {4, 1}};
  EXPECT_EQ(extractor.compressed_remapping_, expected_remapping);

  {
    const auto& attrs = places.getNode("p0"_id).attributes<PlaceNodeAttributes>();
    EXPECT_NEAR((attrs.position - Eigen::Vector3d(0, 1, 0)).norm(), 0.0, 1.0e-9)
        << attrs.position.transpose();
    EXPECT_EQ(attrs.distance, 0.4);
    EXPECT_EQ(attrs.num_basis_points, 5u);
  }

  {
    const auto& attrs = places.getNode("p1"_id).attributes<PlaceNodeAttributes>();
    EXPECT_NEAR((attrs.position - Eigen::Vector3d(0, 0, 4)).norm(), 0.0, 1.0e-9)
        << attrs.position.transpose();
    EXPECT_EQ(attrs.distance, 0.5);
    EXPECT_EQ(attrs.num_basis_points, 5u);
  }
}

// show that isolated voxels deletion gets propagated to compressed graph
TEST_F(CompressionGraphExtractorTestFixture, testSingleVoxelDeletion) {
  CompressionExtractorConfig config;
  config.compression_distance_m = 3.0;
  config.min_node_distance_m = 0.1;
  TestGraphExtractor extractor(config);
  const auto& gvd = extractor.getGvdGraph();
  const auto& places = extractor.getGraph();

  EXPECT_TRUE(gvd.empty());

  extractor.setGvdNode(0, 0, 2, 0.2, 2);
  extractor.setGvdNode(0, 0, 3, 0.3, 3);
  extractor.setGvdNode(0, 0, 4, 0.4, 4);
  extractor.setGvdNode(1, 0, 2, 0.5, 3);
  EXPECT_EQ(gvd.nodes().size(), 4u);

  // updateGvdGraph doesn't use voxel pointers, so nullptr is safe here
  IndexVoxelQueue updated{{GlobalIndex(0, 0, 2), nullptr},
                          {GlobalIndex(0, 0, 3), nullptr},
                          {GlobalIndex(0, 0, 4), nullptr},
                          {GlobalIndex(1, 0, 2), nullptr}};

  extractor.updateGvdGraph(updated, 0);
  extractor.assignCompressedNodeAttributes();

  {  // scope after a normal update: remapping should exist as expected
    const Remapping expected_remapping{{0, 0}, {1, 0}, {2, 0}, {3, 0}};
    EXPECT_EQ(extractor.compressed_remapping_, expected_remapping);

    const auto& attrs = places.getNode("p0"_id).attributes<PlaceNodeAttributes>();
    EXPECT_NEAR((attrs.position - Eigen::Vector3d(0, 0, 4)).norm(), 0.0, 1.0e-9)
        << attrs.position.transpose();
    EXPECT_EQ(attrs.distance, 0.4);
    EXPECT_EQ(attrs.num_basis_points, 4u);
  }

  extractor.clearGvdIndex(GlobalIndex(0, 0, 3));
  // technically we only need to update (0, 0, 4)
  extractor.updateGvdGraph(updated, 0);
  extractor.assignCompressedNodeAttributes();

  EXPECT_EQ(gvd.nodes().size(), 2u);
  EXPECT_TRUE(extractor.to_archive_.empty());

  {  // scope after two gvd nodes are cleared: p0 attributes should update
    const Remapping expected_remapping{{0, 0}, {3, 0}};
    EXPECT_EQ(extractor.compressed_remapping_, expected_remapping);

    const auto& attrs = places.getNode("p0"_id).attributes<PlaceNodeAttributes>();
    EXPECT_NEAR((attrs.position - Eigen::Vector3d(1, 0, 2)).norm(), 0.0, 1.0e-9)
        << attrs.position.transpose();
    EXPECT_EQ(attrs.distance, 0.5);
    EXPECT_EQ(attrs.num_basis_points, 3u);
  }
}

TEST_F(CompressionGraphExtractorTestFixture, testVoxelDeletion) {
  CompressionExtractorConfig config;
  config.compression_distance_m = 3.0;
  config.min_node_distance_m = 0.0;
  TestGraphExtractor extractor(config);
  const auto& gvd = extractor.getGvdGraph();
  const auto& places = extractor.getGraph();

  EXPECT_TRUE(gvd.empty());

  extractor.setGvdNode(0, 0, 1, 0.1, 1);
  extractor.setGvdNode(0, 0, 2, 0.2, 4);
  extractor.setGvdNode(0, 0, 3, 0.3, 3);
  extractor.setGvdNode(0, 1, 0, 0.4, 5);
  extractor.setGvdNode(0, 0, 4, 0.5, 5);
  EXPECT_EQ(gvd.nodes().size(), 5u);

  // updateGvdGraph doesn't use voxel pointers, so nullptr is safe here
  IndexVoxelQueue updated{{GlobalIndex(0, 0, 1), nullptr},
                          {GlobalIndex(0, 0, 2), nullptr},
                          {GlobalIndex(0, 0, 3), nullptr},
                          {GlobalIndex(0, 1, 0), nullptr},
                          {GlobalIndex(0, 0, 4), nullptr}};

  extractor.updateGvdGraph(updated, 0);
  extractor.assignCompressedNodeAttributes();

  {  // scope after a normal update: remapping should exist as expected
    const Remapping expected_remapping{{0, 0}, {1, 1}, {2, 1}, {3, 0}, {4, 1}};
    EXPECT_EQ(extractor.compressed_remapping_, expected_remapping);

    ASSERT_TRUE(places.hasNode("p0"_id));
    const auto& attrs = places.getNode("p0"_id).attributes<PlaceNodeAttributes>();
    EXPECT_NEAR((attrs.position - Eigen::Vector3d(0, 1, 0)).norm(), 0.0, 1.0e-9)
        << attrs.position.transpose();
    EXPECT_EQ(attrs.distance, 0.4);
    EXPECT_EQ(attrs.num_basis_points, 5u);
  }

  extractor.clearGvdIndex(GlobalIndex(0, 1, 0));
  extractor.assignCompressedNodeAttributes();

  EXPECT_EQ(gvd.nodes().size(), 4u);
  EXPECT_TRUE(extractor.to_archive_.empty());

  {  // scope after deleting one gvd member of p0: p0 attributes should update
    const Remapping expected_remapping{{0, 0}, {1, 1}, {2, 1}, {4, 1}};
    EXPECT_EQ(extractor.compressed_remapping_, expected_remapping);

    ASSERT_TRUE(places.hasNode("p0"_id));
    const auto& attrs = places.getNode("p0"_id).attributes<PlaceNodeAttributes>();
    EXPECT_NEAR((attrs.position - Eigen::Vector3d(0, 0, 1)).norm(), 0.0, 1.0e-9)
        << attrs.position.transpose();
    EXPECT_EQ(attrs.distance, 0.1);
    EXPECT_EQ(attrs.num_basis_points, 1u);
  }

  extractor.clearGvdIndex(GlobalIndex(0, 0, 1));
  EXPECT_EQ(gvd.nodes().size(), 3u);
  EXPECT_TRUE(extractor.to_archive_.empty());
  EXPECT_TRUE(extractor.updated_nodes_.empty());
  EXPECT_EQ(places.numNodes(), 1u);
  EXPECT_FALSE(places.hasNode("p0"_id));

  EXPECT_TRUE(extractor.compressed_info_map_.at(1).siblings.empty());
  EXPECT_EQ(extractor.compressed_info_map_.size(), 1u);
  EXPECT_EQ(extractor.compressed_index_map_.size(), 1u);
  EXPECT_EQ(extractor.compressed_id_map_.size(), 1u);
  {
    const Remapping expected_remapping{{1, 1}, {2, 1}, {4, 1}};
    EXPECT_EQ(extractor.compressed_remapping_, expected_remapping);
  }

  extractor.setGvdNode(0, 0, 1, 0.1, 1);
  updated = {{GlobalIndex(0, 0, 1), nullptr}};
  extractor.updateGvdGraph(updated, 0);
  extractor.assignCompressedNodeAttributes();
  {  // scope after re-adding one gvd member of p0
    const Remapping expected_remapping{{3, 2}, {1, 1}, {2, 1}, {4, 1}};
    EXPECT_EQ(extractor.compressed_remapping_, expected_remapping);

    ASSERT_TRUE(places.hasNode("p2"_id));
    const auto& attrs = places.getNode("p2"_id).attributes<PlaceNodeAttributes>();
    EXPECT_NEAR((attrs.position - Eigen::Vector3d(0, 0, 1)).norm(), 0.0, 1.0e-9)
        << attrs.position.transpose();
    EXPECT_EQ(attrs.distance, 0.1);
    EXPECT_EQ(attrs.num_basis_points, 1u);
  }
}

TEST_F(CompressionGraphExtractorTestFixture, testVoxelArchival) {
  CompressionExtractorConfig config;
  config.compression_distance_m = 3.0;
  config.min_node_distance_m = 0.0;
  TestGraphExtractor extractor(config);
  const auto& gvd = extractor.getGvdGraph();
  const auto& places = extractor.getGraph();

  EXPECT_TRUE(gvd.empty());

  extractor.setGvdNode(0, 0, 1, 0.1, 1);
  extractor.setGvdNode(0, 0, 2, 0.2, 4);
  extractor.setGvdNode(0, 0, 3, 0.3, 3);
  extractor.setGvdNode(0, 1, 0, 0.4, 5);
  extractor.setGvdNode(0, 0, 4, 0.5, 5);
  EXPECT_EQ(gvd.nodes().size(), 5u);

  // updateGvdGraph doesn't use voxel pointers, so nullptr is safe here
  IndexVoxelQueue updated{{GlobalIndex(0, 0, 1), nullptr},
                          {GlobalIndex(0, 0, 2), nullptr},
                          {GlobalIndex(0, 0, 3), nullptr},
                          {GlobalIndex(0, 1, 0), nullptr},
                          {GlobalIndex(0, 0, 4), nullptr}};

  extractor.updateGvdGraph(updated, 0);
  extractor.assignCompressedNodeAttributes();

  {  // scope after a normal update: remapping should exist as expected
    const Remapping expected_remapping{{0, 0}, {1, 1}, {2, 1}, {3, 0}, {4, 1}};
    EXPECT_EQ(extractor.compressed_remapping_, expected_remapping);

    const auto& attrs = places.getNode("p0"_id).attributes<PlaceNodeAttributes>();
    EXPECT_NEAR((attrs.position - Eigen::Vector3d(0, 1, 0)).norm(), 0.0, 1.0e-9)
        << attrs.position.transpose();
    EXPECT_EQ(attrs.distance, 0.4);
    EXPECT_EQ(attrs.num_basis_points, 5u);
  }

  // this should force the voxel attributes to flip to 0, 0, 1 for p0
  extractor.setGvdNode(0, 0, 1, 0.1, 6);
  extractor.removeDistantIndex(GlobalIndex(0, 0, 1));
  updated = {{GlobalIndex(0, 1, 0), nullptr}};
  extractor.updateGvdGraph(updated, 0);
  extractor.assignCompressedNodeAttributes();

  EXPECT_EQ(gvd.nodes().size(), 5u);
  EXPECT_TRUE(extractor.to_archive_.empty());

  {  // scope after deleting one gvd member of p0: p0 attributes should update
    const Remapping expected_remapping{{0, 0}, {1, 1}, {2, 1}, {3, 0}, {4, 1}};
    EXPECT_EQ(extractor.compressed_remapping_, expected_remapping);

    const auto& attrs = places.getNode("p0"_id).attributes<PlaceNodeAttributes>();
    EXPECT_NEAR((attrs.position - Eigen::Vector3d(0, 0, 1)).norm(), 0.0, 1.0e-9)
        << attrs.position.transpose();
    EXPECT_EQ(attrs.distance, 0.1);
    EXPECT_EQ(attrs.num_basis_points, 6u);
  }

  extractor.removeDistantIndex(GlobalIndex(0, 1, 0));
  extractor.clearArchived();
  EXPECT_FALSE(extractor.to_archive_.empty());

  // this should disconnect the two sibilings
  extractor.clearGvdIndex(GlobalIndex(0, 0, 2));
  extractor.clearArchived();
  EXPECT_TRUE(extractor.to_archive_.empty());
  EXPECT_EQ(extractor.compressed_info_map_.size(), 1u);
  EXPECT_EQ(gvd.nodes().size(), 2u);
}

TEST_F(CompressionGraphExtractorTestFixture, testArchiveAndDelete) {
  CompressionExtractorConfig config;
  config.compression_distance_m = 3.0;
  config.min_node_distance_m = 0.0;
  TestGraphExtractor extractor(config);
  const auto& gvd = extractor.getGvdGraph();
  const auto& places = extractor.getGraph();

  EXPECT_TRUE(gvd.empty());

  extractor.setGvdNode(0, 0, 1, 0.1, 1);
  extractor.setGvdNode(0, 0, 2, 0.2, 4);
  extractor.setGvdNode(0, 0, 3, 0.3, 3);
  extractor.setGvdNode(0, 1, 0, 0.4, 5);
  extractor.setGvdNode(0, 0, 4, 0.5, 5);
  EXPECT_EQ(gvd.nodes().size(), 5u);

  // updateGvdGraph doesn't use voxel pointers, so nullptr is safe here
  IndexVoxelQueue updated{{GlobalIndex(0, 0, 1), nullptr},
                          {GlobalIndex(0, 0, 2), nullptr},
                          {GlobalIndex(0, 0, 3), nullptr},
                          {GlobalIndex(0, 1, 0), nullptr},
                          {GlobalIndex(0, 0, 4), nullptr}};

  extractor.updateGvdGraph(updated, 0);
  extractor.assignCompressedNodeAttributes();

  {  // scope after a normal update: remapping should exist as expected
    const Remapping expected_remapping{{0, 0}, {1, 1}, {2, 1}, {3, 0}, {4, 1}};
    EXPECT_EQ(extractor.compressed_remapping_, expected_remapping);

    const auto& attrs = places.getNode("p0"_id).attributes<PlaceNodeAttributes>();
    EXPECT_NEAR((attrs.position - Eigen::Vector3d(0, 1, 0)).norm(), 0.0, 1.0e-9)
        << attrs.position.transpose();
    EXPECT_EQ(attrs.distance, 0.4);
    EXPECT_EQ(attrs.num_basis_points, 5u);
  }

  extractor.removeDistantIndex(GlobalIndex(0, 0, 1));
  extractor.clearGvdIndex(GlobalIndex(0, 1, 0));
  extractor.clearArchived();
  extractor.assignCompressedNodeAttributes();

  EXPECT_EQ(gvd.nodes().size(), 4u);
  std::unordered_set<uint64_t> expected_archive{0};
  EXPECT_EQ(extractor.to_archive_, expected_archive);

  {  // scope after deleting one gvd member of p0: p0 attributes should update
    const Remapping expected_remapping{{0, 0}, {1, 1}, {2, 1}, {4, 1}};
    EXPECT_EQ(extractor.compressed_remapping_, expected_remapping);

    const auto& attrs = places.getNode("p0"_id).attributes<PlaceNodeAttributes>();
    EXPECT_NEAR((attrs.position - Eigen::Vector3d(0, 0, 1)).norm(), 0.0, 1.0e-9)
        << attrs.position.transpose();
    EXPECT_EQ(attrs.distance, 0.1);
    EXPECT_EQ(attrs.num_basis_points, 1u);
  }
}

TEST_F(CompressionGraphExtractorTestFixture, testUniformGvd) {
  CompressionExtractorConfig config;
  config.compression_distance_m = 0.5;
  CompressionGraphExtractor extractor(config);

  const auto& gvd = extractor.getGvdGraph();
  EXPECT_TRUE(gvd.empty());

  extractor.pushGvdIndex(GlobalIndex(0, 0, 0));
  extractor.pushGvdIndex(GlobalIndex(1, 0, 0));
  extractor.pushGvdIndex(GlobalIndex(0, 1, 0));
  extractor.pushGvdIndex(GlobalIndex(0, 0, 1));
  extractor.pushGvdIndex(GlobalIndex(2, 0, 0));
  extractor.pushGvdIndex(GlobalIndex(0, 2, 0));
  extractor.pushGvdIndex(GlobalIndex(0, 0, 2));
  extractor.pushGvdIndex(GlobalIndex(3, 0, 0));
  extractor.pushGvdIndex(GlobalIndex(0, 3, 0));
  extractor.pushGvdIndex(GlobalIndex(0, 0, 3));
  // make sure we get coverage for invalid voxels
  extractor.pushGvdIndex(GlobalIndex(0, 0, 23));
  extractor.extract(*gvd_layer, 0);

  EXPECT_EQ(gvd.nodes().size(), 10u);
}

TEST(CompressionNode, testAddEdges) {
  std::unordered_map<uint64_t, CompressedNode> nodes{
      {0, CompressedNode(0)}, {1, CompressedNode(1)}, {2, CompressedNode(2)}};
  nodes.at(0).addEdgeObservation(0, 1, 1);
  nodes.at(1).addEdgeObservation(1, 0, 0);
  nodes.at(1).addEdgeObservation(1, 2, 2);
  nodes.at(2).addEdgeObservation(2, 1, 1);

  std::unordered_map<uint64_t, std::set<uint64_t>> expected_siblings{
      {0, {1}},
      {1, {0, 2}},
      {2, {1}},
  };
  EXPECT_EQ(nodes.at(0).siblings, expected_siblings.at(0));
  EXPECT_EQ(nodes.at(1).siblings, expected_siblings.at(1));
  EXPECT_EQ(nodes.at(2).siblings, expected_siblings.at(2));
}

TEST(CompressionNode, testRemoveEdges) {
  std::unordered_map<uint64_t, CompressedNode> nodes{
      {0, CompressedNode(0)}, {1, CompressedNode(1)}, {2, CompressedNode(2)}};
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

TEST(CompressionNode, testMergePolicies) {
  GvdMemberInfo info1;
  info1.num_basis_points = 1;
  info1.distance = 0.3;
  GvdMemberInfo info2;
  info2.num_basis_points = 3;
  info2.distance = 0.1;
  GvdMemberInfo info3;
  info3.num_basis_points = 3;
  info3.distance = 0.3;

  BasisPointMergePolicy basis_policy;
  EXPECT_EQ(basis_policy.compare(info1, info2), -1);
  EXPECT_EQ(basis_policy.compare(info1, info3), -1);
  EXPECT_EQ(basis_policy.compare(info2, info3), 0);
  EXPECT_EQ(basis_policy.compare(info2, info1), 1);
  EXPECT_EQ(basis_policy.compare(info3, info1), 1);
  EXPECT_EQ(basis_policy.compare(info3, info2), 0);

  DistanceMergePolicy distance_policy;
  EXPECT_EQ(distance_policy.compare(info1, info2), 1);
  EXPECT_EQ(distance_policy.compare(info1, info3), 0);
  EXPECT_EQ(distance_policy.compare(info2, info3), -1);
  EXPECT_EQ(distance_policy.compare(info2, info1), -1);
  EXPECT_EQ(distance_policy.compare(info3, info1), 0);
  EXPECT_EQ(distance_policy.compare(info3, info2), 1);
}

}  // namespace hydra::places
