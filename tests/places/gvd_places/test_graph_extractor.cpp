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
  explicit TestGraphExtractor(const Config& config, float voxel_size)
      : GraphExtractor(config, voxel_size) {}
  ~TestGraphExtractor() = default;

  using GraphExtractor::clearArchived;
  using GraphExtractor::updateGvdGraph;

  void setGvdNode(uint64_t x, uint64_t y, uint64_t z, double distance, uint8_t basis) {
    const GlobalIndex index(x, y, z);
    gvd_.add(index, distance, basis);
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

/*
TEST_F(GraphExtractorFixture, DISABLED_testVoxelDeletion) {
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

TEST_F(GraphExtractorFixture, DISABLED_testVoxelArchival) {
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

*/

}  // namespace hydra::places
