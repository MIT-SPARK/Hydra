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
#include <hydra/places/floodfill_graph_extractor.h>
#include <hydra/places/graph_extractor_utilities.h>

namespace hydra::places {

using NodeSet = std::unordered_set<NodeId>;

namespace {

class TestGraphExtractor : public FloodfillGraphExtractor {
 public:
  explicit TestGraphExtractor(const FloodfillExtractorConfig& config)
      : FloodfillGraphExtractor(config) {}

  ~TestGraphExtractor() = default;

  using FloodfillGraphExtractor::addEdgeToGraph;
  using FloodfillGraphExtractor::addNeighborToFrontier;
  using FloodfillGraphExtractor::addNewPlaceNode;

  using FloodfillGraphExtractor::connected_edges_;
  using FloodfillGraphExtractor::edge_info_map_;
  using FloodfillGraphExtractor::edge_split_queue_;
  using FloodfillGraphExtractor::index_graph_info_map_;
  using FloodfillGraphExtractor::next_edge_id_;
  using FloodfillGraphExtractor::node_child_map_;
  using FloodfillGraphExtractor::node_edge_id_map_;
  using GraphExtractorInterface::node_index_map_;
};

}  // namespace

struct VoxelIndexPair {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GvdVoxel voxel;
  GlobalIndex index;
};

class FloodfillGraphExtractorTestFixture : public ::testing::Test {
 public:
  FloodfillGraphExtractorTestFixture() = default;
  virtual ~FloodfillGraphExtractorTestFixture() = default;

  virtual void SetUp() override {
    layer.reset(new GvdLayer(voxel_size, voxels_per_side));
    // disable "advanced options" to start
    config.max_edge_split_iterations = 0;
    config.merge_new_nodes = false;
    config.edge_splitting_merge_nodes = false;
    config.add_freespace_edges = false;
    config.add_overlap_edges = false;
  }

  VoxelIndexPair makeVoxelAndIndex(double distance,
                                   uint8_t num_extra_basis,
                                   uint64_t x_idx,
                                   uint64_t y_idx,
                                   uint64_t z_idx) {
    VoxelIndexPair to_return;
    to_return.voxel.distance = distance;
    to_return.voxel.num_extra_basis = num_extra_basis;
    to_return.index << x_idx, y_idx, z_idx;
    layer->allocateVoxel(to_return.index) = to_return.voxel;
    return to_return;
  }

  void makeLine(
      FloodfillGraphExtractor& extractor, size_t r1, size_t c1, size_t r2, size_t c2) {
    GlobalIndex start(r1, c1, 0);
    GlobalIndex end(r2, c2, 0);
    GlobalIndices points = makeBresenhamLine(start, end);
    for (const auto& point : points) {
      extractor.pushGvdIndex(
          makeVoxelAndIndex(0.0, 2, point(0), point(1), point(2)).index);
    }
  }

  virtual void setupTestEnvironment(FloodfillGraphExtractor& extractor) {
    for (size_t r = 0; r < 20; ++r) {
      for (size_t c = 0; c < 50; ++c) {
        // set up a 2d grid of non-GVD points
        extractor.pushGvdIndex(makeVoxelAndIndex(0.0, 0, r, c, 0).index);
      }
    }

    extractor.pushGvdIndex(makeVoxelAndIndex(0.0, 5, 6, 4, 0).index);
    extractor.pushGvdIndex(makeVoxelAndIndex(0.0, 5, 3, 25, 0).index);
    extractor.pushGvdIndex(makeVoxelAndIndex(0.0, 5, 9, 25, 0).index);

    // make a y-shape
    makeLine(extractor, 6, 4, 6, 22);
    makeLine(extractor, 6, 22, 3, 25);
    makeLine(extractor, 6, 22, 9, 25);
    // lines are not start / end inclusive
    extractor.pushGvdIndex(makeVoxelAndIndex(0.0, 2, 6, 22, 0).index);

    extractor.extract(*layer, 0);
  }

  int voxels_per_side = 16;
  float voxel_size = 0.1;
  std::unique_ptr<GvdLayer> layer;
  FloodfillExtractorConfig config;
};

TEST_F(FloodfillGraphExtractorTestFixture, AddAndRemovePlace) {
  TestGraphExtractor extractor(config);

  VoxelIndexPair test_info = makeVoxelAndIndex(5.0, 6, 1, 2, 3);
  extractor.addNewPlaceNode(*layer, test_info.voxel, test_info.index);

  const SceneGraphLayer& graph = extractor.getGraph();
  EXPECT_EQ(1u, graph.nodes().size());
  EXPECT_TRUE(graph.hasNode(NodeSymbol('p', 0)));

  Eigen::Vector3d expected_pos(0.15, 0.25, 0.35);
  Eigen::Vector3d result_pos = graph.getPosition(NodeSymbol('p', 0));
  EXPECT_NEAR(0.0, (expected_pos - result_pos).norm(), 1.0e-8);

  EXPECT_EQ(1u, extractor.index_graph_info_map_.size());
  EXPECT_EQ(1u, extractor.node_edge_id_map_.size());
  EXPECT_EQ(1u, extractor.node_child_map_.size());

  extractor.clearGvdIndex(test_info.index);
  EXPECT_EQ(0u, graph.nodes().size());
  EXPECT_FALSE(graph.hasNode(NodeSymbol('p', 0)));
  EXPECT_EQ(0u, extractor.index_graph_info_map_.size());
  EXPECT_EQ(0u, extractor.node_edge_id_map_.size());
  EXPECT_EQ(0u, extractor.node_child_map_.size());
}

TEST_F(FloodfillGraphExtractorTestFixture, ExpandFrontier) {
  TestGraphExtractor extractor(config);

  VoxelIndexPair test_info = makeVoxelAndIndex(5.0, 6, 1, 2, 3);
  extractor.addNewPlaceNode(*layer, test_info.voxel, test_info.index);

  VoxelIndexPair neighbor = makeVoxelAndIndex(5.0, 6, 1, 2, 4);

  VoxelGraphInfo curr_info(NodeSymbol('p', 0), false);
  extractor.addNeighborToFrontier(curr_info, neighbor.index);
  EXPECT_EQ(2u, extractor.index_graph_info_map_.size());
  ASSERT_EQ(1u, extractor.index_graph_info_map_.count(neighbor.index));
  EXPECT_FALSE(extractor.index_graph_info_map_.at(neighbor.index).is_node);

  EXPECT_EQ(1u, extractor.node_child_map_.size());
  ASSERT_EQ(1u, extractor.node_child_map_.count(NodeSymbol('p', 0)));
  EXPECT_EQ(1u, extractor.node_child_map_.at(NodeSymbol('p', 0)).size());

  EXPECT_EQ(1u, extractor.next_edge_id_);
  ASSERT_EQ(1u, extractor.node_edge_id_map_.count(NodeSymbol('p', 0)));
  EXPECT_EQ(1u, extractor.node_edge_id_map_.at(NodeSymbol('p', 0)).size());
  EXPECT_EQ(1u, extractor.edge_info_map_.size());
  EXPECT_EQ(1u, extractor.edge_info_map_.count(0));
}

TEST_F(FloodfillGraphExtractorTestFixture, AddEdgeMapsCorrect) {
  TestGraphExtractor extractor(config);

  VoxelIndexPair first_node = makeVoxelAndIndex(0.2, 3, 1, 2, 3);
  extractor.addNewPlaceNode(*layer, first_node.voxel, first_node.index);

  VoxelIndexPair second_node = makeVoxelAndIndex(0.2, 3, 1, 2, 10);
  extractor.addNewPlaceNode(*layer, second_node.voxel, second_node.index);

  VoxelGraphInfo curr_info(NodeSymbol('p', 0), false);
  VoxelIndexPair good_neighbor = makeVoxelAndIndex(0.0, 0, 1, 2, 8);
  extractor.addNeighborToFrontier(curr_info, good_neighbor.index);

  curr_info.is_node = false;
  VoxelIndexPair bad_neighbor = makeVoxelAndIndex(0.0, 0, 1, 8, 5);
  extractor.addNeighborToFrontier(curr_info, bad_neighbor.index);

  VoxelGraphInfo second_info(NodeSymbol('p', 1), false);
  second_info.edge_id = 1;
  VoxelIndexPair second_neighbor = makeVoxelAndIndex(0.0, 0, 1, 2, 9);
  extractor.addNeighborToFrontier(second_info, second_neighbor.index);

  second_info.is_node = false;
  extractor.addEdgeToGraph(*layer, curr_info, second_info);

  // TOOD(nathan) unclear how merge nodes affects this
  EXPECT_LT(0u, extractor.connected_edges_.size());
  EXPECT_EQ(0u, extractor.edge_split_queue_.size());
}

TEST_F(FloodfillGraphExtractorTestFixture, SimpleExtractionCorrect) {
  TestGraphExtractor extractor(config);
  setupTestEnvironment(extractor);

  const SceneGraphLayer& graph = extractor.getGraph();
  EXPECT_EQ(3u, graph.nodes().size());
  for (size_t i = 0; i < graph.nodes().size(); ++i) {
    NodeId node = NodeSymbol('p', i);
    ASSERT_TRUE(graph.hasNode(node));
    ASSERT_TRUE(extractor.node_index_map_.count(node));
  }

  // row major ordering
  EXPECT_EQ(GlobalIndex(3, 25, 0), extractor.node_index_map_.at(NodeSymbol('p', 0)));
  EXPECT_EQ(GlobalIndex(6, 4, 0), extractor.node_index_map_.at(NodeSymbol('p', 1)));
  EXPECT_EQ(GlobalIndex(9, 25, 0), extractor.node_index_map_.at(NodeSymbol('p', 2)));

  EXPECT_EQ(2u, graph.edges().size());
}

TEST_F(FloodfillGraphExtractorTestFixture, SimpleExtractionWithSplitting) {
  config.max_edge_split_iterations = 5;

  TestGraphExtractor extractor(config);
  setupTestEnvironment(extractor);

  const SceneGraphLayer& graph = extractor.getGraph();
  // we expect two new nodes: one for each original edge
  EXPECT_EQ(5u, graph.nodes().size());
  for (size_t i = 0; i < graph.nodes().size(); ++i) {
    NodeId node = NodeSymbol('p', i);
    ASSERT_TRUE(graph.hasNode(node));
    ASSERT_TRUE(extractor.node_index_map_.count(node));
  }

  EXPECT_EQ(4u, graph.edges().size());
}

TEST_F(FloodfillGraphExtractorTestFixture, SimpleExtractionWithNodeMerging) {
  config.max_edge_split_iterations = 5;
  config.merge_new_nodes = true;
  config.edge_splitting_merge_nodes = true;
  config.node_merge_distance_m = 0.15;

  TestGraphExtractor extractor(config);
  setupTestEnvironment(extractor);

  const SceneGraphLayer& graph = extractor.getGraph();

  // TODO(nathan) something is weird here (both split nodes should remain)
  // node 0 gets merged with the new split node (the split node has more connections)
  ASSERT_EQ(4u, graph.nodes().size());
  for (size_t i = 1; i < 4; ++i) {
    NodeId node = NodeSymbol('p', i);
    EXPECT_TRUE(graph.hasNode(node)) << "missing " << NodeSymbol('p', i).getLabel();
    EXPECT_TRUE(extractor.node_index_map_.count(node));
  }

  EXPECT_EQ(3u, graph.edges().size());
}

}  // namespace hydra::places
