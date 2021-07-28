//#include "kimera_topology_test/test_fixtures.h"

#include <gtest/gtest.h>

#include <kimera_topology/graph_extractor.h>

namespace kimera {
namespace topology {

class TestGraphExtractor : public GraphExtractor {
 public:
  explicit TestGraphExtractor(const GraphExtractorConfig& config)
      : GraphExtractor(config) {}

  ~TestGraphExtractor() = default;

  using GraphExtractor::addEdgeToGraph;
  using GraphExtractor::addNeighborToFrontier;
  using GraphExtractor::addPlaceToGraph;

  using GraphExtractor::connected_edges_;
  using GraphExtractor::edge_deviation_queue_;
  using GraphExtractor::edge_info_map_;
  using GraphExtractor::id_index_map_;
  using GraphExtractor::id_root_index_map_;
  using GraphExtractor::index_info_map_;
  using GraphExtractor::next_edge_id_;
  using GraphExtractor::node_edge_id_map_;
};

struct VoxelIndexPair {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GvdVoxel voxel;
  GlobalIndex index;
};

class GraphExtractorTestFixture : public ::testing::Test {
 public:
  GraphExtractorTestFixture() = default;
  virtual ~GraphExtractorTestFixture() = default;

  virtual void SetUp() override {
    layer.reset(new Layer<GvdVoxel>(voxel_size, voxels_per_side));
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

    VoxelIndex voxel_index;
    BlockIndex block_index;
    voxblox::getBlockAndVoxelIndexFromGlobalVoxelIndex(
        to_return.index, layer->voxels_per_side(), &block_index, &voxel_index);
    Block<GvdVoxel>::Ptr block = layer->allocateBlockPtrByIndex(block_index);
    block->getVoxelByVoxelIndex(voxel_index) = to_return.voxel;

    return to_return;
  }

  void makeLine(GraphExtractor& extractor, size_t r1, size_t c1, size_t r2, size_t c2) {
    GlobalIndex start(r1, c1, 0);
    GlobalIndex end(r2, c2, 0);
    voxblox::AlignedVector<GlobalIndex> points = makeBresenhamLine(start, end);
    for (const auto& point : points) {
      extractor.pushGvdIndex(
          makeVoxelAndIndex(0.0, 2, point(0), point(1), point(2)).index);
    }
  }

  int voxels_per_side = 16;
  float voxel_size = 0.1;
  std::unique_ptr<Layer<GvdVoxel>> layer;
  GraphExtractorConfig config;
};

TEST_F(GraphExtractorTestFixture, AddAndRemovePlace) {
  TestGraphExtractor extractor(config);

  VoxelIndexPair test_info = makeVoxelAndIndex(5.0, 6, 1, 2, 3);
  extractor.addPlaceToGraph(*layer, test_info.voxel, test_info.index);

  const SceneGraphLayer& graph = extractor.getGraph();
  EXPECT_EQ(1u, graph.nodes().size());
  EXPECT_TRUE(graph.hasNode(NodeSymbol('p', 0)));

  Eigen::Vector3d expected_pos(0.15, 0.25, 0.35);
  Eigen::Vector3d result_pos = graph.getPosition(NodeSymbol('p', 0));
  EXPECT_NEAR(0.0, (expected_pos - result_pos).norm(), 1.0e-8);

  EXPECT_EQ(1u, extractor.index_info_map_.size());
  EXPECT_EQ(1u, extractor.node_edge_id_map_.size());
  EXPECT_EQ(1u, extractor.id_index_map_.size());

  extractor.clearGvdIndex(test_info.index);
  EXPECT_EQ(0u, graph.nodes().size());
  EXPECT_FALSE(graph.hasNode(NodeSymbol('p', 0)));
  EXPECT_EQ(0u, extractor.index_info_map_.size());
  EXPECT_EQ(0u, extractor.node_edge_id_map_.size());
  EXPECT_EQ(0u, extractor.id_index_map_.size());
}

TEST_F(GraphExtractorTestFixture, ExpandFrontier) {
  TestGraphExtractor extractor(config);

  VoxelIndexPair test_info = makeVoxelAndIndex(5.0, 6, 1, 2, 3);
  extractor.addPlaceToGraph(*layer, test_info.voxel, test_info.index);

  VoxelIndexPair neighbor = makeVoxelAndIndex(5.0, 6, 1, 2, 4);

  VoxelGraphInfo curr_info(NodeSymbol('p', 0), false);
  extractor.addNeighborToFrontier(curr_info, neighbor.index);
  EXPECT_EQ(2u, extractor.index_info_map_.size());
  ASSERT_EQ(1u, extractor.index_info_map_.count(neighbor.index));
  EXPECT_FALSE(extractor.index_info_map_.at(neighbor.index).is_node);

  EXPECT_EQ(1u, extractor.id_index_map_.size());
  ASSERT_EQ(1u, extractor.id_index_map_.count(NodeSymbol('p', 0)));
  EXPECT_EQ(1u, extractor.id_index_map_.at(NodeSymbol('p', 0)).size());

  EXPECT_EQ(1u, extractor.next_edge_id_);
  ASSERT_EQ(1u, extractor.node_edge_id_map_.count(NodeSymbol('p', 0)));
  EXPECT_EQ(1u, extractor.node_edge_id_map_.at(NodeSymbol('p', 0)).size());
  EXPECT_EQ(1u, extractor.edge_info_map_.size());
  EXPECT_EQ(1u, extractor.edge_info_map_.count(0));
}

TEST_F(GraphExtractorTestFixture, AddEdgeMapsCorrect) {
  TestGraphExtractor extractor(config);

  VoxelIndexPair first_node = makeVoxelAndIndex(0.2, 3, 1, 2, 3);
  extractor.addPlaceToGraph(*layer, first_node.voxel, first_node.index);

  VoxelIndexPair second_node = makeVoxelAndIndex(0.2, 3, 1, 2, 10);
  extractor.addPlaceToGraph(*layer, second_node.voxel, second_node.index);

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
  extractor.addEdgeToGraph(curr_info, second_info);

  EXPECT_EQ(1u, extractor.connected_edges_.size());
  EXPECT_EQ(0u, extractor.edge_deviation_queue_.size());
}

TEST_F(GraphExtractorTestFixture, SimpleExtractionCorrect) {
  config.max_edge_split_iterations = 0;
  TestGraphExtractor extractor(config);

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

  extractor.extract(*layer);

  const SceneGraphLayer& graph = extractor.getGraph();
  EXPECT_EQ(3u, graph.nodes().size());
  for (size_t i = 0; i < graph.nodes().size(); ++i) {
    NodeId node = NodeSymbol('p', i);
    ASSERT_TRUE(graph.hasNode(node));
    ASSERT_TRUE(extractor.id_root_index_map_.count(node));
  }

  // row major ordering
  EXPECT_EQ(GlobalIndex(3, 25, 0), extractor.id_root_index_map_.at(NodeSymbol('p', 0)));
  EXPECT_EQ(GlobalIndex(6, 4, 0), extractor.id_root_index_map_.at(NodeSymbol('p', 1)));
  EXPECT_EQ(GlobalIndex(9, 25, 0), extractor.id_root_index_map_.at(NodeSymbol('p', 2)));

  EXPECT_EQ(2u, graph.edges().size());
}

TEST_F(GraphExtractorTestFixture, SimpleExtractionWithSplitting) {
  TestGraphExtractor extractor(config);

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

  extractor.extract(*layer);

  const SceneGraphLayer& graph = extractor.getGraph();
  // we expect two new nodes: one for each original edge
  EXPECT_EQ(5u, graph.nodes().size());
  for (size_t i = 0; i < graph.nodes().size(); ++i) {
    NodeId node = NodeSymbol('p', i);
    ASSERT_TRUE(graph.hasNode(node));
    ASSERT_TRUE(extractor.id_root_index_map_.count(node));
  }

  EXPECT_EQ(4u, graph.edges().size());
}

}  // namespace topology
}  // namespace kimera
