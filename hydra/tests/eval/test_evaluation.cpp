#include <gtest/gtest.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>

#include "hydra/eval/place_evaluator.h"
#include "hydra/eval/room_evaluator.h"

namespace hydra::eval {
TEST(RoomMetrics, EmptyComparisons) {
  const RoomIndices rooms{{3, {{0, 0, 0}}}};
  const auto empty = scoreRooms({}, {});
  EXPECT_FALSE(empty.valid());
  EXPECT_DOUBLE_EQ(empty.total_recall, 0.0);
  EXPECT_DOUBLE_EQ(empty.total_precision, 0.0);
  const auto missing = scoreRooms(rooms, {});
  EXPECT_TRUE(missing.valid());
  EXPECT_EQ(missing.recalls, std::vector<double>{0.0});
  EXPECT_EQ(missing.overlaps.rows(), 1);
  EXPECT_EQ(missing.overlaps.cols(), 0);
  const auto extra = scoreRooms({}, rooms);
  EXPECT_EQ(extra.precisions, std::vector<double>{0.0});
}

TEST(RoomMetrics, OverlapScores) {
  const RoomIndices gt{{3, {{0, 0, 0}, {1, 0, 0}}}};
  const RoomIndices est{{8, {{1, 0, 0}, {2, 0, 0}}}};
  const auto scores = scoreRooms(gt, est);
  EXPECT_DOUBLE_EQ(scores.total_recall, 0.5);
  EXPECT_DOUBLE_EQ(scores.total_precision, 0.5);
  EXPECT_DOUBLE_EQ(scores.overlaps(0, 0), 1.0);
}

TEST(RoomEvaluator, ReusesGroundTruth) {
  auto tsdf = std::make_shared<TsdfLayer>(1.0f, 2);
  auto block = tsdf->allocateBlockPtr(BlockIndex::Zero());
  auto& voxel = block->getVoxel(0);
  voxel.weight = 1.0f;
  voxel.distance = 0.0f;
  const auto pos = block->getVoxelPosition(0);
  RoomGeometry rooms;
  rooms.addRoom(0, {spark_dsg::BoundingBox(Eigen::Vector3f::Ones(), pos)});
  RoomEvaluator evaluator({}, rooms, tsdf);
  spark_dsg::SceneGraph graph;
  const spark_dsg::NodeSymbol room('R', 0);
  const spark_dsg::NodeSymbol place('p', 0);
  auto attrs = std::make_unique<spark_dsg::PlaceNodeAttributes>();
  attrs->position = pos.cast<double>();
  attrs->distance = 0.0;
  graph.emplaceNode(spark_dsg::DsgLayers::ROOMS,
                    room,
                    std::make_unique<spark_dsg::RoomNodeAttributes>());
  graph.emplaceNode(spark_dsg::DsgLayers::PLACES, place, std::move(attrs));
  graph.insertEdge(room, place);
  EXPECT_DOUBLE_EQ(evaluator.eval(graph).total_recall, 1.0);
  EXPECT_DOUBLE_EQ(evaluator.eval(graph).total_precision, 1.0);
  EXPECT_EQ(evaluator.getRoomIndices().at(0).size(), 1u);
  graph.removeNode(place);
  EXPECT_DOUBLE_EQ(evaluator.eval(graph).total_recall, 0.0);
  EXPECT_EQ(evaluator.getRoomIndices().at(0).size(), 1u);
}

TEST(PlaceMetrics, EmptyReferenceAndLayerSelection) {
  spark_dsg::SceneGraph graph;
  places::GvdLayer gvd(1.0f, 2);
  const spark_dsg::NodeSymbol place('p', 0);
  auto attrs = std::make_unique<spark_dsg::PlaceNodeAttributes>();
  attrs->position = Eigen::Vector3d::Constant(0.5);
  attrs->distance = 1.0;
  graph.addLayer(10, {}, "test_places");
  ASSERT_TRUE(graph.emplaceNode("test_places", place, std::move(attrs)));
  EXPECT_FALSE(scorePlaces(graph, gvd, 1, "test_places").is_valid);
  auto block = gvd.allocateBlockPtr(BlockIndex::Zero());
  auto& voxel = block->getVoxel(0);
  voxel.observed = true;
  voxel.num_extra_basis = 2;
  voxel.distance = 1.25;
  const auto scores = scorePlaces(graph, gvd, 1, "test_places");
  EXPECT_TRUE(scores.is_valid);
  EXPECT_EQ(scores.num_valid, 1u);
  ASSERT_EQ(scores.gvd_distance_errors.size(), 1u);
  EXPECT_DOUBLE_EQ(scores.gvd_distance_errors.front(), 0.25);
  EXPECT_DOUBLE_EQ(scores.node_gvd_distances.front(), 0.0);
  EXPECT_FALSE(scorePlaces(graph, gvd, 3, "test_places").is_valid);
  EXPECT_FALSE(scorePlaces(graph, gvd, 1, "missing").is_valid);
}
}  // namespace hydra::eval
