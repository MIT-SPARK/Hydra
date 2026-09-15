#include <gtest/gtest.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>

#include "hydra/utils/graph_utilities.h"

namespace hydra {
TEST(GraphUtilities, RemapsOverlappingIdsAndEdges) {
  auto graph = std::make_shared<spark_dsg::SceneGraph>();
  const spark_dsg::NodeSymbol room('R', 12);
  const spark_dsg::NodeSymbol place('p', 4);
  graph->emplaceNode(spark_dsg::DsgLayers::ROOMS,
                     room,
                     std::make_unique<spark_dsg::RoomNodeAttributes>());
  graph->emplaceNode(spark_dsg::DsgLayers::PLACES,
                     place,
                     std::make_unique<spark_dsg::PlaceNodeAttributes>());
  graph->insertEdge(room, place);
  const auto merged = mergeGraphs({graph, graph});
  EXPECT_EQ(merged->numUnpartitionedNodes(), 4u);
  EXPECT_EQ(merged->numUnpartitionedEdges(), 2u);
  EXPECT_TRUE(
      merged->hasEdge(spark_dsg::NodeSymbol('R', 0), spark_dsg::NodeSymbol('p', 0)));
  EXPECT_TRUE(
      merged->hasEdge(spark_dsg::NodeSymbol('R', 1), spark_dsg::NodeSymbol('p', 1)));
  EXPECT_TRUE(graph->hasNode(room));
  EXPECT_TRUE(graph->hasNode(place));
}
}  // namespace hydra
