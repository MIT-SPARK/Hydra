#include <gtest/gtest.h>
#include <hydra_multi/operators/dynamic_scene_graph_operator.h>
#include <spark_dsg/serialization/graph_binary_serialization.h>

#include "hydra_multi/common/types.h"

namespace hydra_multi {

inline void fillDsg(DynamicSceneGraph& graph,
                    size_t num_nodes,
                    size_t num_layers,
                    Eigen::Vector3d offset = Eigen::Vector3d::Zero()) {
  std::vector<LayerId> layers(num_layers);
  std::iota(layers.begin(), layers.end(), 0);
  std::vector<LayerKey> layer_keys(layers.begin(), layers.end());
  graph.reset(layer_keys);
  for (size_t i = 0; i < num_nodes; i++) {
    size_t layer = i % num_layers;
    Eigen::Vector3d pos;
    if (i % 3 == 1) {
      pos << static_cast<float>(i), static_cast<float>(i), 0.0;
    } else if (i % 3 == 2) {
      pos << static_cast<float>(i), static_cast<float>(i), 1.0;
    } else {
      pos << static_cast<float>(i), 0.0, static_cast<float>(i);
    }
    pos += offset;
    graph.emplaceNode(layer, i, std::make_unique<NodeAttributes>(pos));

    if (layer % 2 == 1) {
      graph.insertEdge(i, i - 1);
    }
  }
}

TEST(DynamicSceneGraphOperatorTests, IncrementalAppend) {
  auto data = std::make_shared<DynamicSceneGraph>();
  DynamicSceneGraphOperator dsg_operator(data);

  DynamicSceneGraph dsg;
  fillDsg(dsg, 10, 3);
  std::vector<uint8_t> buffer_1;
  io::binary::writeGraph(dsg, buffer_1, false);
  dsg_operator.incrementalAppend(buffer_1);

  EXPECT_EQ(10, data->numNodes());

  // Modify the dsg
  DynamicSceneGraph dsg_mod;
  fillDsg(dsg_mod, 10, 3, {1, 2, 3});

  // Update the dsg
  EXPECT_TRUE(dsg_operator(dsg_mod, OperationType::UPDATE));

  // Then append to it, expect that append will be consistent to update
  Eigen::Vector3d pos10;
  pos10 << 10, 10, 10;
  dsg.emplaceNode(0, 10, std::make_unique<NodeAttributes>(pos10));
  std::vector<uint8_t> buffer_2;
  io::binary::writeGraph(dsg, buffer_2, false);
  EXPECT_TRUE(dsg_operator.incrementalAppend(buffer_2));

  Eigen::Vector3d pos10_mod;
  pos10_mod << 11, 12, 13;
  EXPECT_EQ(11, data->numNodes());
  EXPECT_TRUE(data->getNode(10).attributes().position.isApprox(pos10_mod));
}

TEST(DynamicSceneGraphOperatorTests, Merge) {
  auto data = std::make_shared<DynamicSceneGraph>();
  DynamicSceneGraphOperator dsg_operator(data);

  DynamicSceneGraph dsg;
  fillDsg(dsg, 10, 3);
  std::vector<uint8_t> buffer_1;
  io::binary::writeGraph(dsg, buffer_1, false);
  dsg_operator.incrementalAppend(buffer_1);

  EXPECT_EQ(10, data->numNodes());

  // Modify the dsg
  DynamicSceneGraph dsg_mod;
  fillDsg(dsg_mod, 10, 3, {1, 2, 3});

  // Update the dsg
  EXPECT_TRUE(dsg_operator(dsg_mod, OperationType::UPDATE));

  // Then merge to it, expect that the new parts consistent to update
  DynamicSceneGraph dsg_2;
  fillDsg(dsg_2, 12, 3);

  EXPECT_TRUE(dsg_operator(dsg_2, OperationType::MERGE));
  EXPECT_EQ(12, data->numNodes());
  Eigen::Vector3d pos11_mod;
  pos11_mod << 12, 13, 4;
  EXPECT_TRUE(data->getNode(11).attributes().position.isApprox(pos11_mod));
}

TEST(DynamicSceneGraphOperatorTests, Rebase) {
  auto data = std::make_shared<DynamicSceneGraph>();
  DynamicSceneGraphOperator dsg_operator(data);

  DynamicSceneGraph dsg;
  fillDsg(dsg, 12, 3);
  std::vector<uint8_t> buffer_1;
  io::binary::writeGraph(dsg, buffer_1, false);
  dsg_operator.incrementalAppend(buffer_1);

  EXPECT_EQ(12, data->numNodes());

  // Then rebase on smaller graph, the non-overlapping parts consistent to rebase
  DynamicSceneGraph dsg_2;
  fillDsg(dsg_2, 9, 3, {1, 2, 3});

  EXPECT_TRUE(dsg_operator(dsg_2, OperationType::REBASE));
  EXPECT_EQ(12, data->numNodes());
  Eigen::Vector3d pos11_mod;
  pos11_mod << 12, 13, 4;
  EXPECT_TRUE(data->getNode(11).attributes().position.isApprox(pos11_mod));

  Eigen::Vector3d pos12;
  pos12 << 12, 12, 12;
  dsg.emplaceNode(0, 12, std::make_unique<NodeAttributes>(pos12));
  std::vector<uint8_t> buffer_2;
  io::binary::writeGraph(dsg, buffer_2, false);
  EXPECT_TRUE(dsg_operator.incrementalAppend(buffer_2));

  EXPECT_EQ(13, data->numNodes());
  Eigen::Vector3d pos12_mod;
  pos12_mod << 13, 14, 15;
  EXPECT_TRUE(data->getNode(12).attributes().position.isApprox(pos12_mod));
}

TEST(DynamicSceneGraphOperatorTests, Update) {
  auto data = std::make_shared<DynamicSceneGraph>();
  DynamicSceneGraphOperator dsg_operator(data);

  DynamicSceneGraph dsg;
  fillDsg(dsg, 10, 3);
  std::vector<uint8_t> buffer_1;
  io::binary::writeGraph(dsg, buffer_1, false);
  dsg_operator.incrementalAppend(buffer_1);

  EXPECT_EQ(10, data->numNodes());

  // Modify the dsg
  DynamicSceneGraph dsg_1;
  fillDsg(dsg_1, 10, 3, {1, 2, 3});

  // Update the dsg
  EXPECT_TRUE(dsg_operator(dsg_1, OperationType::UPDATE));
  Eigen::Vector3d pos0;
  pos0 << 1, 2, 3;
  EXPECT_TRUE(data->getNode(0).attributes().position.isApprox(pos0));

  DynamicSceneGraph dsg_2;
  fillDsg(dsg_2, 5, 3);

  EXPECT_FALSE(dsg_operator(dsg_2, OperationType::UPDATE));
}
}  // namespace hydra_multi
