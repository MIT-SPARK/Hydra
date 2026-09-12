#include <gtest/gtest.h>
#include <hydra/common/global_info.h>
#include <hydra/frontend/deformation_graph_builder.h>
#include <hydra/utils/mesh_deduplication.h>
#include <hydra/utils/pgmo_mesh_interface.h>

namespace hydra {

TEST(DeformationGraphBuilder, CompactedBlocksPreserveTriangleConnectivity) {
  GlobalInfo::init(PipelineConfig{});
  DeformationGraphBuilder::Config config;
  config.resolution = 0.1;
  DeformationGraphBuilder raw(config), indexed(config);
  SharedDsgInfo dsg(SharedDsgInfo::Config{});
  const auto make_input = [](bool compact) {
    auto map = std::make_shared<VolumetricMap>(VolumetricMap::Config{});
    auto& mesh = map->getMeshLayer().allocateBlock(BlockIndex(0, 0, 0));
    mesh.resizeVertices(6);
    mesh.points = {
        {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1.0e-6f, 0, 0}, {1, 1, 0}, {0, 1, 0}};
    mesh.faces = {{0, 1, 2}, {3, 4, 5}};
    if (compact) {
      deduplicateMesh(mesh, 1.0e-5);
    }
    ActiveWindowOutput input(std::make_shared<InputData>(nullptr));
    input.timestamp_ns = 1000000000;
    input.setMap(map);
    return input;
  };
  const auto original = make_input(false);
  const auto compact = make_input(true);
  const auto& block = compact.map().getMeshLayer().getBlock(BlockIndex(0, 0, 0));
  ASSERT_EQ(block.numVertices(), 4u);
  PgmoMeshInterface single(block);
  EXPECT_EQ(single.activeBlockSize(), 4u);
  EXPECT_EQ(single.activeBlockNumFaces(), 2u);
  EXPECT_EQ(single.getActiveFace(1), (std::array<size_t, 3>{0, 3, 2}));
  FrontendOutput a(original.timestamp_ns, 1), b(compact.timestamp_ns, 1);
  raw.call(original, dsg, a, nullptr);
  indexed.call(compact, dsg, b, nullptr);
  ASSERT_NE(a.deformation_graph, nullptr);
  ASSERT_NE(b.deformation_graph, nullptr);
  ASSERT_EQ(a.deformation_graph->nodes.size(), 4u);
  ASSERT_EQ(a.deformation_graph->nodes.size(), b.deformation_graph->nodes.size());
  ASSERT_EQ(a.deformation_graph->edges.size(), b.deformation_graph->edges.size());
  for (size_t i = 0; i < a.deformation_graph->nodes.size(); ++i) {
    const auto& lhs = a.deformation_graph->nodes[i];
    const auto& rhs = b.deformation_graph->nodes[i];
    EXPECT_EQ(lhs.key, rhs.key);
    EXPECT_EQ(lhs.pose.matrix(), rhs.pose.matrix());
  }
  for (size_t i = 0; i < a.deformation_graph->edges.size(); ++i) {
    const auto& lhs = a.deformation_graph->edges[i];
    const auto& rhs = b.deformation_graph->edges[i];
    EXPECT_EQ(lhs.key_from, rhs.key_from);
    EXPECT_EQ(lhs.key_to, rhs.key_to);
    EXPECT_EQ(lhs.pose.matrix(), rhs.pose.matrix());
  }
  GlobalInfo::reset();
}

}  // namespace hydra
