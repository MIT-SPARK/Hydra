#include <gtest/gtest.h>
#include <hydra/backend/mst_factors.h>
#include <hydra/common/global_info.h>
#include <kimera_pgmo/deformation_graph.h>
#include <kimera_pgmo/deformation_graph_4dof.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>
#include <spark_dsg/scene_graph.h>

#include "hydra_test/shared_dsg_fixture.h"

namespace hydra {
namespace {

TEST(MstPlaceFactors, BuildsIndependentExtrasWithMeshConnections) {
  auto dsg = test::makeSharedDsg();
  const auto prefix = GlobalInfo::instance().getRobotPrefix().vertex_key;
  kimera_pgmo::DeformationGraph deformation_graph;
  gtsam::Values mesh_nodes;
  const auto vertex = gtsam::Symbol(prefix, 0);
  mesh_nodes.insert(vertex, gtsam::Pose3());
  std::vector<size_t> indices;
  std::vector<kimera_pgmo::Timestamp> stamps;
  deformation_graph.processNewMeshEdgesAndNodes(
      {}, mesh_nodes, {{vertex, 0}}, &indices, &stamps);

  for (size_t i = 0; i < 2; ++i) {
    auto attrs = std::make_unique<spark_dsg::PlaceNodeAttributes>();
    attrs->position = Eigen::Vector3d(i + 1, 0, 0);
    attrs->deformation_connections = {0, 9, std::numeric_limits<size_t>::max()};
    dsg->graph->emplaceNode(
        spark_dsg::DsgLayers::PLACES, spark_dsg::NodeSymbol('p', i), std::move(attrs));
  }

  dsg->graph->insertEdge(spark_dsg::NodeSymbol('p', 0), spark_dsg::NodeSymbol('p', 1));
  const auto base = deformation_graph.getOptimizationProblem();
  MstPlaceFactors hook(MstPlaceFactors::Config{});
  kimera_pgmo::OptimizationProblem extras;
  hook.updateProblem(0, *dsg->graph, deformation_graph, extras, nullptr);
  ASSERT_EQ(extras.initial.size(), 2u);
  ASSERT_EQ(extras.factors.size(), 5u);
  EXPECT_TRUE(extras.inliers.empty());
  auto combined = base.initial;
  combined.insert(extras.initial);
  EXPECT_NEAR(extras.factors.error(combined), 0.0, 1e-10);
  EXPECT_EQ(deformation_graph.getValues()->size(), 1u);
  EXPECT_TRUE(deformation_graph.getFactors()->empty());

  // Callers can prepare the same constraints for a 4DoF problem.
  auto [factors_4dof, initial_4dof] =
      kimera_pgmo::convertGraphToPose4DoF(extras.factors, extras.initial);
  ASSERT_EQ(factors_4dof.size(), 5u);
  initial_4dof.insert(base.initial);
  EXPECT_NEAR(factors_4dof.error(initial_4dof), 0.0, 1e-10);

  auto empty_dsg = test::makeSharedDsg();
  kimera_pgmo::OptimizationProblem next_extras;
  hook.updateProblem(1, *empty_dsg->graph, deformation_graph, next_extras, nullptr);
  EXPECT_TRUE(next_extras.initial.empty());
  EXPECT_TRUE(next_extras.factors.empty());
}

}  // namespace
}  // namespace hydra
