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
#include <hydra/loop_closure/registration.h>

namespace hydra::lcd {

double getPoseDistance(const gtsam::Pose3& expected,
                       const RegistrationSolution& solution) {
  const gtsam::Pose3 result(gtsam::Rot3(solution.to_R_from), solution.to_p_from);
  return gtsam::Pose3::Logmap(expected.between(result)).norm();
}

struct LayerRegistrationTests : public ::testing::Test {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LayerRegistrationTests() : testing::Test() {}

  virtual ~LayerRegistrationTests() = default;

  virtual void SetUp() override {
    dest_R_src << 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    dest_t_src << 1.0, 2.0, 3.0;

    src_points = 5.0 * Eigen::MatrixXd::Random(3, 40);
    dest_points = dest_R_src * src_points;
    dest_points.colwise() += dest_t_src;

    src_layer.reset(new IsolatedSceneGraphLayer(1));
    dest_layer.reset(new IsolatedSceneGraphLayer(1));

    for (int i = 0; i < src_points.cols(); ++i) {
      auto src_attrs = std::make_unique<SemanticNodeAttributes>();
      src_attrs->position = src_points.col(i);
      src_layer->emplaceNode(i, std::move(src_attrs));

      auto dest_attrs = std::make_unique<SemanticNodeAttributes>();
      dest_attrs->position = dest_points.col(i);
      dest_layer->emplaceNode(i, std::move(dest_attrs));

      node_ids.push_back(i);
    }
  }

  Eigen::MatrixXd src_points;
  Eigen::MatrixXd dest_points;

  Eigen::Matrix3d dest_R_src;
  Eigen::Vector3d dest_t_src;

  std::list<NodeId> node_ids;
  std::unique_ptr<IsolatedSceneGraphLayer> src_layer;
  std::unique_ptr<IsolatedSceneGraphLayer> dest_layer;

  LayerRegistrationConfig reg_config;
};

struct GraphRegistrationTests : public ::testing::Test {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GraphRegistrationTests() : testing::Test() {}

  ~GraphRegistrationTests() = default;

  virtual void SetUp() override {
    std::map<LayerId, char> layer_map = {
        {DsgLayers::PLACES, 'p'}, {DsgLayers::OBJECTS, 'O'}, {DsgLayers::ROOMS, 'R'}};

    dsg.reset(new DynamicSceneGraph());

    using namespace std::chrono_literals;

    const double angle = M_PI / 6;
    dest_R_src << std::cos(angle), -std::sin(angle), 0.0, std::sin(angle),
        std::cos(angle), 0.0, 0.0, 0.0, 1.0;
    dest_t_src << 0.1, 0.2, 0.3;

    src_points = 5.0 * Eigen::MatrixXd::Random(3, 30);

    CHECK(dsg->hasLayer(DsgLayers::OBJECTS));

    for (int i = 0; i < src_points.cols(); ++i) {
      SemanticNodeAttributes attrs;
      attrs.position = src_points.col(i);
      attrs.semantic_label = i;
      CHECK(dsg->emplaceNode(DsgLayers::PLACES,
                             NodeSymbol('p', i),
                             std::make_unique<SemanticNodeAttributes>(attrs)));
      CHECK(dsg->emplaceNode(DsgLayers::OBJECTS,
                             NodeSymbol('O', i),
                             std::make_unique<SemanticNodeAttributes>(attrs)));
      dsg->insertEdge(NodeSymbol('p', i), NodeSymbol('O', i));

      attrs.position = dest_R_src * src_points.col(i) + dest_t_src;
      CHECK(dsg->emplaceNode(DsgLayers::PLACES,
                             NodeSymbol('p', i + src_points.cols()),
                             std::make_unique<SemanticNodeAttributes>(attrs)));
      CHECK(dsg->emplaceNode(DsgLayers::OBJECTS,
                             NodeSymbol('O', i + src_points.cols()),
                             std::make_unique<SemanticNodeAttributes>(attrs)));
      dsg->insertEdge(NodeSymbol('p', i + src_points.cols()),
                      NodeSymbol('O', i + src_points.cols()));
    }

    CHECK(dsg->hasNode(NodeSymbol('O', 40)));

    Eigen::Quaterniond world_q_body1(std::cos(M_PI / 8), std::sin(M_PI / 8), 0.0, 0.0);
    Eigen::Vector3d world_t_body1(-1.0, 0.2, 0.5);
    dsg->emplaceNode(
        DsgLayers::AGENTS,
        'a',
        10ns,
        std::make_unique<AgentNodeAttributes>(world_q_body1, world_t_body1, NodeId(0)));

    Eigen::Quaterniond world_q_body2(std::cos(M_PI / 8), 0.0, std::sin(M_PI / 8), 0.0);
    Eigen::Vector3d world_t_body2(5.0, -0.3, 2.1);

    Eigen::Quaterniond dest_q_body2 = Eigen::Quaterniond(dest_R_src) * world_q_body2;
    Eigen::Vector3d dest_t_body2 = dest_R_src * world_t_body2 + dest_t_src;
    dsg->emplaceNode(
        DsgLayers::AGENTS,
        'a',
        20ns,
        std::make_unique<AgentNodeAttributes>(dest_q_body2, dest_t_body2, NodeId(0)));

    dsg->insertEdge(NodeSymbol('p', 0), NodeSymbol('a', 0));
    dsg->insertEdge(NodeSymbol('p', src_points.cols()), NodeSymbol('a', 1));

    gtsam::Pose3 world_T_to(gtsam::Rot3(world_q_body1), world_t_body1);
    gtsam::Pose3 world_T_from(gtsam::Rot3(world_q_body2), world_t_body2);
    to_T_from = world_T_to.between(world_T_from);
  }

  Eigen::MatrixXd src_points;

  Eigen::Matrix3d dest_R_src;
  Eigen::Vector3d dest_t_src;

  gtsam::Pose3 to_T_from;

  DynamicSceneGraph::Ptr dsg;
  LayerRegistrationConfig reg_config;
};

TEST_F(LayerRegistrationTests, TestCorrectCorrespondenceRegistration) {
  teaser::RobustRegistrationSolver::Params params;
  params.estimate_scaling = false;
  teaser::RobustRegistrationSolver solver(params);

  LayerRegistrationProblem problem;
  problem.src_nodes = node_ids;
  problem.dest_nodes = node_ids;
  problem.dest_layer = dest_layer.get();

  auto solution =
      registerDsgLayer(reg_config,
                       solver,
                       problem,
                       *src_layer,
                       [](const SceneGraphNode& src, const SceneGraphNode& dest) {
                         return src.id == dest.id;
                       });

  ASSERT_TRUE(solution.valid);
  EXPECT_NEAR(dest_t_src.x(), solution.dest_T_src.translation().x(), 1.0e-4);
  EXPECT_NEAR(dest_t_src.y(), solution.dest_T_src.translation().y(), 1.0e-4);
  EXPECT_NEAR(dest_t_src.z(), solution.dest_T_src.translation().z(), 1.0e-4);
  gtsam::Rot3 src_R_dest_gt(dest_R_src.transpose());
  double rot_error =
      gtsam::Rot3::Logmap(src_R_dest_gt * solution.dest_T_src.rotation()).norm();
  EXPECT_NEAR(0.0, rot_error, 1.0e-4);

  EXPECT_EQ(solution.inliers.size(), node_ids.size());
  for (const auto& correspondence : solution.inliers) {
    EXPECT_EQ(correspondence.first, correspondence.second);
  }
}

TEST_F(LayerRegistrationTests, TestPairwiseRegistration) {
  teaser::RobustRegistrationSolver::Params params;
  params.estimate_scaling = false;
  teaser::RobustRegistrationSolver solver(params);

  LayerRegistrationProblem problem;
  problem.src_nodes = node_ids;
  problem.dest_nodes = node_ids;
  problem.dest_layer = dest_layer.get();

  auto solution = registerDsgLayerPairwise(reg_config, solver, problem, *src_layer);

  ASSERT_TRUE(solution.valid);
  EXPECT_NEAR(dest_t_src.x(), solution.dest_T_src.translation().x(), 1.0e-4);
  EXPECT_NEAR(dest_t_src.y(), solution.dest_T_src.translation().y(), 1.0e-4);
  EXPECT_NEAR(dest_t_src.z(), solution.dest_T_src.translation().z(), 1.0e-4);
  gtsam::Rot3 src_R_dest_gt(dest_R_src.transpose());
  double rot_error =
      gtsam::Rot3::Logmap(src_R_dest_gt * solution.dest_T_src.rotation()).norm();
  EXPECT_NEAR(0.0, rot_error, 1.0e-4);

  EXPECT_EQ(solution.inliers.size(), node_ids.size());
  for (const auto& correspondence : solution.inliers) {
    EXPECT_EQ(correspondence.first, correspondence.second);
  }
}

TEST_F(LayerRegistrationTests, TestSemanticRegistration) {
  size_t count = 0;
  for (const auto& id_node_pair : src_layer->nodes()) {
    uint8_t label = (count > (src_layer->numNodes() / 2)) ? 0 : 1;
    id_node_pair.second->attributes<SemanticNodeAttributes>().semantic_label = label;
    dest_layer->getNode(id_node_pair.first)
        .attributes<SemanticNodeAttributes>()
        .semantic_label = label;
    count++;
  }

  teaser::RobustRegistrationSolver::Params params;
  params.estimate_scaling = false;
  teaser::RobustRegistrationSolver solver(params);

  LayerRegistrationProblem problem;
  problem.src_nodes = node_ids;
  problem.dest_nodes = node_ids;
  problem.dest_layer = dest_layer.get();

  auto solution = registerDsgLayerSemantic(reg_config, solver, problem, *src_layer);

  ASSERT_TRUE(solution.valid);
  EXPECT_NEAR(dest_t_src.x(), solution.dest_T_src.translation().x(), 1.0e-4);
  EXPECT_NEAR(dest_t_src.y(), solution.dest_T_src.translation().y(), 1.0e-4);
  EXPECT_NEAR(dest_t_src.z(), solution.dest_T_src.translation().z(), 1.0e-4);
  gtsam::Rot3 src_R_dest_gt(dest_R_src.transpose());
  double rot_error =
      gtsam::Rot3::Logmap(src_R_dest_gt * solution.dest_T_src.rotation()).norm();
  EXPECT_NEAR(0.0, rot_error, 1.0e-4);

  EXPECT_EQ(solution.inliers.size(), node_ids.size());
  for (const auto& correspondence : solution.inliers) {
    EXPECT_EQ(correspondence.first, correspondence.second);
  }
}

TEST_F(GraphRegistrationTests, TestFullObjectRegistration) {
  DsgRegistrationInput match;
  for (int i = 0; i < src_points.cols(); ++i) {
    match.query_nodes.insert(NodeSymbol('O', i + src_points.cols()));
    match.match_nodes.insert(NodeSymbol('O', i));
  }

  match.query_root = NodeSymbol('p', src_points.cols());
  match.match_root = NodeSymbol('p', 0);

  teaser::RobustRegistrationSolver::Params params;
  reg_config.use_pairwise_registration = false;
  DsgTeaserSolver solver(DsgLayers::OBJECTS, reg_config, params);

  auto result = solver.solve(*dsg, match, NodeSymbol('a', 1));
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(NodeSymbol('a', 1), result.from_node);
  EXPECT_EQ(NodeSymbol('a', 0), result.to_node);
  EXPECT_NEAR(0.0, getPoseDistance(to_T_from, result), 1.0e-3);
}

TEST_F(GraphRegistrationTests, TestFullObjectRegistrationWithSubgraphExtraction) {
  DsgRegistrationInput match;
  for (int i = 0; i < src_points.cols(); ++i) {
    match.query_nodes.insert(NodeSymbol('O', i + src_points.cols()));
    match.match_nodes.insert(NodeSymbol('O', i));
  }

  match.query_root = NodeSymbol('p', src_points.cols());
  match.match_root = NodeSymbol('p', 0);

  teaser::RobustRegistrationSolver::Params params;
  reg_config.use_pairwise_registration = false;
  DsgTeaserSolver solver(DsgLayers::OBJECTS, reg_config, params);

  auto result = solver.solve(*dsg, match, NodeSymbol('a', 1));
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(NodeSymbol('a', 1), result.from_node);
  EXPECT_EQ(NodeSymbol('a', 0), result.to_node);
  EXPECT_NEAR(0.0, getPoseDistance(to_T_from, result), 1.0e-3);
}

TEST_F(GraphRegistrationTests, DISABLED_TestFullPlaceRegistration) {
  DsgRegistrationInput match;
  for (int i = 0; i < src_points.cols(); ++i) {
    match.query_nodes.insert(NodeSymbol('p', i + src_points.cols()));
    match.match_nodes.insert(NodeSymbol('p', i));
  }

  match.query_root = NodeSymbol('p', src_points.cols());
  match.match_root = NodeSymbol('p', 0);

  teaser::RobustRegistrationSolver::Params params;
  reg_config.use_pairwise_registration = true;
  reg_config.recreate_subgraph = true;
  // no connection between subgraphs, so large radius should get all nodes in the
  // original sets
  reg_config.subgraph_extraction.fixed_radius = true;
  reg_config.subgraph_extraction.max_radius_m = 500.0;
  DsgTeaserSolver solver(DsgLayers::PLACES, reg_config, params);

  auto result = solver.solve(*dsg, match, NodeSymbol('a', 1));
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(NodeSymbol('a', 1), result.from_node);
  EXPECT_EQ(NodeSymbol('a', 0), result.to_node);
  EXPECT_NEAR(0.0, getPoseDistance(to_T_from, result), 1.0e-3);

  solver.config.subgraph_extraction.max_radius_m = -1;
  auto invalid_result = solver.solve(*dsg, match, NodeSymbol('a', 1));
  EXPECT_FALSE(invalid_result.valid);
}

TEST_F(LayerRegistrationTests, TestRepeatedRegistration) {
  teaser::RobustRegistrationSolver::Params params;
  params.estimate_scaling = false;
  teaser::RobustRegistrationSolver solver(params);

  LayerRegistrationProblem problem;
  problem.src_nodes = node_ids;
  problem.dest_nodes = node_ids;
  problem.dest_layer = dest_layer.get();

  auto solution =
      registerDsgLayer(reg_config,
                       solver,
                       problem,
                       *src_layer,
                       [](const SceneGraphNode& src, const SceneGraphNode& dest) {
                         return src.id == dest.id;
                       });

  ASSERT_TRUE(solution.valid);

  std::list<NodeId> partial_list;
  for (size_t i = 0; i < 10; ++i) {
    partial_list.push_back(i);
  }

  LayerRegistrationProblem problem2;
  problem2.src_nodes = partial_list;
  problem2.dest_nodes = partial_list;
  problem2.dest_layer = dest_layer.get();

  solution =
      registerDsgLayer(reg_config,
                       solver,
                       problem2,
                       *src_layer,
                       [](const SceneGraphNode& src, const SceneGraphNode& dest) {
                         return src.id == dest.id;
                       });
  ASSERT_TRUE(solution.valid);
}

}  // namespace hydra::lcd
